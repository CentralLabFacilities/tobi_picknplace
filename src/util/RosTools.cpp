/*
 * RosTools.cpp
 *
 *  Created on: Jul 12, 2015
 *      Author: lziegler
 */

#include "RosTools.h"
#include "ParamReader.h"
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <grasp_viewer/DisplayGrasps.h>
#include <grasping_msgs/Object.h>

using namespace std;

static const string NAME = "RosTools";
const string OBJECT_NAME = "target_object";

RosTools::RosTools() {

    ROS_DEBUG_NAMED(NAME, "called");

    object_publisher = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
    object_att_publisher = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 1);
    grasps_marker = nh.advertise<visualization_msgs::MarkerArray>("grasps_marker", 10);
    grasps_marker_red = nh.advertise<visualization_msgs::MarkerArray>("grasps_marker_red", 10);
    grasps_marker_green = nh.advertise<visualization_msgs::MarkerArray>("grasps_marker_green", 10);
    grasps_marker_white = nh.advertise<visualization_msgs::MarkerArray>("grasps_marker_white", 10);

    clearOctomapClient = nh.serviceClient<std_srvs::Empty>("clear_octomap");

    std::string service = "/display_grasp";
    ROS_INFO_NAMED(NAME, "Wait for Service: %s", service.c_str());
    ros::service::waitForService(service);
    // TODO test if service was found
    grasp_viz_client = nh.serviceClient<grasp_viewer::DisplayGrasps>(service);

    scene_subscriber = nh.subscribe("planning_scene", 10, &RosTools::sceneCallback, this);
    scene_publisher =  nh.advertise<moveit_msgs::PlanningScene>("planning_scene",10);
}

RosTools::~RosTools() {
}

MoveResult RosTools::moveResultFromMoveit(
        moveit::planning_interface::MoveItErrorCode errorCode) {
    switch (errorCode.val) {
        case moveit_msgs::MoveItErrorCodes::SUCCESS:
            return SUCCESS;
        case moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN:
        case moveit_msgs::MoveItErrorCodes::PLANNING_FAILED:
            return NOPLAN;
        case moveit_msgs::MoveItErrorCodes::FAILURE:
        case moveit_msgs::MoveItErrorCodes::CONTROL_FAILED:
            return CRASH;
        case moveit_msgs::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
            return ENV_CHANGE;
        default:
            ROS_WARN_STREAM_NAMED(NAME, "unknown error code: " << errorCode.val);
            return OTHER;
    }
}

GraspReturnType::GraspResult RosTools::graspResultFromMoveit(
        moveit::planning_interface::MoveItErrorCode errorCode) {
    switch (errorCode.val) {
        case moveit_msgs::MoveItErrorCodes::SUCCESS:
            return GraspReturnType::SUCCESS;
        case moveit_msgs::MoveItErrorCodes::PLANNING_FAILED:
        case moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN:
            return GraspReturnType::POSITION_UNREACHABLE;
        case moveit_msgs::MoveItErrorCodes::FAILURE:
        case moveit_msgs::MoveItErrorCodes::CONTROL_FAILED:
            return GraspReturnType::ROBOT_CRASHED;
        case moveit_msgs::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
            return GraspReturnType::NO_RESULT;
        default:
            ROS_WARN_STREAM_NAMED(NAME, "unknown error code: " << errorCode.val);
            return GraspReturnType::FAIL;
    }
}

void RosTools::publish_collision_object(grasping_msgs::Object msg) {

    {
        boost::mutex::scoped_lock lock(sceneMutex);
        ROS_DEBUG_STREAM_NAMED(NAME, "Publish object with name " << msg.name);


        ParamReader &params = ParamReader::getParamReader();

        moveit_msgs::CollisionObject target_object;

        vector<geometry_msgs::Pose>::iterator poseIterator;
        vector<shape_msgs::SolidPrimitive>::iterator primIterator;

        for (primIterator = msg.primitives.begin(); primIterator != msg.primitives.end(); primIterator++) {
            target_object.primitives.push_back(*primIterator);
        }

        for (poseIterator = msg.primitive_poses.begin(); poseIterator != msg.primitive_poses.end(); poseIterator++) {
            target_object.primitive_poses.push_back(*poseIterator);
        }

        target_object.header.frame_id = msg.header.frame_id;
        target_object.id = msg.name;
        target_object.mesh_poses = msg.mesh_poses;
        target_object.meshes = msg.meshes;
        target_object.operation = target_object.ADD;

        moveit_msgs::PlanningScene update;
        update.is_diff = true;
        update.world.collision_objects.push_back(target_object);
        manipulationObjects.push_back(target_object);
        scene_publisher.publish(update);
    }
    ros::spinOnce();
    //object_publisher.publish(target_object);
    //ros::spinOnce();

    //manipulationObjects.push_back(target_object);
    ROS_DEBUG_STREAM_NAMED(NAME, "have " << manipulationObjects.size() );

}

void RosTools::clear_collision_objects(bool with_surface) {

    {
        boost::mutex::scoped_lock lock(sceneMutex);
        ROS_DEBUG_STREAM_NAMED(NAME, "clearing collision objects " << ((with_surface) ? "with surfaces" : "without surfaces"));
        moveit_msgs::PlanningScene update;

        if (with_surface) {
            update.is_diff = false;
        } else {
            update.is_diff = true;
            for (auto o : manipulationObjects) {
                if (o.id.find("surface") != std::string::npos) continue;
                moveit_msgs::CollisionObject object;
                object.operation = object.REMOVE;
                object.id = o.id;

                update.world.collision_objects.push_back(object);
            }

        }

        manipulationObjects.clear();
        scene_publisher.publish(update);
    }
    ros::spinOnce();


}

void RosTools::publish_grasps_as_markerarray(std::vector<moveit_msgs::Grasp> grasps, std::string color) {
    visualization_msgs::MarkerArray markers;
    int i = 0;

    for (std::vector<moveit_msgs::Grasp>::iterator it = grasps.begin(); it != grasps.end(); it++) {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = it->grasp_pose.header.frame_id;
        marker.id = i;
        marker.type = marker.ARROW;
        marker.ns = "graspmarker";
        marker.pose = it->grasp_pose.pose;
        marker.scale.x = -0.1;
        marker.scale.y = 0.002;
        marker.scale.z = 0.002;

        if (color == "red") {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        } else if (color == "green") {
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        } else if (color == "white") {
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
        } else {
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        }

        marker.color.a = 1.0;
        markers.markers.push_back(marker);
        i++;
    }

    if (color == "red") {
        grasps_marker_red.publish(markers);
    } else if (color == "green") {
        grasps_marker_green.publish(markers);
    } else if (color == "white") {
        grasps_marker_white.publish(markers);
    } else {

        grasps_marker.publish(markers);
    }
}

void RosTools::clear_grasps_markerarray() {

    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker marker;

    marker.action = 3; // DELETEALL
    markers.markers.push_back(marker);

    ROS_DEBUG_STREAM_NAMED(NAME, "Invoked clear_grasps_markerarray");
    grasps_marker_red.publish(markers);
    grasps_marker_green.publish(markers);
    grasps_marker_white.publish(markers);
    grasps_marker.publish(markers);
}

void RosTools::display_grasps(const std::vector<moveit_msgs::Grasp> &grasps) {

    grasp_viewer::DisplayGraspsRequest disp_req; //note: also possible to use displaygrasps.request...
    grasp_viewer::DisplayGraspsResponse disp_res;
    disp_req.grasps = grasps;
    grasp_viz_client.call(disp_req, disp_res);
}

void RosTools::clear_grasps() {

    ROS_DEBUG_STREAM_NAMED(NAME, "Invoked clear_grasps");
    grasp_viewer::DisplayGraspsRequest disp_req; //note: also possible to use displaygrasps.request...
    grasp_viewer::DisplayGraspsResponse disp_res;
    grasp_viz_client.call(disp_req, disp_res);
}

void RosTools::publish_place_locations_as_markerarray(std::vector<moveit_msgs::PlaceLocation> loc) {
    visualization_msgs::MarkerArray markers;
    int i = 0;

    ROS_DEBUG_STREAM_NAMED(NAME, "Display place locations.");
    for (std::vector<moveit_msgs::PlaceLocation>::iterator it = loc.begin(); it != loc.end(); it++) {

        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = ParamReader::getParamReader().frameArm;
        marker.id = i;
        marker.type = marker.ARROW;
        marker.ns = "placemarker";
        marker.pose = it->place_pose.pose;
        marker.scale.x = -0.1;
        marker.scale.y = 0.002;
        marker.scale.z = 0.002;
        marker.color.r = 1.0;
        marker.color.a = 1.0;

        markers.markers.push_back(marker);
        i++;
    }
    grasps_marker.publish(markers);
}

void RosTools::remove_collision_object(const string id) {

    moveit_msgs::CollisionObject object;
    moveit_msgs::PlanningScene update;

    {
        boost::mutex::scoped_lock lock(sceneMutex);
        ROS_INFO_STREAM_NAMED(NAME, "remove collision object " << id);

        for (auto it = manipulationObjects.begin(); it != end(manipulationObjects);) {
            if (it->id == id) it = manipulationObjects.erase(it);  // Returns the new iterator to continue from.
            else ++it;
        }


        update.is_diff = true;


        object.operation = object.REMOVE;
        object.id = id;
        update.world.collision_objects.push_back(object);
        scene_publisher.publish(update);
    }
    ros::spinOnce();

    update.world.collision_objects.clear();
    moveit_msgs::AttachedCollisionObject att;
    att.object = object;
    scene_publisher.publish(update);
    ros::spinOnce();

}

void RosTools::detach_collision_object() {

    {
        boost::mutex::scoped_lock lock(sceneMutex);
        ROS_DEBUG_STREAM_NAMED(NAME, "invoke: detach_collision_objects");

        moveit_msgs::PlanningScene update;
        update.is_diff = true;

        ROS_DEBUG_STREAM_NAMED(NAME, "detaching all known objects: " << manipulationObjects.size());
        for (auto o : manipulationObjects) {
            moveit_msgs::CollisionObject object;
            object.operation = object.REMOVE;
            object.id = o.id;

            moveit_msgs::AttachedCollisionObject att;
            att.object = object;
            update.robot_state.attached_collision_objects.push_back(att);
        }
        scene_publisher.publish(update);
    }
    ros::spinOnce();

}
void RosTools::attach_collision_object(moveit_msgs::AttachedCollisionObject& attached_object) {

    removeFromManipulationObjects(attached_object.object.id);
    manipulationObjects.push_back(attached_object.object);

    moveit_msgs::PlanningScene update;
    update.is_diff = true;
    update.robot_state.attached_collision_objects.push_back(attached_object);
    scene_publisher.publish(update);
    ros::spinOnce();

    //object_att_publisher.publish(attached_object);
    ros::spinOnce();

}

bool RosTools::has_attached_object() {

}

void RosTools::clear_octomap(double sleep_seconds) {
    std_srvs::Empty srv;

    if (!clearOctomapClient.call(srv))
        ROS_WARN("Failed to call clear octomap service.");

    ros::spinOnce();
    ros::WallDuration sleep_time(sleep_seconds);
    sleep_time.sleep();
}

void RosTools::sceneCallback(const moveit_msgs::PlanningScene& currentScene) {
    //ROS_DEBUG_STREAM_NAMED(NAME, "sceneCallback");
    boost::mutex::scoped_lock lock(sceneMutex);
    currentPlanningScene = currentScene;

    if(currentScene.is_diff) {
        for(auto o : currentScene.world.collision_objects) {
            for(auto it = manipulationObjects.begin(); it != end(manipulationObjects);) {
                if (it->id == o.id) it = manipulationObjects.erase(it);  // Returns the new iterator to continue from.
                else ++it;
            }

            if(o.operation == o.ADD || o.operation == o.MOVE) {
                manipulationObjects.push_back(o);
            } else if (o.operation == o.REMOVE) {
            }

        }

    } else {
        manipulationObjects.clear();
        for(auto o : currentScene.world.collision_objects) {
            manipulationObjects.push_back(o);
        }
    }

    //ROS_DEBUG_STREAM_NAMED(NAME, "sceneCallback done: (attached:)" << currentScene.robot_state.attached_collision_objects.size()  );
}

bool RosTools::getCollisionObjectByName(const std::string &id, moveit_msgs::CollisionObject &obj) {

    boost::mutex::scoped_lock lock(sceneMutex);
    ROS_DEBUG_STREAM_NAMED(NAME, "invoked: getCollisionObjectByName, have " << manipulationObjects.size() << " objects");

    for(auto o : manipulationObjects) {
        if(o.id == id) {
            obj = o;
            ROS_DEBUG_STREAM_NAMED(NAME, "Found CollisionObject with id" << o.id);
            return true;
        }
    }


    ROS_DEBUG_STREAM_NAMED(NAME, "Did not find CollisionObject with id:" << id << " all objects:");
    for(auto o : manipulationObjects) {
        ROS_DEBUG_STREAM_NAMED(NAME, "id" << o.id);
    }
    return false;
}

grasping_msgs::Object RosTools::convertMoveItToGrasping(moveit_msgs::CollisionObject obj) {

    grasping_msgs::Object msg;
    shape_msgs::Plane plane;

    ParamReader& params = ParamReader::getParamReader();

    vector<geometry_msgs::Pose>::iterator poseIterator;
    vector<shape_msgs::SolidPrimitive>::iterator primIterator;
    vector<shape_msgs::Plane>::iterator planeIterator;

    for (primIterator = obj.primitives.begin(); primIterator != obj.primitives.end(); primIterator++) {
        msg.primitives.push_back(*primIterator);
    }

    for (poseIterator = obj.primitive_poses.begin(); poseIterator != obj.primitive_poses.end(); poseIterator++) {
        msg.primitive_poses.push_back(*poseIterator);
    }

    //ROS_DEBUG_STREAM_NAMED(NAME, "CollisionObject planesize: " << obj.planes.size());

    //fill hight manually:
    moveit_msgs::CollisionObject collisionObjectArmCoords;

    tfTransformer.transform(obj, collisionObjectArmCoords,
            "base_link");
    ROS_DEBUG("Calculate tableHeight base_link");
    //ROS_DEBUG_STREAM_NAMED(NAME, obj);
    double tableHeightArmCoords =
            collisionObjectArmCoords.primitive_poses[0].position.z
            - collisionObjectArmCoords.primitives[0].dimensions[2]
            / 2.0;
    ROS_DEBUG_STREAM_NAMED(NAME, "tableHeight bl: " << tableHeightArmCoords);

    plane.coef[2] = 1;
    plane.coef[3] = -tableHeightArmCoords;

    msg.header.frame_id = params.frameArm;
    msg.name = obj.id;
    msg.mesh_poses = obj.mesh_poses;
    msg.meshes = obj.meshes;
    msg.surface = plane;

    return msg;
}

bool RosTools::getGraspingObjectByName(const std::string &name, grasping_msgs::Object &msg) {
    boost::mutex::scoped_lock lock(sceneMutex);

    for(auto o : manipulationObjects) {
        if(o.id == name) {
            msg = convertMoveItToGrasping(o);
            return true;
        }
    }

    return false;
}

bool RosTools::getCollisionObjectByHeight(const double &h, moveit_msgs::CollisionObject &obj, const std::regex e) {
    boost::mutex::scoped_lock lock(sceneMutex);

    double best = DBL_MAX;
    double cur;
    for(auto& o : manipulationObjects) {

        std::smatch m;
        bool hasMatch = std::regex_search(o.id, m, e);
        if( !hasMatch) continue;

        //todo hack, should search best primitive? getCollisionPrimitiveByHeigth?
        double oh = o.primitive_poses[0].position.z;
        if ((cur = std::abs(oh - h)) < best) {
            best = cur;
            obj = o;
        }
    }

    if (best < DBL_MAX) {
        return true;
    } else {
        return false;
    }
}

void RosTools::removeFromManipulationObjects(const std::string& id) {
    for(auto it = manipulationObjects.begin(); it != end(manipulationObjects);) {
        if (it->id == id) it = manipulationObjects.erase(it);  // Returns the new iterator to continue from.
        else ++it;
    }
}

/////ALT

string RosTools::getDefaultObjectName() const {

    return OBJECT_NAME;
}

void RosTools::publish_collision_object(const string &id, ObjectShape shape, double sleep_seconds) {

    ParamReader& params = ParamReader::getParamReader();

    //declare attached_object attribute
    moveit_msgs::CollisionObject target_object;
    target_object.id = id;
    target_object.header.frame_id = params.frameArm;

    // first remove any leftovers
    target_object.operation = target_object.REMOVE;
    object_publisher.publish(target_object);

    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.object.id = id;
    attached_object.object.operation = attached_object.object.REMOVE;
    object_att_publisher.publish(attached_object);

    ros::spinOnce();

    tfTransformer.transform(shape, shape, params.frameArm);

    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = shape.center.xMeter + 0.015;
    pose.position.y = shape.center.yMeter;
    pose.position.z = shape.center.zMeter;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = shape.heightMeter - 0.03;
    primitive.dimensions[1] = shape.widthMeter;
    primitive.dimensions[2] = shape.depthMeter;
    //	primitive.dimensions[1] = 0.02;
    //	primitive.dimensions[2] = 0.02;
    target_object.primitives.push_back(primitive);
    target_object.primitive_poses.push_back(pose);
    target_object.operation = target_object.ADD;
    target_object.header.frame_id = params.frameArm;
    object_publisher.publish(target_object);

    ROS_INFO("Publish collision object at %.3f,%.3f,%.3f (frame: %s) - h:%.3f w:%.3f d:%.3f",
            pose.position.x, pose.position.y, pose.position.z, params.frameArm.c_str(),
            shape.heightMeter, shape.widthMeter, shape.depthMeter);

    ros::spinOnce();

    clear_octomap(sleep_seconds);
}
