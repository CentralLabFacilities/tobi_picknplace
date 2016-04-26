/*
 * H2R5.cpp
 *
 *  Created on: Nov 30, 2015
 *      Author: plueckin
 */

#include "H2R5.h"

#include <ros/ros.h>
#include <moveit/move_group_pick_place_capability/capability_names.h>
#include <std_srvs/Empty.h>
#include <kdl/frames.hpp>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/common_planning_interface_objects/common_objects.h>
#include <boost/algorithm/string.hpp>

#include "../../grasping/CentroidGrasping.h"
#include "../../interface/AGNIInterface.h"

using namespace std;
using namespace moveit;
using namespace actionlib;
using namespace moveit::planning_interface;

static const double DEFAULT_PLACE_HEIGHT = 0.15;

H2R5::H2R5() :
        Model() {
    string group = ParamReader::getParamReader().groupArm;
    string substr;

    if (group.find("left") != std::string::npos)
        substr = "left";
    else
        substr = "right";

    string output_scope = ParamReader::getParamReader().eefCmdScope;
    printf("> sending targets on '%s'\n", output_scope.c_str());
    target_publisher = nh.advertise<trajectory_msgs::JointTrajectory>(
            output_scope, 100);

    ROS_INFO("H2R5Model: connected");

}

H2R5::~H2R5() {
    delete groupArm;
}

MoveResult H2R5::moveTo(const std::string& poseName, bool plan) {
    ROS_INFO("### Invoked moveTo (string) ###");

    if (plan) {
        ROS_INFO_STREAM("plan and execute path to: " << poseName);
        groupArm->clearPoseTargets();
        groupArm->setStartStateToCurrentState();
        groupArm->setNamedTarget(poseName);
        return rosTools.moveResultFromMoveit(groupArm->move());
    } else {
        ROS_ERROR("H2R5 moveTo currently only with plan param set to true!");
        return NOPLAN;
    }
}

void H2R5::openEef(bool withSensors = false) {
    ROS_INFO("### Invoked openGripper ###");

    target_publisher.publish(generate_open_eef_msg());
}

void H2R5::closeEef(bool withSensors = false) {
    ROS_INFO("### Invoked closeGripper ###");

    target_publisher.publish(generate_close_eef_msg());
}

GraspReturnType H2R5::graspObject(ObjectShape obj, bool simulate,
        const string &startPose) {

    ROS_INFO("### Invoked graspObject(ObjectShape) ###");

    if (obj.widthMeter > 0.1) {
        obj.widthMeter = 0.1;
    }
    if (obj.depthMeter > 0.1) {
        obj.depthMeter = 0.1;
    }

    ROS_INFO("Trying to pick object at %.3f, %.3f, %.3f (frame: %s).",
            obj.center.xMeter, obj.center.yMeter, obj.center.zMeter,
            obj.center.frame.c_str());
    string objId = rosTools.getDefaultObjectName();

    // publish collision object NOT FOR h2r5! use agni vis.
    //rosTools.publish_collision_object(objId, obj, 0.5);

    vector<moveit_msgs::Grasp> grasps;

    if(graspGenerator->getName() == CENTROID_GRASP_NAME) {
        tfTransformer.transform(obj, obj, ParamReader::getParamReader().frameArm);
        grasps = graspGenerator->generate_grasps(obj);
    } else { //agni
        grasps = graspGenerator->generate_grasps(obj);
        //todo: do we have to do a transformation?
    }

    //fill up with pre and post grasp postures, model specific!
    for (moveit_msgs::Grasp &i : grasps)
        fillGrasp(i);

    //rosTools.publish_grasps_as_markerarray(grasps);

    ObjectShape objArmFrame;
    tfTransformer.transform(obj, objArmFrame,
            ParamReader::getParamReader().frameArm);
    double tableHeightArmFrame = objArmFrame.center.xMeter
            - objArmFrame.heightMeter / 2.0;

    return Model::graspObject(objId, "", grasps, tableHeightArmFrame, simulate,
            startPose);
}

GraspReturnType H2R5::graspObject(const string &obj, const string &surface,
        bool simulate, const string &startPose) {

    ROS_INFO("### Invoked graspObject(string) ###");

    moveit_msgs::CollisionObject collisionObject;
    bool success = rosTools.getCollisionObjectByName(obj, collisionObject);

    GraspReturnType grt;
    if (!success) {
        ROS_WARN_STREAM(
                "No object with id \"" << obj << "\" found in planning scene");
        grt.result = GraspReturnType::FAIL;
        return grt;
    }

    moveit_msgs::CollisionObject collisionObjectArmCoords;
    tfTransformer.transform(collisionObject, collisionObjectArmCoords,
            ParamReader::getParamReader().frameArm);
    double tableHeightArmCoords =
            collisionObjectArmCoords.primitive_poses[0].position.x
                    - collisionObjectArmCoords.primitives[0].dimensions[0]
                            / 2.0;

    vector<moveit_msgs::Grasp> grasps;

    if(graspGenerator->getName() == CENTROID_GRASP_NAME) {
        tfTransformer.transform(collisionObject, collisionObject,
                    ParamReader::getParamReader().frameArm);
        grasps = graspGenerator->generate_grasps(collisionObject);
    } else { //agni
        grasps = graspGenerator->generate_grasps(obj);
        //todo: do we have to do a transformation?
    }

    //rosTools.publish_grasps_as_markerarray(grasps);
    return Model::graspObject(obj, surface, grasps, tableHeightArmCoords,
            simulate, startPose);
}

GraspReturnType H2R5::placeObject(EefPose obj, bool simulate,
        const string &startPose) {
    ROS_INFO("### Invoked placeObject ###");

    vector<moveit_msgs::PlaceLocation> locations = generate_place_locations(
            obj);
    rosTools.publish_place_locations_as_markerarray(locations);

    return Model::placeObject("", locations, simulate, startPose);
}

GraspReturnType H2R5::placeObject(ObjectShape obj, bool simulate,
        const string &startPose) {
    ROS_INFO("### Invoked placeObject (bb) ###");

    vector<moveit_msgs::PlaceLocation> locations = generate_place_locations(
            obj);
    rosTools.publish_place_locations_as_markerarray(locations);

    return Model::placeObject("", locations, simulate, startPose);
}

GraspReturnType H2R5::placeObject(const string &obj, bool simulate,
        const string &startPose) {
    ROS_INFO("### Invoked placeObject (str) ###");
    ROS_ERROR("placing with surface string not suppported yet!");
    GraspReturnType grt;
    grt.result = GraspReturnType::FAIL;
    return grt;
}

std::vector<moveit_msgs::PlaceLocation> H2R5::generate_place_locations(
        EefPose obj) {

    tfTransformer.transform(obj, obj, ParamReader::getParamReader().frameArm);

    ROS_INFO_STREAM(
            "generate_place_locations(): lastGraspPose:" << lastGraspPose << " - lastHeightAboveTable: " << lastHeightAboveTable);
    geometry_msgs::Quaternion orientMsg = lastGraspPose.pose.orientation;
    tf::Quaternion orientation = tf::Quaternion(orientMsg.x, orientMsg.y,
            orientMsg.z, orientMsg.w);
    if (orientation.w() == 0.0f && orientation.x() == 0.0f
            && orientation.y() == 0.0f && orientation.z() == 0.0f) {
        orientation = tf::createQuaternionFromRPY(0, -M_PI_2, 0);
    }

    Vec t = obj.translation;
//  if (lastHeightAboveTable == 0.0) {
//      t.xMeter += DEFAULT_PLACE_HEIGHT;
//  } else {
//      t.xMeter += lastHeightAboveTable;
//  }

    return graspGenerator->generate_placeloc_angle_trans(t.xMeter, t.yMeter,
            t.zMeter);

}

std::vector<moveit_msgs::PlaceLocation> H2R5::generate_place_locations(
        ObjectShape obj) {

    tfTransformer.transform(obj, obj, ParamReader::getParamReader().frameArm);

    ROS_INFO_STREAM(
            "generate_place_locations(): lastGraspPose:" << lastGraspPose << " - lastHeightAboveTable: " << lastHeightAboveTable);
    geometry_msgs::Quaternion orientMsg = lastGraspPose.pose.orientation;
    tf::Quaternion orientation = tf::Quaternion(orientMsg.x, orientMsg.y,
            orientMsg.z, orientMsg.w);
    if (orientation.w() == 0.0f && orientation.x() == 0.0f
            && orientation.y() == 0.0f && orientation.z() == 0.0f) {
        orientation = tf::createQuaternionFromRPY(0, -M_PI_2, 0);
    }

    Vec t = obj.center;
    if (lastHeightAboveTable == 0.0) {
        t.xMeter += DEFAULT_PLACE_HEIGHT;
    } else {
        t.xMeter += lastHeightAboveTable;
    }

    return graspGenerator->generate_place_locations(t.xMeter, t.yMeter, t.zMeter,
            obj.widthMeter, obj.heightMeter, obj.depthMeter, orientation);

}

//todo: generalize
trajectory_msgs::JointTrajectory H2R5::generate_close_eef_msg() {
    trajectory_msgs::JointTrajectory msg;
    trajectory_msgs::JointTrajectoryPoint p;

    vector<double> pos_close = ParamReader::getParamReader().eefPosClosed;
    for (uint i = 0; i < groupEe->getActiveJoints().size(); i++) {
       msg.joint_names.push_back(groupEe->getActiveJoints().at(i));
       p.positions.push_back(pos_close.at(i));
    }

    p.time_from_start = ros::Duration(1.0); //sec

    msg.points.push_back(p);

    return msg;
}

trajectory_msgs::JointTrajectory H2R5::generate_open_eef_msg() {
    trajectory_msgs::JointTrajectory msg;
    trajectory_msgs::JointTrajectoryPoint p;

    vector<double> pos_open = ParamReader::getParamReader().eefPosOpen;
    for (uint i = 0; i < groupEe->getActiveJoints().size(); i++) {
        msg.joint_names.push_back(groupEe->getActiveJoints().at(i));
        p.positions.push_back(pos_open.at(i));
    }

    p.time_from_start = ros::Duration(1.0);

    msg.points.push_back(p);

    return msg;
}

void H2R5::fillGrasp(moveit_msgs::Grasp& grasp) {

    ParamReader& params = ParamReader::getParamReader();

    grasp.pre_grasp_approach.direction.vector.z = 1.0;
    grasp.pre_grasp_approach.direction.header.stamp = ros::Time::now();
    grasp.pre_grasp_approach.direction.header.frame_id = params.frameGripper;
    grasp.pre_grasp_approach.min_distance = params.approachMinDistance;
    grasp.pre_grasp_approach.desired_distance = params.approachDesiredDistance;

    // direction: lift up
    grasp.post_grasp_retreat.direction.vector.z = 1.0;
    grasp.post_grasp_retreat.direction.header.stamp = ros::Time::now();
    grasp.post_grasp_retreat.direction.header.frame_id = params.frameArm; //base_link!
    grasp.post_grasp_retreat.min_distance = params.liftUpMinDistance;
    grasp.post_grasp_retreat.desired_distance = params.liftUpDesiredDistance;

    // open on approach and close when reached
    if (grasp.pre_grasp_posture.points.size()==0)
    {
        grasp.pre_grasp_posture = generate_open_eef_msg();
    }
    if (grasp.grasp_posture.points.size()==0)
    {
        grasp.grasp_posture = generate_close_eef_msg();
    }

}
