/*
 * Katana.cpp
 *
 *  Created on: Apr 29, 2015
 *      Author: lziegler
 */

#include "Katana.h"

#include <ros/ros.h>
#include <moveit/move_group_pick_place_capability/capability_names.h>
#include <std_srvs/Empty.h>
#include <kdl/frames.hpp>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/common_planning_interface_objects/common_objects.h>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include <math.h>
#include <eigen3/Eigen/src/Core/Matrix.h>

using namespace std;
using namespace moveit;
using namespace actionlib;
using namespace moveit::planning_interface;

static const double DEFAULT_PLACE_HEIGHT = 0.15;

Katana::Katana() : Model() {

    sensor_subscriber = nh.subscribe("sensor_states", 1,
            &Katana::sensorCallback, this);

    movementActionClient.reset(
            new actionlib::SimpleActionClient<katana_msgs::JointMovementAction>(
            nh, "katana_arm_controller/joint_movement_action", false));

    rosTools.waitForAction(movementActionClient, ros::Duration(0, 0),
            "katana_arm_controller/joint_movement_action");

    ROS_INFO("KatanaModel: connected");
}

Katana::~Katana() {
    delete groupArm;
}

void Katana::setJointAngle(const string &joint, double angle) {

    if (!isSomethingInGripper())
        rosTools.detach_collision_object();

    Model::setJointAngle(joint, angle);
}

void Katana::setJointAngles(const map<string, double> &angles) {

    if (!isSomethingInGripper())
        rosTools.detach_collision_object();

    Model::setJointAngles(angles);
}

void Katana::setJointAngles(const vector<double> &angles) {

    if (!isSomethingInGripper())
        rosTools.detach_collision_object();

    Model::setJointAngles(angles);
}

int Katana::getNumJoints() const {
    ROS_DEBUG("Invoked getNumJoints");
    return groupArm->getJoints().size();
}

void Katana::openEef(bool withSensors) {
    ROS_INFO("### Invoked openGripper ###");
    moveToGripper(ParamReader::getParamReader().eefPosOpen.at(0), withSensors);
    rosTools.detach_collision_object();
}

void Katana::closeEef(bool withSensors) {
    ROS_INFO("### Invoked closeGripper ###");
    moveToGripper(ParamReader::getParamReader().eefPosClosed.at(0),
            withSensors);
    if (isSomethingInGripper())
        Model::attachDefaultObject();
}

void Katana::moveToGripper(double target, bool withSensors) {
    ROS_DEBUG("### Invoked moveToGripper ###");
    string actionName = "gripper_grasp_posture_controller";
    if (withSensors) {
        actionName = "gripper_grasp_posture_with_sensors_controller";
    }

    // TODO when we have a move sophisticated controller manager, this can be done with move_group.

    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> client(
            actionName, true);

    control_msgs::GripperCommandGoal goal;
    goal.command.position = target;

    client.waitForServer();
    client.sendGoal(goal);
    client.waitForResult(ros::Duration(5.0));

    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_DEBUG("Closing gripper succeeded");
    } else {
        ROS_ERROR("Closing gripper had an error");
    }
}

void Katana::motorsOn() {
    ROS_INFO("Invoked motorsOn");
    std_srvs::Empty empty;
    nh.serviceClient<std_srvs::Empty>("switch_motors_on").call(empty.request,
            empty.response);
}

void Katana::motorsOff() {
    ROS_INFO("Invoked motorsOff");
    std_srvs::Empty empty;
    nh.serviceClient<std_srvs::Empty>("switch_motors_off").call(empty.request,
            empty.response);
}

MoveResult Katana::moveTo(const EefPose& pose, bool linear, bool orientation) {
    if (!isSomethingInGripper()) {
        rosTools.detach_collision_object();
    }

    return Model::moveTo(pose, linear, orientation);
}

MoveResult Katana::moveTo(const std::string& poseName, bool plan) {
    ROS_INFO("### Invoked moveTo (string) ###");

    if (!isSomethingInGripper()) {
        rosTools.detach_collision_object();
    }

    if (plan) {
        ROS_INFO_STREAM("plan and execute path to: " << poseName);
        groupArm->clearPoseTargets();
        groupArm->setStartStateToCurrentState();
        groupArm->setNamedTarget(poseName);
        return rosTools.moveResultFromMoveit(groupArm->move());
    } else {
        katana_msgs::JointMovementGoal goal = buildMovementGoal(poseName);
        ROS_INFO_STREAM("send joint movement goal for pose: " << poseName);
        movementActionClient->sendGoal(goal);
        if (!movementActionClient->waitForResult(ros::Duration(10.0))) {
            ROS_WARN_STREAM("Movement action returned early");
            return OTHER;
        }
        if (movementActionClient->getState()
                == actionlib::SimpleClientGoalState::SUCCEEDED) {
            return SUCCESS;
        } else {
            return OTHER;
        }
    }
}

bool Katana::isSomethingInGripper() const {
    ROS_DEBUG("### Invoked isSomethingInGripper ###");
    vector<double> fingerJointAngles = groupEe->getCurrentJointValues();
    if (fingerJointAngles.empty()) {
        ROS_ERROR("Cannot read finger joint angles");
        return false;
    }
    ROS_DEBUG("finger joint 0: %f", fingerJointAngles[0]);

    boost::mutex::scoped_lock lock(sensorMutex);
    if (currentSensorReadings.empty()) {
        ROS_ERROR("Cannot read current sensor values");
        return false;
    }
    //	ROS_DEBUG("sensor right distance inside near: %d",
    //			currentSensorReadings.at("katana_r_inside_near_distance_sensor"));
    //	ROS_DEBUG("sensor right distance inside far: %d",
    //			currentSensorReadings.at("katana_r_inside_far_distance_sensor"));
    //	ROS_DEBUG("sensor left distance inside near: %d",
    //			currentSensorReadings.at("katana_l_inside_near_distance_sensor"));
    //	ROS_DEBUG("sensor left distance inside far: height%d",
    //			currentSensorReadings.at("katana_l_inside_far_distance_sensor"));
    ROS_DEBUG("sensor right force inside near: %d",
            currentSensorReadings.at("katana_r_inside_near_force_sensor"));
    ROS_DEBUG("sensor right force inside far: %d",
            currentSensorReadings.at("katana_r_inside_far_force_sensor"));
    ROS_DEBUG("sensor left force inside near: %d",
            currentSensorReadings.at("katana_l_inside_near_force_sensor"));
    ROS_DEBUG("sensor left force inside far: %d",
            currentSensorReadings.at("katana_l_inside_far_force_sensor"));

    bool force = currentSensorReadings.at("katana_r_inside_near_force_sensor")
            > ParamReader::getParamReader().gripperThresholdDistance
            || currentSensorReadings.at("katana_r_inside_far_force_sensor")
            > ParamReader::getParamReader().gripperThresholdDistance
            || currentSensorReadings.at("katana_l_inside_near_force_sensor")
            > ParamReader::getParamReader().gripperThresholdDistance
            || currentSensorReadings.at("katana_l_inside_far_force_sensor")
            > ParamReader::getParamReader().gripperThresholdDistance;

    //	bool distance = currentSensorReadings.at("katana_r_inside_near_distance_sensor")
    //			< GRIPPER_THRESHOLD_DISTANCE
    //			|| currentSensorReadings.at("katana_r_inside_far_distance_sensor")
    //					< GRIPPER_THRESHOLD_DISTANCE
    //			|| currentSensorReadings.at("katana_l_inside_near_distance_sensor")
    //					< GRIPPER_THRESHOLD_DISTANCE
    //			|| currentSensorReadings.at("katana_l_inside_far_distance_sensor")
    //					< GRIPPER_THRESHOLD_DISTANCE;

    bool gripperClosed = fabs(
            fingerJointAngles[0] - ParamReader::getParamReader().eefPosClosed[0])
            < 0.05;
    bool gripperNearClosed = fabs(
            fingerJointAngles[0] - ParamReader::getParamReader().eefPosClosed[0])
            < 0.15;

    //	return (force && !gripperClosed) || (distance && !gripperNearClosed);
    return (force && !gripperClosed);
}

map<string, short> Katana::getGripperSensors() const {
    ROS_DEBUG("Invoked getGripperSensors");
    boost::mutex::scoped_lock lock(sensorMutex);
    return currentSensorReadings;
}

/**
GraspReturnType Katana::graspObject(ObjectShape obj, bool simulate,
        const string &startPose) {

    ROS_INFO("### Invoked graspObject(ObjectShape) ###");

    ROS_INFO("Trying to pick object at %.3f, %.3f, %.3f (frame: %s).",
            obj.center.xMeter, obj.center.yMeter, obj.center.zMeter,
            obj.center.frame.c_str());
    string objId = rosTools.getDefaultObjectName();

    // publish collision object
    rosTools.publish_collision_object(objId, obj, 0.5);

    vector<moveit_msgs::Grasp> grasps = graspGenerator->generate_grasps(obj);

    for (moveit_msgs::Grasp &i : grasps)
        fillGrasp(i);

    rosTools.publish_grasps_as_markerarray(grasps);

    return Model::graspObject(objId, "", grasps, simulate,
            startPose);
}**/

GraspReturnType Katana::graspObject(const string &obj, const string &surface,
        bool simulate, const string &startPose) {

    ROS_INFO("### Invoked graspObject(string) ###");

    moveit_msgs::CollisionObject collisionObject;

    bool success = rosTools.getCollisionObjectByName(obj, collisionObject);

    GraspReturnType grt;
    if (!success) {
        ROS_WARN_STREAM("No object with id \"" << obj << "\" found in planning scene");
        grt.result = GraspReturnType::FAIL;
        return grt;
    }
    ROS_DEBUG_STREAM("grasping Object:\n" << collisionObject);

    vector<moveit_msgs::Grasp> grasps = graspGenerator->generate_grasps(
            collisionObject);

    shape_msgs::SolidPrimitive primitive;

    if (collisionObject.primitives[0].type == primitive.CYLINDER) {
        grasps = augmentCylinderGrasps(collisionObject, grasps);
    } else if (collisionObject.primitives[0].type == primitive.BOX) {
        grasps = augmentBoxGrasps(collisionObject, grasps);
    } else if (collisionObject.primitives[0].type == primitive.SPHERE) {
        grasps = augmentSphereGrasps(collisionObject, grasps);
    }

    /* incoming grasps represent the chosen grasp_frame for each grasp in given frame-id=reference frame
    * transformation is needed to find what is the equivalent end-effector frame pose to go to
    * (ee frame = last link of arm chain or = first link of eef ) in the reference frame
    * that would reach such a grasp_frame.
    * If ee frame = grasp_frame no transform needed
    */
    std::string grasp_frame = ParamReader::getParamReader().frameGripper;
    std::string ee_frame = groupArm->getEndEffectorLink();
    if (grasp_frame == ee_frame)
    {
      ROS_INFO_STREAM("grasp_frame '" << grasp_frame << "' is equal to ee_frame '" << ee_frame << "' no transformation needed to all grasps");
      for (moveit_msgs::Grasp &i : grasps)
      {
          // fill the postures
          fillGrasp(i);
      }
    }
    else
    {
      ROS_INFO_STREAM("grasp_frame '" << grasp_frame << " is not equal to ee_frame '" << ee_frame << "' transformation will be applied to all grasps");
      for (moveit_msgs::Grasp &i : grasps)
      {
          tfTransformer.transformLink(i, i, grasp_frame, ee_frame);
          // fill the postures
          fillGrasp(i);
      }
    }

    rosTools.publish_grasps_as_markerarray(grasps);

    //closeEef(false);
    auto ret = Model::graspObject(obj, surface, grasps, simulate, startPose);
    if (ret.result == GraspReturnType::ROBOT_CRASHED) {
        ROS_INFO_STREAM("  try to RECOVER !!");
        if (isSomethingInGripper()) {
            if (moveTo("carry_side", true)) {
                ret.result = GraspReturnType::COLLISION_HANDLED;
            }
        } else {
            if (moveTo("fold_up", true)) {
                ret.result = GraspReturnType::SUCCESS;
            }
        }
    }
    return ret;
}

GraspReturnType Katana::placeObject(EefPose obj, bool simulate,
        const string &startPose) {
    ROS_INFO("### Invoked placeObject ###");

    vector<moveit_msgs::PlaceLocation> locations = generate_place_locations(
            obj);
    rosTools.publish_place_locations_as_markerarray(locations);

    return Model::placeObject(graspedObjectID, "", locations, simulate, startPose);
}

GraspReturnType Katana::placeObject(ObjectShape obj, bool simulate,
        const string &startPose) {
    ROS_INFO("### Invoked placeObject (bb) ###");

    vector<moveit_msgs::PlaceLocation> locations = generate_place_locations(
            obj);
    rosTools.publish_place_locations_as_markerarray(locations);

    return Model::placeObject(graspedObjectID, "", locations, simulate, startPose);
}

GraspReturnType Katana::placeObject(const string &surface, bool simulate,
        const string &startPose) {

    ROS_INFO_STREAM("### Invoked placeObject Surface (str) ### :" << surface);

    vector<moveit_msgs::PlaceLocation> locations = Model::generate_place_locations(surface);
    rosTools.publish_place_locations_as_markerarray(locations);

    return Model::placeObject(graspedObjectID, surface, locations, simulate, startPose);
}

std::vector<moveit_msgs::PlaceLocation> Katana::generate_place_locations(
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
    //	if (lastHeightAboveTable == 0.0) {
    //		t.xMeter += DEFAULT_PLACE_HEIGHT;
    //	} else {
    //		t.xMeter += lastHeightAboveTable;
    //	}

    return graspGenerator->generate_placeloc_angle_trans(t.xMeter, t.yMeter,
            t.zMeter);

}

std::vector<moveit_msgs::PlaceLocation> Katana::generate_place_locations(
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

void Katana::sensorCallback(const sensor_msgs::JointStatePtr& sensorReadings) {
    boost::mutex::scoped_lock lock(sensorMutex);
    for (int i = 0; i < sensorReadings->name.size(); i++) {
        string sensor = sensorReadings->name[i];
        double reading = sensorReadings->position[i];
        currentSensorReadings[sensor] = reading;
    }
}

katana_msgs::JointMovementGoal Katana::buildMovementGoal(
        const std::string &poseName) {
    ArmPose pose = getRememberedPose(poseName);
    katana_msgs::JointMovementGoal goal;
    for (ArmPose::iterator i = pose.begin(); i != pose.end(); ++i) {
        goal.jointGoal.name.push_back(i->first);
        goal.jointGoal.position.push_back(i->second);
    }
    return goal;
}

trajectory_msgs::JointTrajectory Katana::generate_close_eef_msg() {
    ParamReader& params = ParamReader::getParamReader();
    trajectory_msgs::JointTrajectory msg;

    msg.joint_names.push_back("katana_l_finger_joint");
    msg.joint_names.push_back("katana_r_finger_joint");
    msg.points.resize(1);
    msg.points[0].positions.push_back(
            params.eefPosClosed.at(0));
    msg.points[0].positions.push_back(
            params.eefPosClosed.at(0));

    return msg;
}

trajectory_msgs::JointTrajectory Katana::generate_open_eef_msg() {
    ParamReader& params = ParamReader::getParamReader();
    trajectory_msgs::JointTrajectory msg;

    msg.joint_names.push_back("katana_l_finger_joint");
    msg.joint_names.push_back("katana_r_finger_joint");
    msg.points.resize(1);
    msg.points[0].positions.push_back(
            params.eefPosOpen.at(0));
    msg.points[0].positions.push_back(
            params.eefPosOpen.at(0));

    return msg;
}

vector<moveit_msgs::Grasp> Katana::augmentCylinderGrasps(moveit_msgs::CollisionObject collisionObject, vector<moveit_msgs::Grasp> graspsIn){
    ROS_INFO_STREAM("Generate Grasps for Cylinder");

    vector<moveit_msgs::Grasp> newGrasps = graspsIn;
    //vary side grasps around y axis of grasp frame
    for (moveit_msgs::Grasp &i : graspsIn) {
        //only vary side grasps around y
        if(i.id.find("Side") == std::string::npos){
            continue;
        }

        Eigen::Vector3d graspY(0.0,1.0,0.0);
        Eigen::Quaterniond graspOrientation(i.grasp_pose.pose.orientation.w,
                i.grasp_pose.pose.orientation.x,
                i.grasp_pose.pose.orientation.y,
                i.grasp_pose.pose.orientation.z);
        graspY = graspOrientation * graspY;

        Eigen::Vector3d graspPos;
        tf2::fromMsg(i.grasp_pose.pose.position, graspPos);

        double vary = -0.2;
        for (; vary <= 0.2; vary = vary + 0.1) {
            if(vary == 0.0){
                continue;
            }

            moveit_msgs::Grasp augGrasp;
            augGrasp = rotateGrasp(i,vary,graspY,graspPos);

            augGrasp.id = i.id + "_augY" + std::to_string(vary);
            newGrasps.push_back(augGrasp);
        }
    }

    //vary side grasps around the z-axis of the object
    //determine z axis of object
    Eigen::Vector3d objectZ(0.0,0.0,1.0);
    Eigen::Quaterniond objectOrientation(collisionObject.primitive_poses[0].orientation.w,
            collisionObject.primitive_poses[0].orientation.x,
            collisionObject.primitive_poses[0].orientation.y,
            collisionObject.primitive_poses[0].orientation.z);
    objectZ = objectOrientation * objectZ;

    Eigen::Vector3d objectPos;
    tf2::fromMsg(collisionObject.primitive_poses[0].position, objectPos);

    graspsIn = newGrasps;
    for (moveit_msgs::Grasp &i : graspsIn) {
        //only vary side grasps
        if(i.id.find("Side") == std::string::npos){
            continue;
        }

        double vary = -0.4;
        for (; vary <= 0.4; vary = vary + 0.2) {
            if(vary == 0.0){
                continue;
            }

            moveit_msgs::Grasp augGrasp;
            augGrasp = rotateGrasp(i,vary,objectZ, objectPos);

            augGrasp.id = i.id + "_augZ" + std::to_string(vary);
            newGrasps.push_back(augGrasp);
        }
    }

    //vary top grasps around the z axis of the grasp
    graspsIn = newGrasps;
    for (moveit_msgs::Grasp &i : graspsIn) {
        //only vary top grasps around y
        if(i.id.find("Top") == std::string::npos){
            continue;
        }

        Eigen::Vector3d graspZ(0.0,0.0,1.0);
        Eigen::Quaterniond graspOrientation(i.grasp_pose.pose.orientation.w,
                i.grasp_pose.pose.orientation.x,
                i.grasp_pose.pose.orientation.y,
                i.grasp_pose.pose.orientation.z);
        graspZ = graspOrientation * graspZ;

        Eigen::Vector3d graspPos;
        tf2::fromMsg(i.grasp_pose.pose.position, graspPos);

        double vary = -0.5;
        for (; vary <= 0.5; vary = vary + 0.1) {
            if(vary == 0.0){
                continue;
            }

            moveit_msgs::Grasp augGrasp;
            augGrasp = rotateGrasp(i,vary,graspZ,graspPos);

            augGrasp.id = i.id + "_augZ" + std::to_string(vary);
            newGrasps.push_back(augGrasp);
        }
    }

    //vary top grasps around the y axis of the grasp
    graspsIn = newGrasps;
    for (moveit_msgs::Grasp &i : graspsIn) {
        //only vary top grasps around y
        if(i.id.find("Top") == std::string::npos){
            continue;
        }

        Eigen::Vector3d graspY(0.0,1.0,0.0);
        Eigen::Quaterniond graspOrientation(i.grasp_pose.pose.orientation.w,
                i.grasp_pose.pose.orientation.x,
                i.grasp_pose.pose.orientation.y,
                i.grasp_pose.pose.orientation.z);
        graspY = graspOrientation * graspY;

        Eigen::Vector3d graspPos;
        tf2::fromMsg(i.grasp_pose.pose.position, graspPos);

        double vary = -0.8;
        for (; vary <= 0.8; vary = vary + 0.1) {
            if(vary == 0.0){
                continue;
            }

            moveit_msgs::Grasp augGrasp;
            augGrasp = rotateGrasp(i,vary,graspY,graspPos);

            augGrasp.id = i.id + "_augY" + std::to_string(vary);
            newGrasps.push_back(augGrasp);
        }
    }

    //rotate all grasps around z by 180 degree to allow different wrist positions
    graspsIn = newGrasps;
    for (moveit_msgs::Grasp &i : graspsIn) {

        Eigen::Vector3d graspZ(0.0,0.0,1.0);
        Eigen::Quaterniond graspOrientation(i.grasp_pose.pose.orientation.w,
                i.grasp_pose.pose.orientation.x,
                i.grasp_pose.pose.orientation.y,
                i.grasp_pose.pose.orientation.z);
        graspZ = graspOrientation * graspZ;

        Eigen::Vector3d graspPos;
        tf2::fromMsg(i.grasp_pose.pose.position, graspPos);

        moveit_msgs::Grasp augGrasp;
        augGrasp = rotateGrasp(i,M_PI,graspZ,graspPos);

        augGrasp.id = i.id + "_rot180";
        newGrasps.push_back(augGrasp);
    }

    return newGrasps;
}

vector<moveit_msgs::Grasp> Katana::augmentBoxGrasps(moveit_msgs::CollisionObject collisionObject, vector<moveit_msgs::Grasp> graspsIn){
    ROS_INFO_STREAM("Generate Grasps for Box");

    vector<moveit_msgs::Grasp> newGrasps = graspsIn;
    //TODO: vary side grasps along z axis of the object

    //vary all grasps around the y axis of the grasp
    graspsIn = newGrasps;
    for (moveit_msgs::Grasp &i : graspsIn) {

        Eigen::Vector3d graspY(0.0,1.0,0.0);
        Eigen::Quaterniond graspOrientation(i.grasp_pose.pose.orientation.w,
                i.grasp_pose.pose.orientation.x,
                i.grasp_pose.pose.orientation.y,
                i.grasp_pose.pose.orientation.z);
        graspY = graspOrientation * graspY;

        Eigen::Vector3d graspPos;
        tf2::fromMsg(i.grasp_pose.pose.position, graspPos);

        if(i.id.find("Top") != std::string::npos){
            double vary = -0.8;
            for (; vary <= 0.8; vary = vary + 0.1) {
                if(vary == 0.0){
                    continue;
                }

                moveit_msgs::Grasp augGrasp;
                augGrasp = rotateGrasp(i,vary,graspY,graspPos);

                augGrasp.id = i.id + "_augY" + std::to_string(vary);
                newGrasps.push_back(augGrasp);
            }
        } else {
            double vary = -0.2;
            for (; vary <= 0.2; vary = vary + 0.1) {
                if(vary == 0.0){
                    continue;
                }

                moveit_msgs::Grasp augGrasp;
                augGrasp = rotateGrasp(i,vary,graspY,graspPos);

                augGrasp.id = i.id + "_augY" + std::to_string(vary);
                newGrasps.push_back(augGrasp);
            }
        }
    }

    //rotate all grasps around z by 180 degree to allow different wrist positions
    graspsIn = newGrasps;
    for (moveit_msgs::Grasp &i : graspsIn) {

        Eigen::Vector3d graspZ(0.0,0.0,1.0);
        Eigen::Quaterniond graspOrientation(i.grasp_pose.pose.orientation.w,
                i.grasp_pose.pose.orientation.x,
                i.grasp_pose.pose.orientation.y,
                i.grasp_pose.pose.orientation.z);
        graspZ = graspOrientation * graspZ;

        Eigen::Vector3d graspPos;
        tf2::fromMsg(i.grasp_pose.pose.position, graspPos);

        moveit_msgs::Grasp augGrasp;
        augGrasp = rotateGrasp(i,M_PI,graspZ,graspPos);

        augGrasp.id = i.id + "_rot180";
        newGrasps.push_back(augGrasp);
    }

    return newGrasps;
}

vector<moveit_msgs::Grasp> Katana::augmentSphereGrasps(moveit_msgs::CollisionObject collisionObject, vector<moveit_msgs::Grasp> graspsIn){
    ROS_INFO_STREAM("Generate Grasps for Sphere");

    //TODO: explore sensible variations of sphere grasps
    return graspsIn;
}

moveit_msgs::Grasp Katana::rotateGrasp(moveit_msgs::Grasp grasp, double angle, Eigen::Vector3d axis, Eigen::Vector3d rotationCenter) {
    Eigen::Affine3d graspPose;
    tf2::fromMsg(grasp.grasp_pose.pose, graspPose);

    Eigen::Affine3d A = Eigen::Translation3d(rotationCenter) * Eigen::AngleAxisd(angle, axis) * Eigen::Translation3d(-rotationCenter);
    moveit_msgs::Grasp augGrasp;
    A = A * graspPose;
    augGrasp.grasp_pose.pose = Eigen::toMsg(A);

    augGrasp.grasp_pose.header.frame_id = grasp.grasp_pose.header.frame_id;
    return augGrasp;
}
