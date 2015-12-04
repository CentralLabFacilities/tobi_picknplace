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

using namespace std;
using namespace moveit;
using namespace actionlib;
using namespace moveit::planning_interface;

static const double DEFAULT_PLACE_HEIGHT = 0.15;

H2R5::H2R5() {

    lastHeightAboveTable = 0.0;

    touchlinks.push_back("palm_left");
    touchlinks.push_back("thumb0_left");
    touchlinks.push_back("thumb1_left");
    touchlinks.push_back("thumb2_left");
    touchlinks.push_back("index0_left");
    touchlinks.push_back("index1_left");
    touchlinks.push_back("index2_left");
    touchlinks.push_back("ring0_left");
    touchlinks.push_back("ring1_left");
    touchlinks.push_back("ring2_left");
    touchlinks.push_back("pinky0_left");
    touchlinks.push_back("pinky1_left");
    touchlinks.push_back("pinky2_left");

    frame = "palm_left";

    groupArm = new moveit::planning_interface::MoveGroup(ParamReader::getParamReader().groupArm);
    groupArm->setPlanningTime(120.0);
    groupArm->startStateMonitor();

    groupArm->setPoseReferenceFrame(ParamReader::getParamReader().frameOriginArm);

    groupArm->setGoalJointTolerance(0.01); //rad
    groupArm->setGoalPositionTolerance(0.02);  //m
    groupArm->setGoalOrientationTolerance(0.5); //rad

    groupEe = new moveit::planning_interface::MoveGroup(ParamReader::getParamReader().groupEef);
    groupEe->startStateMonitor();

    for(vector<string>::const_iterator it = groupEe->getActiveJoints().begin(); it != groupEe->getActiveJoints().end(); ++it) {
        printf("active joint '%s'\n", it->c_str());
    }

    pickActionClient.reset(
            new actionlib::SimpleActionClient<moveit_msgs::PickupAction>(nh,
                    move_group::PICKUP_ACTION, false));
    placeActionClient.reset(
            new actionlib::SimpleActionClient<moveit_msgs::PlaceAction>(nh,
                    move_group::PLACE_ACTION, false));

    waitForAction(pickActionClient, ros::Duration(0, 0), move_group::PICKUP_ACTION);
    waitForAction(placeActionClient, ros::Duration(0, 0), move_group::PLACE_ACTION);

    string output_scope = ParamReader::getParamReader().eefCmdScope;
    printf("> sending targets on '%s'\n", output_scope.c_str());
    target_publisher = nh.advertise<trajectory_msgs::JointTrajectory>(output_scope, 100);

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

    trajectory_msgs::JointTrajectory msg;
    msg.joint_names.push_back("left_hand_j0");
    msg.joint_names.push_back("left_hand_j1");
    msg.joint_names.push_back("left_hand_j2");
    msg.joint_names.push_back("left_hand_j3");
    msg.joint_names.push_back("left_hand_j4");

    trajectory_msgs::JointTrajectoryPoint p;
    p.positions.push_back(0.0);
    p.positions.push_back(0.0);
    p.positions.push_back(0.0);
    p.positions.push_back(0.0);
    p.positions.push_back(0.0);

    p.time_from_start = ros::Duration(1.2 * 1.0 / 50.0);

    msg.points.push_back(p);

    target_publisher.publish(msg);

}

void H2R5::closeEef(bool withSensors = false) {
    ROS_INFO("### Invoked closeGripper ###");

    trajectory_msgs::JointTrajectory msg;
    msg.joint_names.push_back("left_hand_j0");
    msg.joint_names.push_back("left_hand_j1");
    msg.joint_names.push_back("left_hand_j2");
    msg.joint_names.push_back("left_hand_j3");
    msg.joint_names.push_back("left_hand_j4");

    trajectory_msgs::JointTrajectoryPoint p;
    p.positions.push_back(0.0);
    p.positions.push_back(0.0);
    p.positions.push_back(143 * M_PI / 180.0);
    p.positions.push_back(143 * M_PI / 180.0);
    p.positions.push_back(143 * M_PI / 180.0);

    p.time_from_start = ros::Duration(1.2 * 1.0 / 50.0);

    msg.points.push_back(p);

    target_publisher.publish(msg);
}

GraspReturnType H2R5::graspObject(ObjectShape obj, bool simulate, const string &startPose) {

    ROS_INFO("### Invoked graspObject(ObjectShape) ###");

    if (obj.widthMeter > 0.1) {
        obj.widthMeter = 0.1;
    }
    if (obj.depthMeter > 0.1) {
        obj.depthMeter = 0.1;
    }

    ROS_INFO("Trying to pick object at %.3f, %.3f, %.3f (frame: %s).", obj.center.xMeter, obj.center.yMeter, obj.center.zMeter, obj.center.frame.c_str());
    string objId = rosTools.getDefaultObjectName();

    // publish collision object
    rosTools.publish_collision_object(objId, obj, 0.5);

    vector<moveit_msgs::Grasp> grasps = generate_grasps_angle_trans(obj);
    rosTools.publish_grasps_as_markerarray(grasps);

    ObjectShape objArmFrame;
    tfTransformer.transform(obj, objArmFrame, ParamReader::getParamReader().frameOriginArm);
    double tableHeightArmFrame = objArmFrame.center.xMeter - objArmFrame.heightMeter / 2.0;

    return Model::graspObject(objId, "", grasps, tableHeightArmFrame, simulate, startPose);
}

GraspReturnType H2R5::graspObject(const string &obj, const string &surface, bool simulate, const string &startPose) {

    ROS_INFO("### Invoked graspObject(string) ###");

    moveit_msgs::CollisionObject collisionObject;
    bool success = rosTools.getCollisionObjectByName(obj, collisionObject);

    GraspReturnType grt;
    if (!success) {
        ROS_WARN_STREAM("No object with id \"" << obj << "\" found in planning scene");
        grt.result = GraspReturnType::FAIL;
        return grt;
    }

    moveit_msgs::CollisionObject collisionObjectArmCoords;
    tfTransformer.transform(collisionObject, collisionObjectArmCoords, ParamReader::getParamReader().frameOriginArm);
    double tableHeightArmCoords = collisionObjectArmCoords.primitive_poses[0].position.x - collisionObjectArmCoords.primitives[0].dimensions[0] / 2.0;

    vector<moveit_msgs::Grasp> grasps = generate_grasps_angle_trans(collisionObject);
    rosTools.publish_grasps_as_markerarray(grasps);

    return Model::graspObject(obj, surface, grasps, tableHeightArmCoords, simulate, startPose);
}

GraspReturnType H2R5::placeObject(EefPose obj, bool simulate,
        const string &startPose) {
    ROS_INFO("### Invoked placeObject ###");

    vector<moveit_msgs::PlaceLocation> locations = generate_place_locations(obj);
    rosTools.publish_place_locations_as_markerarray(locations);

    return Model::placeObject("", locations, simulate, startPose);
}

GraspReturnType H2R5::placeObject(ObjectShape obj, bool simulate,
        const string &startPose) {
    ROS_INFO("### Invoked placeObject (bb) ###");

    vector<moveit_msgs::PlaceLocation> locations = generate_place_locations(obj);
    rosTools.publish_place_locations_as_markerarray(locations);

    return Model::placeObject("", locations, simulate, startPose);
}

GraspReturnType H2R5::placeObject(const string &obj, bool simulate, const string &startPose) {
    ROS_INFO("### Invoked placeObject (str) ###");
    ROS_ERROR("placing with surface string not suppported yet!");
    GraspReturnType grt;
    grt.result = GraspReturnType::FAIL;
    return grt;
}

std::vector<moveit_msgs::PlaceLocation> H2R5::generate_place_locations(
        EefPose obj) {

    tfTransformer.transform(obj, obj, ParamReader::getParamReader().frameOriginArm);

    ROS_INFO_STREAM(
            "generate_place_locations(): lastGraspPose:" << lastGraspPose << " - lastHeightAboveTable: " << lastHeightAboveTable);
    geometry_msgs::Quaternion orientMsg = lastGraspPose.pose.orientation;
    tf::Quaternion orientation = tf::Quaternion(orientMsg.x,orientMsg.y,orientMsg.z,orientMsg.w);
    if (orientation.w() == 0.0f && orientation.x() == 0.0f && orientation.y() == 0.0f
            && orientation.z() == 0.0f) {
        orientation = tf::createQuaternionFromRPY(0, -M_PI_2, 0);
    }

    Vec t = obj.translation;
//  if (lastHeightAboveTable == 0.0) {
//      t.xMeter += DEFAULT_PLACE_HEIGHT;
//  } else {
//      t.xMeter += lastHeightAboveTable;
//  }

    return graspGenerator.generate_placeloc_angle_trans(t.xMeter, t.yMeter, t.zMeter);

}

std::vector<moveit_msgs::PlaceLocation> H2R5::generate_place_locations(
        ObjectShape obj) {

    tfTransformer.transform(obj, obj, ParamReader::getParamReader().frameOriginArm);

    ROS_INFO_STREAM(
            "generate_place_locations(): lastGraspPose:" << lastGraspPose << " - lastHeightAboveTable: " << lastHeightAboveTable);
    geometry_msgs::Quaternion orientMsg = lastGraspPose.pose.orientation;
    tf::Quaternion orientation = tf::Quaternion(orientMsg.x,orientMsg.y,orientMsg.z,orientMsg.w);
    if (orientation.w() == 0.0f && orientation.x() == 0.0f && orientation.y() == 0.0f
            && orientation.z() == 0.0f) {
        orientation = tf::createQuaternionFromRPY(0, -M_PI_2, 0);
    }

    Vec t = obj.center;
    if (lastHeightAboveTable == 0.0) {
        t.xMeter += DEFAULT_PLACE_HEIGHT;
    } else {
        t.xMeter += lastHeightAboveTable;
    }

    return graspGenerator.generate_place_locations(t.xMeter, t.yMeter, t.zMeter, obj.widthMeter,
            obj.heightMeter, obj.depthMeter, orientation);

}

std::vector<moveit_msgs::Grasp> H2R5::generate_grasps_angle_trans(ObjectShape shape) {
    tfTransformer.transform(shape, shape, ParamReader::getParamReader().frameOriginArm);
    return graspGenerator.generate_grasps_angle_trans(shape.center.xMeter, shape.center.yMeter, shape.center.zMeter, shape.heightMeter);
}

std::vector<moveit_msgs::Grasp> H2R5::generate_grasps_angle_trans(moveit_msgs::CollisionObject shape) {
    tfTransformer.transform(shape, shape, ParamReader::getParamReader().frameOriginArm);
    return graspGenerator.generate_grasps_angle_trans(shape.primitive_poses[0].position.x, shape.primitive_poses[0].position.y, shape.primitive_poses[0].position.z, shape.primitives[0].dimensions[0]);
}

