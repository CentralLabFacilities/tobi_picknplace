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

using namespace std;
using namespace moveit;
using namespace actionlib;
using namespace moveit::planning_interface;

static const string DEFAULT_MG_ARM = "arm"; //todo: params
static const string DEFAULT_MG_EEF = "gripper"; //todo: params
static const double DEFAULT_PLACE_HEIGHT = 0.15;


Katana::Katana() {


    lastHeightAboveTable = 0.0; //todo: move to parent
    //todo: params
    touchlinks.push_back("katana_gripper_tool_frame");
    touchlinks.push_back("katana_gripper_link");
    touchlinks.push_back("katana_gripper_tool_frame");
    touchlinks.push_back("katana_l_finger_link");
    touchlinks.push_back("katana_r_finger_link");
    touchlinks.push_back("katana_base_link");
    touchlinks.push_back("katana_motor4_lift_link");
    touchlinks.push_back("katana_motor5_wrist_roll_link");

    frame = "katana_gripper_tool_frame";

	groupArm = new moveit::planning_interface::MoveGroup(DEFAULT_MG_ARM);
	groupArm->setPlanningTime(120.0);
	groupArm->startStateMonitor();
        
    groupArm->setPoseReferenceFrame(ParamReader::getParamReader().frameArm);

    //todo: params
	groupArm->setGoalJointTolerance(0.01); //rad
	groupArm->setGoalPositionTolerance(0.02);  //m
	groupArm->setGoalOrientationTolerance(0.5); //rad

	groupEe = new moveit::planning_interface::MoveGroup(DEFAULT_MG_EEF);
	groupEe->startStateMonitor();

	sensor_subscriber = nh.subscribe("sensor_states", 1, &Katana::sensorCallback, this);

	pickActionClient.reset(
			new actionlib::SimpleActionClient<moveit_msgs::PickupAction>(nh,
					move_group::PICKUP_ACTION, false));
	placeActionClient.reset(
			new actionlib::SimpleActionClient<moveit_msgs::PlaceAction>(nh,
					move_group::PLACE_ACTION, false));
	movementActionClient.reset(
	            new actionlib::SimpleActionClient<katana_msgs::JointMovementAction>(nh,
	                    "katana_arm_controller/joint_movement_action", false));

	waitForAction(pickActionClient, ros::Duration(0, 0), move_group::PICKUP_ACTION);
	waitForAction(placeActionClient, ros::Duration(0, 0), move_group::PLACE_ACTION);
	waitForAction(movementActionClient, ros::Duration(0, 0), "katana_arm_controller/joint_movement_action");

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
}

void Katana::closeEef(bool withSensors) {
	ROS_INFO("### Invoked closeGripper ###");
	moveToGripper(ParamReader::getParamReader().eefPosClosed.at(0), withSensors);
}

void Katana::moveToGripper(double target, bool withSensors) {
	ROS_DEBUG("### Invoked moveToGripper ###");
	string actionName = "gripper_grasp_posture_controller";
	if (withSensors) {
		actionName = "gripper_grasp_posture_with_sensors_controller";
	}

	// TODO when we have a move sophisticated controller manager, this can be done with move_group.

	actionlib::SimpleActionClient<control_msgs::GripperCommandAction> client(actionName, true);

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
	nh.serviceClient<std_srvs::Empty>("switch_motors_on").call(empty.request, empty.response);
}

void Katana::motorsOff() {
	ROS_INFO("Invoked motorsOff");
	std_srvs::Empty empty;
	nh.serviceClient<std_srvs::Empty>("switch_motors_off").call(empty.request, empty.response);
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
        if (movementActionClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
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

	bool gripperClosed = fabs(fingerJointAngles[0] - ParamReader::getParamReader().eefPosClosed) < 0.05;
	bool gripperNearClosed = fabs(fingerJointAngles[0] - ParamReader::getParamReader().eefPosClosed) < 0.15;

//	return (force && !gripperClosed) || (distance && !gripperNearClosed);
	return (force && !gripperClosed);
}

map<string, short> Katana::getGripperSensors() const {
	ROS_DEBUG("Invoked getGripperSensors");
	boost::mutex::scoped_lock lock(sensorMutex);
	return currentSensorReadings;
}

GraspReturnType Katana::graspObject(ObjectShape obj, bool simulate, const string &startPose) {

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
    tfTransformer.transform(obj, objArmFrame, ParamReader::getParamReader().frameArm);
    double tableHeightArmFrame = objArmFrame.center.xMeter - objArmFrame.heightMeter / 2.0;

    return Model::graspObject(objId, "", grasps, tableHeightArmFrame, simulate, startPose);
}

GraspReturnType Katana::graspObject(const string &obj, const string &surface, bool simulate, const string &startPose) {

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
    tfTransformer.transform(collisionObject, collisionObjectArmCoords, ParamReader::getParamReader().frameArm);
    double tableHeightArmCoords = collisionObjectArmCoords.primitive_poses[0].position.x - collisionObjectArmCoords.primitives[0].dimensions[0] / 2.0;

	vector<moveit_msgs::Grasp> grasps = generate_grasps_angle_trans(collisionObject);
	rosTools.publish_grasps_as_markerarray(grasps);

	return Model::graspObject(obj, surface, grasps, tableHeightArmCoords, simulate, startPose);
}


GraspReturnType Katana::placeObject(EefPose obj, bool simulate,
		const string &startPose) {
	ROS_INFO("### Invoked placeObject ###");

	vector<moveit_msgs::PlaceLocation> locations = generate_place_locations(obj);
	rosTools.publish_place_locations_as_markerarray(locations);

	return Model::placeObject("", locations, simulate, startPose);
}

GraspReturnType Katana::placeObject(ObjectShape obj, bool simulate,
		const string &startPose) {
	ROS_INFO("### Invoked placeObject (bb) ###");

	vector<moveit_msgs::PlaceLocation> locations = generate_place_locations(obj);
	rosTools.publish_place_locations_as_markerarray(locations);

	return Model::placeObject("", locations, simulate, startPose);
}

GraspReturnType Katana::placeObject(const string &obj, bool simulate, const string &startPose) {
    ROS_INFO("### Invoked placeObject (str) ###");
    ROS_ERROR("placing with surface string not suppported yet!");
    GraspReturnType grt;
    grt.result = GraspReturnType::FAIL;
    return grt;
}

std::vector<moveit_msgs::PlaceLocation> Katana::generate_place_locations(
		EefPose obj) {

	tfTransformer.transform(obj, obj, ParamReader::getParamReader().frameArm);

	ROS_INFO_STREAM(
			"generate_place_locations(): lastGraspPose:" << lastGraspPose << " - lastHeightAboveTable: " << lastHeightAboveTable);
	geometry_msgs::Quaternion orientMsg = lastGraspPose.pose.orientation;
	tf::Quaternion orientation = tf::Quaternion(orientMsg.x,orientMsg.y,orientMsg.z,orientMsg.w);
	if (orientation.w() == 0.0f && orientation.x() == 0.0f && orientation.y() == 0.0f
			&& orientation.z() == 0.0f) {
		orientation = tf::createQuaternionFromRPY(0, -M_PI_2, 0);
	}

	Vec t = obj.translation;
//	if (lastHeightAboveTable == 0.0) {
//		t.xMeter += DEFAULT_PLACE_HEIGHT;
//	} else {
//		t.xMeter += lastHeightAboveTable;
//	}

	return graspGenerator.generate_placeloc_angle_trans(t.xMeter, t.yMeter, t.zMeter);

}

std::vector<moveit_msgs::PlaceLocation> Katana::generate_place_locations(
		ObjectShape obj) {

	tfTransformer.transform(obj, obj, ParamReader::getParamReader().frameArm);

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

std::vector<moveit_msgs::Grasp> Katana::generate_grasps_angle_trans(ObjectShape shape) {
    tfTransformer.transform(shape, shape, ParamReader::getParamReader().frameArm);
    return graspGenerator.generate_grasps_angle_trans(shape.center.xMeter, shape.center.yMeter, shape.center.zMeter, shape.heightMeter);
}

std::vector<moveit_msgs::Grasp> Katana::generate_grasps_angle_trans(moveit_msgs::CollisionObject shape) {
    tfTransformer.transform(shape, shape, ParamReader::getParamReader().frameArm);
    return graspGenerator.generate_grasps_angle_trans(shape.primitive_poses[0].position.x, shape.primitive_poses[0].position.y, shape.primitive_poses[0].position.z, shape.primitives[0].dimensions[0]);
}


void Katana::sensorCallback(const sensor_msgs::JointStatePtr& sensorReadings) {
	boost::mutex::scoped_lock lock(sensorMutex);
	for (int i = 0; i < sensorReadings->name.size(); i++) {
		string sensor = sensorReadings->name[i];
		double reading = sensorReadings->position[i];
		currentSensorReadings[sensor] = reading;
	}
}

katana_msgs::JointMovementGoal Katana::buildMovementGoal(const std::string &poseName) {
    ArmPose pose = getRememberedPose(poseName);
    katana_msgs::JointMovementGoal goal;
    for (ArmPose::iterator i = pose.begin(); i != pose.end(); ++i) {
        goal.jointGoal.name.push_back(i->first);
        goal.jointGoal.position.push_back(i->second);
    }
    return goal;
}
