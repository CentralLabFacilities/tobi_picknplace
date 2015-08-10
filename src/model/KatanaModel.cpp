/*
 * KatanaModel.cpp
 *
 *  Created on: Apr 29, 2015
 *      Author: lziegler
 */

#include "KatanaModel.h"

#include "../util/TransformerTF.h"
#include "../util/ParamReader.h"

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

KatanaModel::KatanaModel(): lastHeightAboveTable(0.0) {

	groupArm = new moveit::planning_interface::MoveGroup("arm");
	groupArm->setPlanningTime(120.0);
	groupArm->startStateMonitor();
        
    groupArm->setPoseReferenceFrame(ParamReader::getParamReader().frameOriginArm);

	groupArm->setGoalJointTolerance(0.01); //rad
	groupArm->setGoalPositionTolerance(0.02);  //m
	groupArm->setGoalOrientationTolerance(0.5); //rad

	groupGripper = new moveit::planning_interface::MoveGroup("gripper");
	groupGripper->startStateMonitor();

	sensor_subscriber = nh.subscribe("sensor_states", 1, &KatanaModel::sensorCallback, this);
	pickActionClient.reset(
			new actionlib::SimpleActionClient<moveit_msgs::PickupAction>(nh,
					move_group::PICKUP_ACTION, false));
	placeActionClient.reset(
			new actionlib::SimpleActionClient<moveit_msgs::PlaceAction>(nh,
					move_group::PLACE_ACTION, false));

	//scene = moveit::planning_interface::getSharedStateMonitor(moveit::planning_interface::getSharedRobotModel("tobi"), moveit::planning_interface::getSharedTF());
	//scene->startStateMonitor();

	waitForAction(pickActionClient, ros::Duration(0, 0), move_group::PICKUP_ACTION);
	waitForAction(placeActionClient, ros::Duration(0, 0), move_group::PLACE_ACTION);

	ROS_INFO("KatanaModel: connected");
}

KatanaModel::~KatanaModel() {
	delete groupArm;
}

void KatanaModel::addListener(ModelListener* listener) {
	listeners.push_back(listener);
}

void KatanaModel::removeListener(ModelListener* listener) {
	vector<ModelListener*>::iterator i;
	for (i = listeners.begin(); i != listeners.end(); ++i) {
		if (listener == *i) {
			listeners.erase(i);
			return;
		}
	}
}

vector<double> KatanaModel::getJointAngles() const {
	ROS_DEBUG("Invoked getJointAngles");
	return groupArm->getCurrentJointValues();
}

void KatanaModel::setJointAngle(int joint, double angle) {
	ROS_INFO("### Invoked setJointAngle ###");
	vector<string> joints = groupArm->getJoints();
	if (joint < 0 || joint >= joints.size()) {
		ROS_ERROR("Requested joint %i does not exist!", joint);
		return;
	}

	if (!isSomethingInGripper()) {
		rosTools.detach_collision_object();
	}

	groupArm->clearPoseTargets();
	groupArm->setStartStateToCurrentState();
	groupArm->setJointValueTarget(joints[joint], angle);
	groupArm->move();
}

void KatanaModel::setJointAngles(const vector<double> &angles) {
	ROS_INFO("### Invoked setJointAngles ###");
	vector<string> joints = groupArm->getJoints();
	if (angles.size() < 0 || angles.size() > joints.size()) {
		ROS_ERROR("Requested number of joints wrong! (%i)", (int )angles.size());
		return;
	}

	if (!isSomethingInGripper()) {
		rosTools.detach_collision_object();
	}

	groupArm->clearPoseTargets();
	groupArm->setStartStateToCurrentState();
	for (int i = 0; i < angles.size(); i++) {
		groupArm->setJointValueTarget(joints[i], angles[i]);
	}
	groupArm->move();
}

int KatanaModel::getNumJoints() const {
	ROS_DEBUG("Invoked getNumJoints");
	return groupArm->getJoints().size();
}

void KatanaModel::openGripper(bool withSensors) {
	ROS_INFO("### Invoked openGripper ###");
	moveToGripper(ParamReader::getParamReader().gripperPositionOpen, withSensors);
}

void KatanaModel::closeGripper(bool withSensors) {
	ROS_INFO("### Invoked closeGripper ###");
	moveToGripper(ParamReader::getParamReader().gripperPositionClosed, withSensors);
}

void KatanaModel::moveToGripper(double target, bool withSensors) {
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

void KatanaModel::motorsOn() {
	ROS_INFO("Invoked motorsOn");
	std_srvs::Empty empty;
	nh.serviceClient<std_srvs::Empty>("switch_motors_on").call(empty.request, empty.response);
}

void KatanaModel::motorsOff() {
	ROS_INFO("Invoked motorsOff");
	std_srvs::Empty empty;
	nh.serviceClient<std_srvs::Empty>("switch_motors_off").call(empty.request, empty.response);
}

EefPose KatanaModel::getEefPose() const {
	ROS_DEBUG("Invoked getEefPose");
	EefPose pose;
	geometry_msgs::PoseStamped ps = groupArm->getCurrentPose();

	ROS_INFO_STREAM("getEefPose() 1: " << ps.pose.position.x << "," << ps.pose.position.y << "," << ps.pose.position.z << "," << ps.header.frame_id);

	tfTransformer.transform(ps, ps, ParamReader::getParamReader().frameOriginArm);

	ROS_INFO_STREAM("getEefPose() 2: " << ps.pose.position.x << "," << ps.pose.position.y << "," << ps.pose.position.z << "," << ps.header.frame_id);

	pose.translation.xMeter = ps.pose.position.x;
	pose.translation.yMeter = ps.pose.position.y;
	pose.translation.zMeter = ps.pose.position.z;
	pose.rotation.qw = ps.pose.orientation.w;
	pose.rotation.qx = ps.pose.orientation.x;
	pose.rotation.qy = ps.pose.orientation.y;
	pose.rotation.qz = ps.pose.orientation.z;
	pose.frame = ParamReader::getParamReader().frameOriginArm;
	return pose;
}

MoveResult KatanaModel::moveTo(const EefPose& pose, bool linear, bool orientation) {
	ROS_INFO("### Invoked moveTo (pose) ###");

	if (!isSomethingInGripper()) {
		rosTools.detach_collision_object();
	}

	groupArm->clearPoseTargets();
	groupArm->setStartStateToCurrentState();

	if (orientation) {
		geometry_msgs::Pose p;
		p.position.x = pose.translation.xMeter;
		p.position.y = pose.translation.yMeter;
		p.position.z = pose.translation.zMeter;
		p.orientation.w = pose.rotation.qw;
		p.orientation.x = pose.rotation.qx;
		p.orientation.y = pose.rotation.qy;
		p.orientation.z = pose.rotation.qz;
		groupArm->setPoseTarget(p);
	} else {
		groupArm->setPositionTarget(pose.translation.xMeter, pose.translation.yMeter,
				pose.translation.zMeter);
	}

	return rosTools.moveResultFromMoveit(groupArm->move());
}

MoveResult KatanaModel::moveTo(const std::string& poseName) {
	ROS_INFO("### Invoked moveTo (string) ###");

	if (!isSomethingInGripper()) {
		rosTools.detach_collision_object();
	}

	groupArm->clearPoseTargets();
	groupArm->setStartStateToCurrentState();
	groupArm->setNamedTarget(poseName);

	return rosTools.moveResultFromMoveit(groupArm->move());
}


map<string, vector<double> > KatanaModel::getRememberedPoses() const {
	ROS_DEBUG("Invoked getRememberedPoses");
	string planningGroup = groupArm->getName();
	const robot_model::JointModelGroup* jmg =
			groupArm->getCurrentState()->getRobotModel()->getJointModelGroup(planningGroup);
	vector<string> names = jmg->getDefaultStateNames();
	Poses poses;
	for (vector<string>::iterator it = names.begin(); it != names.end(); it++) {
		string name = *it;
		vector<double> angleVals;
		map<string, double> angles;
		jmg->getVariableDefaultPositions(name, angles);
		map<string, double>::iterator ait;
		for (ait = angles.begin(); ait != angles.end(); ++ait) {
			angleVals.push_back(ait->second);
		}
		poses[name] = angleVals;
	}
	ROS_DEBUG("poses %d, names %d", (int )poses.size(), (int )names.size());
	return poses;
}

void KatanaModel::stop() const {
	groupArm->stop();
}

bool KatanaModel::isSomethingInGripper() const {
	ROS_DEBUG("### Invoked isSomethingInGripper ###");
	vector<double> fingerJointAngles = groupGripper->getCurrentJointValues();
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

	bool gripperClosed = fabs(fingerJointAngles[0] - ParamReader::getParamReader().gripperPositionClosed) < 0.05;
	bool gripperNearClosed = fabs(fingerJointAngles[0] - ParamReader::getParamReader().gripperPositionClosed) < 0.15;

//	return (force && !gripperClosed) || (distance && !gripperNearClosed);
	return (force && !gripperClosed);
}

map<string, short> KatanaModel::getGripperSensors() const {
	ROS_DEBUG("Invoked getGripperSensors");
	boost::mutex::scoped_lock lock(sensorMutex);
	return currentSensorReadings;
}

std::vector<moveit_msgs::Grasp> KatanaModel::generate_grasps_angle_trans(ObjectShape shape) {
	tfTransformer.transform(shape, shape, ParamReader::getParamReader().frameOriginArm);
	return graspGenerator.generate_grasps_angle_trans(shape.center.xMeter, shape.center.yMeter, shape.center.zMeter, shape.heightMeter);
}


GraspReturnType KatanaModel::graspObject(ObjectShape obj, bool simulate, const string &startPose) {

	ROS_INFO("### Invoked graspObject ###");

	if (obj.widthMeter > 0.1) {
		obj.widthMeter = 0.1;
	}
	if (obj.depthMeter > 0.1) {
		obj.depthMeter = 0.1;
	}

	GraspReturnType grt;

	if (!pickActionClient) {
		ROS_ERROR_STREAM("Pick action client not found");
		grt.result = GraspReturnType::FAIL;
		return grt;
	}
	if (!pickActionClient->isServerConnected()) {
		ROS_ERROR_STREAM("Pick action server not connected");
		grt.result = GraspReturnType::FAIL;
		return grt;
	}

	ROS_INFO("Trying to pick object at %.3f, %.3f, %.3f (frame: %s).", obj.center.xMeter, obj.center.yMeter, obj.center.zMeter, obj.center.frame.c_str());

	// publish collision object
	rosTools.publish_collision_object(obj, 0.5);

	vector<moveit_msgs::Grasp> grasps = generate_grasps_angle_trans(obj);
	rosTools.publish_grasps_as_markerarray(grasps);
	
	moveit_msgs::PickupGoal goal;
	goal.possible_grasps = grasps;
	goal.attached_object_touch_links.push_back("katana_gripper_tool_frame");
	goal.attached_object_touch_links.push_back("katana_gripper_link");
	goal.attached_object_touch_links.push_back("katana_gripper_tool_frame");
	goal.attached_object_touch_links.push_back("katana_l_finger_link");
	goal.attached_object_touch_links.push_back("katana_r_finger_link");
	goal.attached_object_touch_links.push_back("katana_base_link");
	goal.attached_object_touch_links.push_back("katana_motor4_lift_link");
	goal.attached_object_touch_links.push_back("katana_motor5_wrist_roll_link");
	goal.target_name = rosTools.getDefaultObjectName();
	goal.group_name = groupArm->getName();
	goal.end_effector = groupArm->getEndEffector();
	goal.allowed_planning_time = groupArm->getPlanningTime();
	goal.support_surface_name = "";
	goal.planner_id = "";
	goal.planning_options.plan_only = simulate;
	goal.planning_options.look_around = false;
	goal.planning_options.replan = false;
	goal.planning_options.replan_delay = 2.0;
	goal.planning_options.planning_scene_diff.is_diff = true;
	goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

	pickActionClient->sendGoal(goal);
	if (!pickActionClient->waitForResult()) {
		ROS_INFO_STREAM("Pickup action returned early");
	}
	ROS_INFO("###########################");
	if (pickActionClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
		ROS_INFO("  Pick Action succeeded. (err_code: %d, state: %s)", pickActionClient->getResult()->error_code.val, pickActionClient->getState().toString().c_str());

		if (simulate || isSomethingInGripper()) {
			moveit_msgs::Grasp resultGrasp = pickActionClient->getResult()->grasp;
			grt.point.xMeter = resultGrasp.grasp_pose.pose.position.x;
			grt.point.yMeter = resultGrasp.grasp_pose.pose.position.y;
			grt.point.zMeter = resultGrasp.grasp_pose.pose.position.z;
			grt.point.frame = resultGrasp.grasp_pose.header.frame_id;
			lastGraspPose = resultGrasp.grasp_pose;

			ObjectShape objArmFrame;
			tfTransformer.transform(obj, objArmFrame, ParamReader::getParamReader().frameOriginArm);
			lastHeightAboveTable = resultGrasp.grasp_pose.pose.position.x - objArmFrame.center.xMeter + objArmFrame.heightMeter / 2.0;
			grt.result = GraspReturnType::SUCCESS;
			ROS_INFO("  Grasped object at %.3f, %.3f, %.3f (frame: %s).", grt.point.xMeter, grt.point.yMeter, grt.point.zMeter, grt.point.frame.c_str());
		} else {
			ROS_WARN_STREAM("  Grasped no object: nothing in gripper!");
			grt.result = GraspReturnType::FAIL;
		}
	} else if (pickActionClient->getState() == actionlib::SimpleClientGoalState::ABORTED
			|| (pickActionClient->getState() == SimpleClientGoalState::PENDING
					&& placeActionClient->getResult()->error_code.val == MoveItErrorCode::SUCCESS)) {
		ROS_WARN_STREAM(
				"  Pick Action ABORTED (" << pickActionClient->getResult()->error_code.val << "): " << pickActionClient->getState().getText());
		grt.result = GraspReturnType::ROBOT_CRASHED;
		rosTools.clear_octomap();

	} else {
		ROS_WARN_STREAM(
				"  Pick Action failed: " << pickActionClient->getState().toString() << " (" << pickActionClient->getResult()->error_code.val  << "): " << pickActionClient->getState().getText());
		grt.result = rosTools.graspResultFromMoveit(pickActionClient->getResult()->error_code);
	}
	ROS_INFO("###########################");

	rosTools.remove_collision_object();

	if (grt.result != GraspReturnType::SUCCESS) {
		rosTools.detach_collision_object();
	}

	return grt;
}

GraspReturnType KatanaModel::placeObject(EefPose obj, bool simulate,
		const string &startPose) {
	ROS_INFO("### Invoked placeObject ###");

	GraspReturnType grt;

	if (!pickActionClient) {
		ROS_ERROR_STREAM("Pick action client not found");
		grt.result = GraspReturnType::FAIL;
		return grt;
	}
	if (!pickActionClient->isServerConnected()) {
		ROS_ERROR_STREAM("Pick action server not connected");
		grt.result = GraspReturnType::FAIL;
		return grt;
	}

	vector<moveit_msgs::PlaceLocation> locations = generate_place_locations(obj);
	rosTools.publish_place_locations_as_markerarray(locations);

	moveit_msgs::PlaceGoal goal = buildPlaceGoal(locations, simulate);

	for (int i = 0; i < 3; i++) {
		placeActionClient->sendGoal(goal);
		SimpleClientGoalState resultState = placeActionClient->getState();

		if (!placeActionClient->waitForResult()) {
			ROS_INFO_STREAM("Place action returned early");
		}

		ROS_INFO("###########################");
		if (resultState == SimpleClientGoalState::SUCCEEDED) {
			ROS_INFO("Place Action succeeded.");
			grt.point = obj.translation;
			grt.result = GraspReturnType::SUCCESS;
			ROS_INFO("###########################");
			break;
		} else if (resultState == SimpleClientGoalState::PENDING
				&& placeActionClient->getResult()->error_code.val == MoveItErrorCode::INVALID_GROUP_NAME) {
			ROS_WARN("  PENDING: attached object not preset! Try to fix by attaching default object.");
			attachDefaultObject();
			ROS_INFO("###########################");
			continue;
		} else if (resultState == SimpleClientGoalState::ABORTED
				|| (resultState == SimpleClientGoalState::PENDING
						&& (placeActionClient->getResult()->error_code.val
								== MoveItErrorCode::SUCCESS
								|| placeActionClient->getResult()->error_code.val
										== MoveItErrorCode::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE))) {
			ROS_WARN("  ABORTED: %s", resultState.getText().c_str());
			grt.result = GraspReturnType::ROBOT_CRASHED;
			rosTools.clear_octomap();
			break;
		} else {
			ROS_WARN("  Fail: %s (%d): %s", resultState.toString().c_str(), placeActionClient->getResult()->error_code.val, resultState.getText().c_str());
			grt.result = rosTools.graspResultFromMoveit(placeActionClient->getResult()->error_code);
			ROS_INFO("###########################");
			break;
		}
		break;
	}

	if (!isSomethingInGripper()) {
		rosTools.detach_collision_object();
	}
	rosTools.remove_collision_object();

	return grt;
}

GraspReturnType KatanaModel::placeObject(ObjectShape obj, bool simulate,
		const string &startPose) {
	ROS_INFO("### Invoked placeObject (bb) ###");

	GraspReturnType grt;

	if (!pickActionClient) {
		ROS_ERROR_STREAM("Pick action client not found");
		grt.result = GraspReturnType::FAIL;
		return grt;
	}
	if (!pickActionClient->isServerConnected()) {
		ROS_ERROR_STREAM("Pick action server not connected");
		grt.result = GraspReturnType::FAIL;
		return grt;
	}

	vector<moveit_msgs::PlaceLocation> locations = generate_place_locations(obj);
	rosTools.publish_place_locations_as_markerarray(locations);

	moveit_msgs::PlaceGoal goal = buildPlaceGoal(locations, simulate);

	for (int i = 0; i < 3; i++) {
		placeActionClient->sendGoal(goal);
		SimpleClientGoalState resultState = placeActionClient->getState();

		if (!placeActionClient->waitForResult()) {
			ROS_INFO_STREAM("Place action returned early");
		}

		ROS_INFO("###########################");
		if (resultState == SimpleClientGoalState::SUCCEEDED) {
			ROS_INFO("Place Action succeeded.");
			grt.point = obj.center;
			grt.result = GraspReturnType::SUCCESS;
			ROS_INFO("###########################");
			break;
		} else if (resultState == SimpleClientGoalState::PENDING
				&& placeActionClient->getResult()->error_code.val == MoveItErrorCode::INVALID_GROUP_NAME) {
			ROS_WARN("  PENDING: attached object not preset! Try to fix by attaching default object.");
			attachDefaultObject();
			ROS_INFO("###########################");
			continue;
		} else if (resultState == SimpleClientGoalState::ABORTED
				|| (resultState == SimpleClientGoalState::PENDING
						&& (placeActionClient->getResult()->error_code.val
								== MoveItErrorCode::SUCCESS
								|| placeActionClient->getResult()->error_code.val
										== MoveItErrorCode::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE))) {
			ROS_WARN("  ABORTED: %s", resultState.getText().c_str());
			grt.result = GraspReturnType::ROBOT_CRASHED;
			rosTools.clear_octomap();
			break;
		} else {
			ROS_WARN("  Fail: %s (%d): %s", resultState.toString().c_str(), placeActionClient->getResult()->error_code.val, resultState.getText().c_str());
			grt.result = rosTools.graspResultFromMoveit(placeActionClient->getResult()->error_code);
			ROS_INFO("###########################");
			break;
		}
		break;
	}

	if (!isSomethingInGripper()) {
		rosTools.detach_collision_object();
	}
	rosTools.remove_collision_object();

	return grt;
}

void KatanaModel::attachDefaultObject() {
	ROS_INFO("Publishing default object!");
	ObjectShape shape;
	shape.heightMeter = 0.05;
	shape.widthMeter = 0.05;
	shape.depthMeter = 0.05;
	shape.center.frame = "katana_gripper_tool_frame";
	rosTools.publish_collision_object(shape, 0.5);

	vector<string> touchLinks;
	touchLinks.push_back("katana_gripper_tool_frame");
	touchLinks.push_back("katana_gripper_link");
	touchLinks.push_back("katana_gripper_tool_frame");
	touchLinks.push_back("katana_l_finger_link");
	touchLinks.push_back("katana_r_finger_link");
	touchLinks.push_back("katana_base_link");
	touchLinks.push_back("katana_motor4_lift_link");
	touchLinks.push_back("katana_motor5_wrist_roll_link");

	groupGripper->attachObject(rosTools.getDefaultObjectName(), "katana_gripper_tool_frame", touchLinks);

	ros::spinOnce();
	ros::WallDuration sleep_time(1);
	sleep_time.sleep();
}


std::vector<moveit_msgs::PlaceLocation> KatanaModel::generate_place_locations(
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
//	if (lastHeightAboveTable == 0.0) {
//		t.xMeter += DEFAULT_PLACE_HEIGHT;
//	} else {
//		t.xMeter += lastHeightAboveTable;
//	}

	return graspGenerator.generate_placeloc_angle_trans(t.xMeter, t.yMeter, t.zMeter);

}

std::vector<moveit_msgs::PlaceLocation> KatanaModel::generate_place_locations(
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


void KatanaModel::sensorCallback(const sensor_msgs::JointStatePtr& sensorReadings) {
	boost::mutex::scoped_lock lock(sensorMutex);
	for (int i = 0; i < sensorReadings->name.size(); i++) {
		string sensor = sensorReadings->name[i];
		double reading = sensorReadings->position[i];
		currentSensorReadings[sensor] = reading;
	}
}

moveit_msgs::PlaceGoal KatanaModel::buildPlaceGoal(
		const vector<moveit_msgs::PlaceLocation>& locations, bool simulate) {
	moveit_msgs::PlaceGoal goal;
	goal.attached_object_name = rosTools.getDefaultObjectName();
	goal.allowed_touch_objects.push_back(rosTools.getDefaultObjectName());
	goal.group_name = groupArm->getName();
	goal.allowed_planning_time = groupArm->getPlanningTime();
	goal.support_surface_name = "";
	goal.planner_id = "";
	goal.place_eef = true;
	goal.place_locations = locations;
	goal.planning_options.plan_only = simulate;
	goal.planning_options.look_around = false;
	goal.planning_options.replan = false;
	goal.planning_options.replan_delay = 2.0;
	goal.planning_options.planning_scene_diff.is_diff = true;
	goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
	return goal;
}
