/*
 * KatanaSimModel.cpp
 *
 *  Created on: Apr 29, 2015
 *      Author: lziegler
 */

#include "KatanaSim.h"
#include <ros/ros.h>

using namespace std;

KatanaSim::KatanaSim() {
}

KatanaSim::~KatanaSim() {
}

map<string, double> KatanaSim::getJointAngles() const {
	ROS_INFO("Invoked getJointAngles");
	return map<string, double>();
}
vector<string> KatanaSim::getJointNames() const {
    ROS_INFO("Invoked getJointNames");
    return vector<string>();
}

void KatanaSim::setJointAngle(const string &joint, double angle) {
	ROS_INFO("Invoked setJointAngle");
}

void KatanaSim::setJointAngles(const map<string, double> &angles) {
	ROS_INFO("Invoked setJointAngles");
}
void KatanaSim::setJointAngles(const vector<double> &angles) {
    ROS_INFO("Invoked setJointAngles");
}

int KatanaSim::getNumJoints() const {
	ROS_INFO("Invoked getNumJoints");
	return -1;
}

void KatanaSim::openEef(bool withSensors) {
	ROS_INFO("Invoked openGripper");
}

void KatanaSim::closeEef(bool withSensors) {
	ROS_INFO("Invoked closeGripper");
}

void KatanaSim::moveToGripper(double target, bool withSensors) {
	ROS_INFO("Invoked moveToGripper");
}

void KatanaSim::motorsOn() {
	ROS_INFO("Invoked motorsOn");
}

void KatanaSim::motorsOff() {
	ROS_INFO("Invoked motorsOff");
}

EefPose KatanaSim::getEefPose() const {
	ROS_INFO("Invoked getEefPose");

	EefPose pose;
	return pose;
}

MoveResult KatanaSim::moveTo(const EefPose& pose, bool linear, bool orientation) {
	ROS_INFO("Invoked moveTo (pose)");
	return OTHER;
}

ArmPoses KatanaSim::getRememberedPoses() const {
	return ArmPoses();
}

ArmPose KatanaSim::getRememberedPose(const std::string &name) const {
    return ArmPose();
}

MoveResult KatanaSim::moveTo(const std::string& poseName, bool plan) {
	ROS_INFO("Invoked moveTo (string)");
	return OTHER;
}

void KatanaSim::stop() const {

}

bool KatanaSim::isSomethingInGripper() const {
	ROS_INFO("Invoked isSomethingInGripper");
	return false;
}

map<string, short> KatanaSim::getGripperSensors() const {
	ROS_INFO("Invoked getGripperSensors");
	map<string, short> sensors;
	return sensors;
}

/**GraspReturnType KatanaSim::graspObject(ObjectShape obj, bool simulate, const std::string &startPose) {
	ROS_INFO("Invoked graspObject");
	GraspReturnType grt;
	return grt;
}**/
GraspReturnType KatanaSim::graspObject(const string &obj, const string &surface, bool simulate, const std::string &startPose) {
	ROS_INFO("Invoked graspObject");
	GraspReturnType grt;
	return grt;
}

/**GraspReturnType KatanaSim::placeObject(ObjectShape obj, bool simulate, const std::string &startPose) {
	ROS_INFO("Invoked placeObject");
	GraspReturnType grt;
	return grt;
}**/

GraspReturnType KatanaSim::placeObject(EefPose obj, bool simulate, const std::string &startPose) {
	ROS_INFO("Invoked placeObject");
	GraspReturnType grt;
	return grt;
}
GraspReturnType KatanaSim::placeObject(const string &surface, bool simulate, const std::string &startPose) {
	ROS_INFO("Invoked placeObject");
	GraspReturnType grt;
	return grt;
}
