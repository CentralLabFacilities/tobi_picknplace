/*
 * KatanaSimModel.cpp
 *
 *  Created on: Apr 29, 2015
 *      Author: lziegler
 */

#include "KatanaSimModel.h"

#include <ros/ros.h>

using namespace std;

KatanaSimModel::KatanaSimModel() {
}

KatanaSimModel::~KatanaSimModel() {
}

void KatanaSimModel::addListener(ModelListener* listener) {
}

void KatanaSimModel::removeListener(ModelListener* listener) {
}

map<string, double> KatanaSimModel::getJointAngles() const {
	ROS_INFO("Invoked getJointAngles");
	return map<string, double>();
}
vector<string> KatanaSimModel::getJointNames() const {
    ROS_INFO("Invoked getJointNames");
    return vector<string>();
}

void KatanaSimModel::setJointAngle(const string &joint, double angle) {
	ROS_INFO("Invoked setJointAngle");
}

void KatanaSimModel::setJointAngles(const map<string, double> &angles) {
	ROS_INFO("Invoked setJointAngles");
}
void KatanaSimModel::setJointAngles(const vector<double> &angles) {
    ROS_INFO("Invoked setJointAngles");
}

int KatanaSimModel::getNumJoints() const {
	ROS_INFO("Invoked getNumJoints");
	return -1;
}

void KatanaSimModel::openGripper(bool withSensors) {
	ROS_INFO("Invoked openGripper");
}

void KatanaSimModel::closeGripper(bool withSensors) {
	ROS_INFO("Invoked closeGripper");
}

void KatanaSimModel::moveToGripper(double target, bool withSensors) {
	ROS_INFO("Invoked moveToGripper");
}

void KatanaSimModel::motorsOn() {
	ROS_INFO("Invoked motorsOn");
}

void KatanaSimModel::motorsOff() {
	ROS_INFO("Invoked motorsOff");
}

EefPose KatanaSimModel::getEefPose() const {
	ROS_INFO("Invoked getEefPose");

	EefPose pose;
	return pose;
}

MoveResult KatanaSimModel::moveTo(const EefPose& pose, bool linear, bool orientation) {
	ROS_INFO("Invoked moveTo (pose)");
	return OTHER;
}

ArmPoses KatanaSimModel::getRememberedPoses() const {
	return ArmPoses();
}

ArmPose KatanaSimModel::getRememberedPose(const std::string &name) const {
    return ArmPose();
}

MoveResult KatanaSimModel::moveTo(const std::string& poseName, bool plan) {
	ROS_INFO("Invoked moveTo (string)");
	return OTHER;
}

void KatanaSimModel::stop() const {

}


bool KatanaSimModel::isSomethingInGripper() const {
	ROS_INFO("Invoked isSomethingInGripper");
	return false;
}

map<string, short> KatanaSimModel::getGripperSensors() const {
	ROS_INFO("Invoked getGripperSensors");
	map<string, short> sensors;
	return sensors;
}

GraspReturnType KatanaSimModel::graspObject(ObjectShape obj, bool simulate, const std::string &startPose) {
	ROS_INFO("Invoked graspObject");
	GraspReturnType grt;
	return grt;
}
GraspReturnType KatanaSimModel::graspObject(const string &obj, const string &surface, bool simulate, const std::string &startPose) {
	ROS_INFO("Invoked graspObject");
	GraspReturnType grt;
	return grt;
}

GraspReturnType KatanaSimModel::placeObject(ObjectShape obj, bool simulate, const std::string &startPose) {
	ROS_INFO("Invoked placeObject");
	GraspReturnType grt;
	return grt;
}

GraspReturnType KatanaSimModel::placeObject(EefPose obj, bool simulate, const std::string &startPose) {
	ROS_INFO("Invoked placeObject");
	GraspReturnType grt;
	return grt;
}
GraspReturnType KatanaSimModel::placeObject(const string &surface, bool simulate, const std::string &startPose) {
	ROS_INFO("Invoked placeObject");
	GraspReturnType grt;
	return grt;
}
