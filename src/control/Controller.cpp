/*
 * Controller.cpp
 *
 *  Created on: Apr 29, 2015
 *      Author: lziegler
 */

#include "Controller.h"

using namespace std;

rsc::logging::LoggerPtr Controller::logger = rsc::logging::Logger::getLogger(
		"picknplace.Controller");

Controller::Controller(const Model::Ptr& model, const ControlStrategy::Ptr &strategy) :
		katana(model), strategy(strategy) {
}

Controller::~Controller() {
}

void Controller::addControlInterface(const ControlInterface::Ptr& interface) {
	controlInterfaces.push_back(interface);
	interface->setListener(this);
}

void Controller::removeControlInterface(const ControlInterface::Ptr& interface) {

	vector<ControlInterface::Ptr>::iterator i;
	for (i = controlInterfaces.begin(); i != controlInterfaces.end(); ++i) {
		if (interface == *i) {
			controlInterfaces.erase(i);
			interface->removeListener();
			return;
		}
	}
}

bool Controller::requestMoveTo(const std::string &poseName) {
	return strategy->moveTo(poseName);
}

std::string Controller::requestNearestPose() const{
	return strategy->findNearestPose();
}

ArmPoses Controller::requestPoses() const {
	return katana->getRememberedPoses();
}

GraspReturnType Controller::requestGraspObject(ObjectShape obj, bool simulate) {
	return strategy->graspObject(obj, simulate);
}

GraspReturnType Controller::requestPlaceObject(ObjectShape obj, bool simulate) {
	return strategy->placeObject(obj, simulate);
}
GraspReturnType Controller::requestPlaceObject(EefPose obj, bool simulate) {
	return strategy->placeObject(obj, simulate);
}

void Controller::requestMoveJointAngles(const vector<double>& angle) {
	katana->setJointAngles(angle);
}

int Controller::requestNumJoints() const {
	return katana->getNumJoints();
}

map<string, double> Controller::requestJointAngles() const {
	return katana->getJointAngles();
}

void Controller::requestOpenGripper(bool withSensors) {
	katana->openGripper(withSensors);
}

void Controller::requestCloseGripper(bool withSensors) {
	katana->closeGripper(withSensors);
}

void Controller::requestMotorsOn() {
	katana->motorsOn();
}

void Controller::requestMotorsOff() {
	katana->motorsOff();
}

EefPose Controller::requestEefPose() const {
	return katana->getEefPose();
}

bool Controller::requestMoveTo(const EefPose& pose, bool linear, bool orientation) {
	return katana->moveTo(pose, linear, orientation);
}

SensorReadings Controller::requestGripperSensors() const {
	return katana->getGripperSensors();
}

bool Controller::requestIsSomethingInGripper() const {
	return katana->isSomethingInGripper();
}
