/*
 * Controller.cpp
 *
 *  Created on: Apr 29, 2015
 *      Author: lziegler
 */

#include "Controller.h"

using namespace std;

Controller::Controller(const Model::Ptr& model, const ControlStrategy::Ptr &strategy) :
		model(model), strategy(strategy) {
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

bool Controller::requestPlanTo(const std::string &poseName) {
	//return katana->moveTo(poseName);
	return strategy->planTo(poseName);
	//return strategy->moveTo(poseName);
}

std::string Controller::requestNearestPose() const{
	return strategy->findNearestPose();
}

ArmPoses Controller::requestPoses() const {
	return model->getRememberedPoses();
}

int Controller::requestFindObjects() const {
    return model->findObjects();
}

std::string Controller::requestGetSurfaceByHeight(const float h) {
	return model->getSurfaceByHeight(h);
}

/**GraspReturnType Controller::requestGraspObject(ObjectShape obj, bool simulate) {
	return strategy->graspObject(obj, simulate);
}**/

GraspReturnType Controller::requestGraspObject(const string &obj, const string &surface, bool simulate) {
    return strategy->graspObject(obj, surface, simulate);
}

GraspReturnType Controller::requestPlaceObject(ObjectShape obj, bool simulate) {
	return strategy->placeObject(obj, simulate);
}
GraspReturnType Controller::requestPlaceObject(EefPose obj, bool simulate) {
	return strategy->placeObject(obj, simulate);
}
GraspReturnType Controller::requestPlaceObject(const string &surface, bool simulate) {
    return strategy->placeObject(surface, simulate);
}

void Controller::requestMoveJointAngles(const vector<double>& angle) {
	model->setJointAngles(angle);
}

int Controller::requestNumJoints() const {
	return model->getNumJoints();
}

map<string, double> Controller::requestJointAngles() const {
	return model->getJointAngles();
}

void Controller::requestOpenGripper(bool withSensors) {
	model->openEef(withSensors);
}

void Controller::requestCloseGripper(bool withSensors) {
	model->closeEef(withSensors);
}

void Controller::requestMotorsOn() {
	model->motorsOn();
}

void Controller::requestMotorsOff() {
	model->motorsOff();
}

EefPose Controller::requestEefPose() const {
	return model->getEefPose();
}

bool Controller::requestMoveTo(const EefPose& pose, bool linear, bool orientation) {
	return model->moveTo(pose, linear, orientation);
}

SensorReadings Controller::requestGripperSensors() const {
	return model->getGripperSensors();
}

bool Controller::requestIsSomethingInGripper() const {
	return model->isSomethingInGripper();
}
