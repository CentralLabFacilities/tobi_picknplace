/*
 * Controller.h
 *
 *  Created on: Apr 29, 2015
 *      Author: lziegler
 */

#pragma once

#include <vector>

#include "../interface/ControlInterface.h"
#include "../interface/ControlInterfaceListener.h"
#include "../util/Dijkstra.h"
#include "../util/Transition.h"
#include "ControlStrategy.h"


class Controller: public ControlInterfaceListener {
public:
	Controller(const Model::Ptr &model, const ControlStrategy::Ptr &strategy);
	virtual ~Controller();

	void addControlInterface(const ControlInterface::Ptr &interface);
	void removeControlInterface(const ControlInterface::Ptr &interface);
        
        void requestsetFilterType(std::string type);

	void requestMoveJointAngles(const std::vector<double> &angle);
	int requestNumJoints() const;
	std::map<std::string, double> requestJointAngles() const;

	void requestOpenGripper(bool withSensors);
	void requestCloseGripper(bool withSensors);

	void requestMotorsOn();
	void requestMotorsOff();

	EefPose requestEefPose() const;
	bool requestMoveTo(const EefPose &pose, bool linear, bool orientation);
	bool requestMoveTo(const std::string &poseName);
	bool requestPlanTo(const std::string &poseName);

	SensorReadings requestGripperSensors() const;
	bool requestIsSomethingInGripper() const;

	int requestFindObjects() const;

	//GraspReturnType requestGraspObject(ObjectShape obj, bool simulate);
	GraspReturnType requestGraspObject(const std::string &obj, const std::string &surface, bool simulate);
	GraspReturnType requestPlaceObject(EefPose obj, bool simulate);
	GraspReturnType requestPlaceObject(ObjectShape obj, bool simulate);
	GraspReturnType requestPlaceObject(const std::string &surface, bool simulate);
	std::string requestNearestPose() const;
	ArmPoses requestPoses() const;

	std::string requestGetSurfaceByHeight(const float);

private:
	Model::Ptr model;
	ControlStrategy::Ptr strategy;
	std::vector<ControlInterface::Ptr> controlInterfaces;

};

