/*
 * ControlInterfaceListener.h
 *
 *  Created on: Apr 29, 2015
 *      Author: lziegler
 */

#pragma once

#include "../model/ModelTypes.h"

#include <string>
#include <vector>

class ControlInterfaceListener {
public:
	ControlInterfaceListener(){
	}
	virtual ~ControlInterfaceListener() {
	}

	virtual void requestMoveJointAngles(const std::vector<double> &angle) = 0;
	virtual int requestNumJoints() const = 0;
	virtual std::map<std::string, double> requestJointAngles() const = 0;

	virtual void requestOpenGripper(bool withSensors) = 0;
	virtual void requestCloseGripper(bool withSensors) = 0;

	virtual void requestMotorsOn() = 0;
	virtual void requestMotorsOff() = 0;

	virtual EefPose requestEefPose() const = 0;
	virtual bool requestMoveTo(const EefPose &pose, bool linear, bool orientation) = 0;
	virtual bool requestMoveTo(const std::string &poseName) = 0;

	virtual SensorReadings requestGripperSensors() const = 0;
	virtual bool requestIsSomethingInGripper() const = 0;

	virtual GraspReturnType requestGraspObject(ObjectShape obj, bool simulate) = 0;
	virtual GraspReturnType requestPlaceObject(EefPose obj, bool simulate) = 0;
	virtual GraspReturnType requestPlaceObject(ObjectShape obj, bool simulate) = 0;
	virtual std::string requestNearestPose() const = 0;
    virtual ArmPoses requestPoses() const = 0;
};

class EmptyControlInterfaceListener: public ControlInterfaceListener {
public:
	void requestMoveJointAngles(const std::vector<double> &angle){}
	int requestNumJoints() const{return -1;}
	std::map<std::string, double> requestJointAngles() const{return std::map<std::string, double>();}

	void requestOpenGripper(bool withSensors){}
	void requestCloseGripper(bool withSensors){}

	void requestMotorsOn(){}
	void requestMotorsOff(){}

	EefPose requestEefPose() const{return EefPose();}
	bool requestMoveTo(const EefPose &pose, bool linear, bool orientation){return false;}
	bool requestMoveTo(const std::string &poseName){return false;}

	SensorReadings requestGripperSensors() const{return SensorReadings();}
	bool requestIsSomethingInGripper() const{return false;}

	GraspReturnType requestGraspObject(ObjectShape obj, bool simulate){return GraspReturnType();}
	GraspReturnType requestPlaceObject(EefPose obj, bool simulate){return GraspReturnType();}
	GraspReturnType requestPlaceObject(ObjectShape obj, bool simulate){return GraspReturnType();}
	std::string requestNearestPose() const{return std::string();}
	ArmPoses requestPoses() const{return ArmPoses();};
};


