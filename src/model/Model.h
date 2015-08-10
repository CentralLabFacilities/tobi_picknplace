/*
 * Model.h
 *
 *  Created on: Apr 29, 2015
 *      Author: lziegler
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <vector>

#include "ModelListener.h"
#include "ModelTypes.h"

class Model {
public:
	typedef boost::shared_ptr<Model> Ptr;

	Model() {
	}
	virtual ~Model() {
	}

	virtual void addListener(ModelListener* listener) = 0;
	virtual void removeListener(ModelListener* listener) = 0;

	virtual std::vector<double> getJointAngles() const = 0;
	virtual void setJointAngle(int joint, double angle) = 0;
	virtual void setJointAngles(const std::vector<double> &angle) = 0;
	virtual int getNumJoints() const = 0;

	virtual void openGripper(bool withSensors) = 0;
	virtual void closeGripper(bool withSensors) = 0;

	virtual void motorsOn() = 0;
	virtual void motorsOff() = 0;

	virtual EefPose getEefPose() const = 0;
	virtual Poses getRememberedPoses() const = 0;
	virtual MoveResult moveTo(const EefPose &pose, bool linear, bool orientation) = 0;
	virtual MoveResult moveTo(const std::string &poseName) = 0;
	virtual void stop() const = 0;

	virtual bool isSomethingInGripper() const = 0;
	virtual SensorReadings getGripperSensors() const = 0;

	virtual GraspReturnType graspObject(ObjectShape object, bool simulate, const std::string &startPose="") = 0;
	virtual GraspReturnType placeObject(ObjectShape obj, bool simulate, const std::string &startPose="") = 0;
	virtual GraspReturnType placeObject(EefPose obj, bool simulate, const std::string &startPose="") = 0;
};

