/*
 * KatanaSimModel.h
 *
 *  Created on: Apr 29, 2015
 *      Author: lziegler
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <vector>

#include "Model.h"

class KatanaSimModel: public Model {
public:
	typedef boost::shared_ptr<KatanaSimModel> Ptr;

	KatanaSimModel();
	virtual ~KatanaSimModel();

	void addListener(ModelListener* listener);
	void removeListener(ModelListener* listener);

	virtual std::vector<double> getJointAngles() const;
	virtual void setJointAngle(int joint, double angle);
	virtual void setJointAngles(const std::vector<double> &angle);
	virtual int getNumJoints() const;

	virtual void openGripper(bool withSensors);
	virtual void closeGripper(bool withSensors);
	virtual void moveToGripper(double target, bool withSensors);

	virtual void motorsOn();
	virtual void motorsOff();

	virtual EefPose getEefPose() const;
	virtual Poses getRememberedPoses() const;
	virtual MoveResult moveTo(const EefPose &pose, bool linear, bool orientation);
	virtual MoveResult moveTo(const std::string &poseName);
	virtual void stop() const;

	virtual bool isSomethingInGripper() const;
	virtual SensorReadings getGripperSensors() const;

	virtual GraspReturnType graspObject(ObjectShape obj, bool simulate, const std::string &startPose="");
	virtual GraspReturnType placeObject(ObjectShape obj, bool simulate, const std::string &startPose="");
	virtual GraspReturnType placeObject(EefPose obj, bool simulate, const std::string &startPose="");

};

