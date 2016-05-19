/*
 * KatanaSimModel.h
 *
 *  Created on: Apr 29, 2015
 *      Author: lziegler
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <vector>

#include "../Model.h"

#define KATANA_SIM_NAME "katana_sim"

class KatanaSim: public Model {
public:
	typedef boost::shared_ptr<KatanaSim> Ptr;

	KatanaSim();
	~KatanaSim();

	virtual std::map<std::string, double> getJointAngles() const;
	virtual std::vector<std::string> getJointNames() const;
    virtual void setJointAngle(const std::string &joint, double angle);
    virtual void setJointAngles(const std::map<std::string, double> &angle);
    virtual void setJointAngles(const std::vector<double> &angles);
	virtual int getNumJoints() const;

	virtual void openEef(bool withSensors);
	virtual void closeEef(bool withSensors);
	virtual void moveToGripper(double target, bool withSensors);

	virtual void motorsOn();
	virtual void motorsOff();

	virtual EefPose getEefPose() const;
	virtual ArmPoses getRememberedPoses() const;
	virtual ArmPose getRememberedPose(const std::string &name) const;
	virtual MoveResult moveTo(const EefPose &pose, bool linear, bool orientation);
	virtual MoveResult moveTo(const std::string &poseName, bool plan);
	virtual void stop() const;

	virtual bool isSomethingInGripper() const;
	virtual SensorReadings getGripperSensors() const;
	
	virtual void fillGrasp(moveit_msgs::Grasp& grasp);

	virtual GraspReturnType graspObject(ObjectShape obj, bool simulate, const std::string &startPose="");
	virtual GraspReturnType graspObject(const std::string &obj, const std::string &surface, bool simulate, const std::string &startPose="");
	virtual GraspReturnType placeObject(ObjectShape obj, bool simulate, const std::string &startPose="");
	virtual GraspReturnType placeObject(EefPose obj, bool simulate, const std::string &startPose="");
	virtual GraspReturnType placeObject(const std::string &surface, bool simulate, const std::string &startPose="");

};

