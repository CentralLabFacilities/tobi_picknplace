/*
 * ParamReader.h
 *
 *  Created on: Jul 12, 2015
 *      Author: lziegler
 */

#pragma once

#include <ros/ros.h>
#include <rsc/patterns/Singleton.h>

class ParamReader: private rsc::patterns::Singleton<ParamReader> {
public:
	virtual ~ParamReader();
	friend class rsc::patterns::Singleton<ParamReader>;

	static ParamReader& getParamReader();

	std::string model;

    std::string groupArm;
    std::string groupEef;

    std::string frameArm;
    std::string frameGripper;

    std::string endEffector;
    std::string eefCmdScope;

    std::string plannerId;
    std::string graspGen;
    int planningTime;

	std::vector<double> graspRot;
	std::vector<double> eefPosClosed;
	std::vector<double> eefPosOpen;

	std::vector<std::string> touchLinks;

	double goalJointTolerance;
	double goalPositionTolerance;
	double goalOrientationTolerance;

	int gripperThresholdForce;
	int gripperThresholdDistance;

	double approachDesiredDistance;
	double approachMinDistance;

	double liftUpDesiredDistance;
	double liftUpMinDistance;

	double retreatDesiredDistance;
	double retreatMinDistance;

	double graspThroughDistance;

	double anglePitchInc;
	double anglePitchMax;

	double angleYawInc;
	double angleYawMax;

	double transInc;
	double transMax;

	double placeAtInc;
	double placeAtMax;
	double placeAtAnglePitchInc;
	double placeAtAnglePitchMax;
	double placeAtAngleYawInc;
	double placeAtAngleYawMax;

	double placeXInc;
	double placeYZInc;
	double placeXMax;
	double placeYMax;
	double placeZMax;

	double heightSkipTop;
	double heightSkipBottom;
	double heightInc;

private:
	ros::NodeHandle private_nh_;

	ParamReader();
	static ParamReader& getInstanceBase();

};

