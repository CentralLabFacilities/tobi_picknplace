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

	double gripperPositionClosed;
	double gripperPositionOpen;

	int gripperThresholdForce;
	int gripperThresholdDistance;

	double approachDesiredDistance;
	double approachMinDistance;

	double liftUpDesiredDistance;
	double liftUpMinDistance;

	double retreatDesiredDistance;
	double retreatMinDistance;

	double graspThroughDistance;

	std::string frameOriginArm;
	std::string frameOriginGripper;

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

