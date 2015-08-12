/*
 * Model.h
 *
 *  Created on: Apr 29, 2015
 *      Author: lziegler
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <vector>
#include <map>
#include <string>
#include <stdint.h>
#include <time.h>
#include <sys/time.h>

#include "ModelListener.h"

typedef std::map<std::string, double> ArmPose;
typedef std::map<std::string, ArmPose > ArmPoses;
typedef std::map<std::string, short> SensorReadings;

enum MoveResult {
	SUCCESS, CRASH, NOPLAN, ENV_CHANGE, OTHER
};

class Vec {
public:
	Vec() :
			xMeter(0), yMeter(0), zMeter(0) {
	}
	Vec(const Vec &other) :
			xMeter(other.xMeter), yMeter(other.yMeter), zMeter(other.zMeter), frame(other.frame) {
	}
	virtual ~Vec() {
	}
	double xMeter, yMeter, zMeter;
	std::string frame;
};

class Rot {
public:
	Rot() :
			qw(0), qx(0), qy(0), qz(0) {
	}
	Rot(const Rot &other) :
			qw(other.qw), qx(other.qx), qy(other.qy), qz(other.qz), frame(other.frame) {
	}
	virtual ~Rot() {
	}
	double qw, qx, qy, qz;
	std::string frame;
};

class EefPose {
public:
	EefPose() {
	}
	EefPose(const EefPose &other) :
			translation(other.translation), rotation(other.rotation), frame(other.frame) {
	}
	virtual ~EefPose() {
	}
	Vec translation;
	Rot rotation;
	std::string frame;
};

class GraspReturnType {
public:
	enum GraspResult {
		SUCCESS, POSITION_UNREACHABLE, ROBOT_CRASHED, FAIL, COLLISION_HANDLED, NO_RESULT
	};

	GraspReturnType() :
			rating(0), result(NO_RESULT) {
	}
	GraspReturnType(const GraspReturnType& other) :
			rating(other.rating), result(other.result), point(other.point) {
	}
	virtual ~GraspReturnType() {
	}

	static std::string resultToString(GraspResult result) {
		switch (result) {
		case SUCCESS:
			return "SUCCESS";
		case POSITION_UNREACHABLE:
			return "POSITION_UNREACHABLE";
		case ROBOT_CRASHED:
			return "ROBOT_CRASHED";
		case FAIL:
			return "FAIL";
		case COLLISION_HANDLED:
			return "COLLISION_HANDLED";
		default:
			return "NO_RESULT";
		}
	}
	static GraspResult resultFromString(const std::string &name) {
		if (name == "SUCCESS") {
			return SUCCESS;
		} else if (name == "POSITION_UNREACHABLE") {
			return POSITION_UNREACHABLE;
		} else if (name == "ROBOT_CRASHED") {
			return ROBOT_CRASHED;
		} else if (name == "FAIL") {
			return FAIL;
		} else {
			return NO_RESULT;
		}
	}

	Vec point;
	double rating;
	GraspResult result;
};

class ObjectShape {
public:
	ObjectShape() :
			widthMeter(0), heightMeter(0), depthMeter(0) {
	}
	ObjectShape(const ObjectShape &other) :
			widthMeter(other.widthMeter),
			heightMeter(other.heightMeter),
			depthMeter(other.depthMeter),
			center(other.center) {
	}
	virtual ~ObjectShape() {
	}

	Vec center;
	double widthMeter, heightMeter, depthMeter;
};

class Timestamp {
public:
	static uint64_t currentTimeMillis() {
	    timeval tv;
	    gettimeofday(&tv, NULL);

	    // (second-value in milliseconds) + (microsecond value in milliseconds)
	    return (((uint64_t) tv.tv_sec) * 1000l) + (tv.tv_usec / 1000l);
	}
};
