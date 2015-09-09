/*
 * ControlStrategy.h
 *
 *  Created on: May 5, 2015
 *      Author: lziegler
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <vector>
#include <string>

#include "../model/Model.h"
#include "../util/Transition.h"

class ControlStrategy {
public:
	typedef boost::shared_ptr<ControlStrategy> Ptr;
	ControlStrategy(){
	}
	virtual ~ControlStrategy() {
	}

	virtual GraspReturnType graspObject(const std::string &obj, const std::string &surface, bool simulate) = 0;
	virtual GraspReturnType graspObject(ObjectShape obj, bool simulate) = 0;
	virtual GraspReturnType placeObject(ObjectShape obj, bool simulate) = 0;
	virtual GraspReturnType placeObject(EefPose obj, bool simulate) = 0;
	virtual GraspReturnType placeObject(const std::string &surface, bool simulate) = 0;
    virtual std::string findNearestPose() const = 0;
	virtual bool moveTo(const std::string &poseName, bool withRecovery = true) = 0;

};

