/*
 * ServiceInterface.h
 *
 *  Created on: Jul 12, 2017
 *      Author: ffriese
 */

#pragma once

#include "ControlInterface.h"
#include <ros/ros.h>
#include "tobi_picknplace/BironPostureExecution.h"
#include "boost/shared_ptr.hpp"

class ServiceInterface: public ControlInterface {
public:
   // typedef boost::shared_ptr<ServiceInterface> Ptr;

    ServiceInterface(const std::string &serviceName);
    virtual ~ServiceInterface();

	virtual void setListener(ControlInterfaceListener* listener);
	virtual void removeListener();

private:
    std::string serviceName;

    boost::shared_ptr<ros::ServiceServer> servicePtr;
    ros::NodeHandle nodeHandle;
    ControlInterfaceListener* listener;

    bool postureCallback(tobi_picknplace::BironPostureExecution::Request &request, tobi_picknplace::BironPostureExecution::Response &response);
};

