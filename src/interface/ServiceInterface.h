/*
 * ServiceInterface.h
 *
 *  Created on: Apr 29, 2015
 *      Author: lziegler
 */

#pragma once

#include "ControlInterface.h"

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>


class ServiceInterface: public ControlInterface {
public:
	typedef boost::shared_ptr<ServiceInterface> Ptr;

	ServiceInterface(const std::string &service_name);
	virtual ~ServiceInterface();

	virtual void setListener(ControlInterfaceListener* listener);
	virtual void removeListener();

private:

	class Private;
	boost::shared_ptr<Private> d;
};

