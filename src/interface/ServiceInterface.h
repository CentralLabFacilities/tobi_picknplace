/*
 * ServiceInterface.h
 *
 *  Created on: Apr 29, 2015
 *      Author: lziegler
 */

#pragma once

#include "ControlInterface.h"


class ServiceInterface: public ControlInterface {
public:
	typedef boost::shared_ptr<ServiceInterface> Ptr;

	ServiceInterface(std::string service_name);
	virtual ~ServiceInterface();

	virtual void setListener(ControlInterfaceListener* listener);
	virtual void removeListener();

private:
	std::string serverScope;

	class Private;
	boost::shared_ptr<Private> d;
};

