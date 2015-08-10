/*
 * ControlInterface.h
 *
 *  Created on: Apr 29, 2015
 *      Author: lziegler
 */

#pragma once

#include <vector>
#include <boost/shared_ptr.hpp>

#include "ControlInterfaceListener.h"

class ControlInterface {
public:
	typedef boost::shared_ptr<ControlInterface> Ptr;

	ControlInterface() {
	}
	virtual ~ControlInterface() {
	}

	virtual void setListener(ControlInterfaceListener* listener) = 0;
	virtual void removeListener() = 0;

};

