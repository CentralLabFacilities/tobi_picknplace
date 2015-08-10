/*
 * ViewInterface.h
 *
 *  Created on: Apr 29, 2015
 *      Author: lziegler
 */

#pragma once

#include "ControlInterface.h"

class ViewInterface: public ControlInterface {
public:
	typedef boost::shared_ptr<ViewInterface> Ptr;

	ViewInterface();
	virtual ~ViewInterface();

	virtual void setListener(ControlInterfaceListener* listener);
	virtual void removeListener();

private:
	ControlInterfaceListener* listener;
};

