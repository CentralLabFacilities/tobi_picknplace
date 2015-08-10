/*
 * RsbInterface.h
 *
 *  Created on: Apr 29, 2015
 *      Author: lziegler
 */

#pragma once

#include "ControlInterface.h"

#include <rsc/logging/Logger.h>

class RsbInterface: public ControlInterface {
public:
	typedef boost::shared_ptr<RsbInterface> Ptr;

	RsbInterface(const std::string &serverScope);
	virtual ~RsbInterface();

	void init();

	virtual void setListener(ControlInterfaceListener* listener);
	virtual void removeListener();

private:
	std::string serverScope;

    class Private;
    boost::shared_ptr<Private> d;
};

