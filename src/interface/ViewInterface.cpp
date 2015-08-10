/*
 * ViewInterface.cpp
 *
 *  Created on: Apr 29, 2015
 *      Author: lziegler
 */

#include "ViewInterface.h"

using namespace std;

ViewInterface::ViewInterface() {

}

ViewInterface::~ViewInterface() {
}

void ViewInterface::setListener(ControlInterfaceListener* listener) {
	this->listener = listener;
}

void ViewInterface::removeListener() {
	this->listener = 0;
}
