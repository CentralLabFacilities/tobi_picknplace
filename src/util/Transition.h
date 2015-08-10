/*
 * Transition.h
 *
 *  Created on: 12.07.2014
 *      Author: leon
 */

#pragma once

#include <string>

class Transition {
public:
	Transition():weight(0){
	}
	virtual ~Transition(){
	}

	std::string source;
	std::string target;
	double weight;
};
