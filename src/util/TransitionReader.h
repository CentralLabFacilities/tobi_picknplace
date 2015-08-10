/*
 * PoseReader.h
 *
 *  Created on: 12.07.2014
 *      Author: leon
 */

#pragma once

#include <vector>
#include <string>
#include <sstream>
#include "Transition.h"
#include <rsc/logging/Logger.h>

class ReaderException: public std::exception {
public:
	std::string msg;
	ReaderException() _GLIBCXX_USE_NOEXCEPT { }
	virtual ~ReaderException() _GLIBCXX_USE_NOEXCEPT{}
	virtual const char* what() const _GLIBCXX_USE_NOEXCEPT {
		return msg.c_str();
	}
};

class TransitionsReader {
public:

	TransitionsReader();
	virtual ~TransitionsReader();

	std::vector<Transition> read(const std::string &filePath);
private:
	static rsc::logging::LoggerPtr logger;
};
