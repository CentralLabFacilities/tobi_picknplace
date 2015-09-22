/*
 * PoseReader.cpp
 *
 *  Created on: 12.07.2014
 *      Author: leon
 */

#include "Transition.h"
#include <fstream>
#include <iostream>
#include <exception>
#include <boost/algorithm/string.hpp>
#include <ros/console.h>
#include "TransitionReader.h"

using namespace std;

TransitionsReader::TransitionsReader() {
}
TransitionsReader::~TransitionsReader() {
}

vector<Transition> TransitionsReader::read(const std::string &filename) {

	ifstream file(filename.c_str());
	string str;
	vector<Transition> transitions;
	while (getline(file, str)) {
		boost::trim(str);
		vector<string> strs;
		boost::split(strs, str, boost::is_any_of(":;| \t"), boost::token_compress_on);
		if (strs.size() != 3) {
			stringstream ss;
			ss << "Malformatted line in " << filename << " : \"" << str << "\". Expected Format: \"<posename>:<posename>:<weight>\"";
			ReaderException exception;
			exception.msg = ss.str();
			ROS_ERROR_STREAM(ss.str());
			throw exception;
		}
		Transition t;
		t.source = strs[0];
		t.target = strs[1];
		t.weight = atof(strs[2].c_str());
		transitions.push_back(t);
	}
	return transitions;
}
