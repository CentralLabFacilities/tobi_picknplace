/*
 * ViaPoseStrategy.h
 *
 *  Created on: May 5, 2015
 *      Author: lziegler
 */

#pragma once

#include "ControlStrategy.h"
#include "../util/Dijkstra.h"


class ViaPoseStrategy: public ControlStrategy {
public:
	typedef boost::shared_ptr<ViaPoseStrategy> Ptr;
	ViaPoseStrategy(const Model::Ptr &model);
	virtual ~ViaPoseStrategy();

	virtual void setTransitions(const std::vector<Transition> &transitions);
	virtual std::string findNearestPose() const;
	//virtual GraspReturnType graspObject(ObjectShape obj, bool simulate);
	virtual GraspReturnType graspObject(const std::string &obj, const std::string &surface, bool simulate);
	//virtual GraspReturnType placeObject(ObjectShape obj, bool simulate);
	virtual GraspReturnType placeObject(EefPose obj, bool simulate);
	virtual GraspReturnType placeObject(const std::string &surface, bool simulate);
	virtual bool planTo(const std::string &poseName);
	virtual bool moveTo(const std::string &poseName, bool withRecovery = true);
	virtual bool recoverAndMoveTo(const std::string &poseName);

private:

	double calcErrorToPose(const std::string& pose) const;
	DijkstraAlgorithm dijkstraPlanner;
	Model::Ptr model;

};
