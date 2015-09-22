/*
 * ViaPoseStrategy.cpp
 *
 *  Created on: May 5, 2015
 *      Author: lziegler
 */

#include "ViaPoseStrategy.h"
#include <ros/ros.h>
#include <limits>
#include <cmath>

using namespace std;
using namespace boost;

const double ERROR_THRESHOLD = 100;

const string RECOVER_POSE = "via_default";

rsc::logging::LoggerPtr ViaPoseStrategy::logger = rsc::logging::Logger::getLogger(
		"picknplace.ViaPoseStrategy");

ViaPoseStrategy::ViaPoseStrategy(const Model::Ptr& model) :
		model(model) {
}

ViaPoseStrategy::~ViaPoseStrategy() {
}

void ViaPoseStrategy::setTransitions(const std::vector<Transition>& transitions) {
	for (int i = 0; i < transitions.size(); i++) {
		RSCDEBUG(logger, "Add transition: " << transitions[i].source << " -> " << transitions[i].target<< " (w: " << transitions[i].weight << ")");
		Edge e(transitions[i].source, transitions[i].target, transitions[i].weight);
		dijkstraPlanner.addEdge(e);
	}
}

GraspReturnType ViaPoseStrategy::graspObject(ObjectShape obj, bool simulate) {
	model->openGripper(false);
	GraspReturnType ret = model->graspObject(obj, simulate);

	if (ret.result == GraspReturnType::ROBOT_CRASHED) {
		// recover !!!
		RSCWARN(logger, "  try to RECOVER !!");
		if (model->isSomethingInGripper()) {
			if (moveTo("carry_side")) {
				ret.result = GraspReturnType::SUCCESS;
			}
		} else {
			if (moveTo("fold_up")) {
				ret.result = GraspReturnType::COLLISION_HANDLED;
			}
		}
	}

	return ret;
}

GraspReturnType ViaPoseStrategy::graspObject(const string &obj, const std::string &surface, bool simulate) {
    model->openGripper(false);
    GraspReturnType ret = model->graspObject(obj, surface, simulate);

    if (ret.result == GraspReturnType::ROBOT_CRASHED) {
        // recover !!!
        RSCWARN(logger, "  try to RECOVER !!");
        if (model->isSomethingInGripper()) {
            if (moveTo("carry_side")) {
                ret.result = GraspReturnType::SUCCESS;
            }
        } else {
            if (moveTo("fold_up")) {
                ret.result = GraspReturnType::COLLISION_HANDLED;
            }
        }
    }

    return ret;
}

GraspReturnType ViaPoseStrategy::placeObject(ObjectShape obj, bool simulate) {
	GraspReturnType ret = model->placeObject(obj, simulate);

	if (ret.result == GraspReturnType::ROBOT_CRASHED) {
		// recover !!!
		RSCWARN(logger, "  try to RECOVER !!");
		if (model->isSomethingInGripper()) {
			if (moveTo("carry_side")) {
				ret.result = GraspReturnType::COLLISION_HANDLED;
			}
		} else {
			if (moveTo("fold_up")) {
				ret.result = GraspReturnType::SUCCESS;
			}
		}
	}
	return ret;
}

GraspReturnType ViaPoseStrategy::placeObject(EefPose obj, bool simulate) {
	GraspReturnType ret = model->placeObject(obj, simulate);

	if (ret.result == GraspReturnType::ROBOT_CRASHED) {
		// recover !!!
		RSCWARN(logger, "  try to RECOVER !!");
		if (model->isSomethingInGripper()) {
			if (moveTo("carry_side")) {
				ret.result = GraspReturnType::COLLISION_HANDLED;
			}
		} else {
			if (moveTo("fold_up")) {
				ret.result = GraspReturnType::SUCCESS;
			}
		}
	}
	return ret;
}

GraspReturnType ViaPoseStrategy::placeObject(const string &surface, bool simulate) {
    GraspReturnType ret = model->placeObject(surface, simulate);

    if (ret.result == GraspReturnType::ROBOT_CRASHED) {
        // recover !!!
        RSCWARN(logger, "  try to RECOVER !!");
        if (model->isSomethingInGripper()) {
            if (moveTo("carry_side")) {
                ret.result = GraspReturnType::COLLISION_HANDLED;
            }
        } else {
            if (moveTo("fold_up")) {
                ret.result = GraspReturnType::SUCCESS;
            }
        }
    }
    return ret;
}

bool ViaPoseStrategy::recoverAndMoveTo(const std::string& poseName) {

	RSCINFO(logger, "  recover to pose: " << RECOVER_POSE);
	MoveResult success = model->moveTo(RECOVER_POSE);
	if (success != SUCCESS) {
		return false;
	}
	return moveTo(poseName, false);
}

bool ViaPoseStrategy::moveTo(const std::string& poseName, bool withRecovery) {

    RSCINFO(logger, "Strategy for moveTo(" << poseName << ")");
	RSCINFO(logger, "looking for nearest pose...");
	string start = findNearestPose();
	RSCINFO(logger, "nearest pose: " << start);

	if (poseName == start) {
	    RSCINFO(logger, "already there yet!");
	    return true;
	}

	RSCINFO(logger, "calculate shortest path...");
	typedef vector<string> Path;
	Path path = dijkstraPlanner.getShortestPath(start, poseName);
	if (path.empty()) {
		RSCWARN(logger, "no path found for start pose: " << start);
		return false;
	}

	RSCINFO(logger, "Path found:  ");
	for (Path::iterator pathIt = path.begin(); pathIt != path.end(); ++pathIt) {
		RSCINFO(logger, " - " << *pathIt);
	}

	bool moveGripper = true;
	if (model->isSomethingInGripper()) {
		RSCWARN(logger, "Something is in gripper. We dont move the gripper-joint");
		moveGripper = false;
	}

	// execute path
	for (Path::iterator pathIt = path.begin(); pathIt != path.end(); ++pathIt) {
		bool first = pathIt == path.begin();
		bool last = pathIt == path.end() - 1;
		string currentTargetPose = *pathIt;

		RSCINFO(logger, "  next pose: " << currentTargetPose);
		MoveResult success;
		if (first) {
		    if (calcErrorToPose(currentTargetPose) < 0.03) {
                RSCDEBUG(logger, "First Pose is reached");
                continue;
            } else {
                RSCWARN(logger,
                        "!!!WARNING!!!! Starting pose is not a known pose. Planning path to " << currentTargetPose);
                bool doPlanning = true;
                success = model->moveTo(currentTargetPose, doPlanning);
            }
		} else {
		    bool doPlanning = false;
		    success = model->moveTo(currentTargetPose, doPlanning);
		}
		if (success == SUCCESS) {
			RSCINFO(logger, "    result: success");
		} else if (success == NOPLAN) {
			RSCINFO(logger, "    result: no plan");
			if (last) {
				return false;
			}
			RSCINFO(logger, "  skip pose");
		} else if (success == ENV_CHANGE) {
			RSCINFO(logger, "    result: env change");
			RSCINFO(logger, "  try again");
			pathIt--;
			continue;
		} else if (success == CRASH) {
			RSCINFO(logger, "    result: crash");
			if (last) {
				return false;
			}
			if (withRecovery) {
				RSCINFO(logger, "  recover from crash");
				return recoverAndMoveTo(poseName);
			} else {
				return false;
			}
		} else {
			RSCINFO(logger, "    result: other");
			return false;
		}
	}
	return true;
}

string ViaPoseStrategy::findNearestPose() const {

	ROS_INFO("Invoked findNearestPose");
	std::string bestPose = "NoUsablePoseFound";

	double lowestError = numeric_limits<double>().max();
	ArmPoses rememberedPoses = model->getRememberedPoses();

	ArmPoses::iterator it;
	RSCDEBUG(logger, "Poses we know: " << rememberedPoses.size());
	for (it = rememberedPoses.begin(); it != rememberedPoses.end(); ++it) {
		double squareError = calcErrorToPose(it->first);
		RSCDEBUG(logger, "square error to " << it->first << ": " << squareError);
		if (lowestError > squareError) {
			lowestError = squareError;
			if (lowestError <= ERROR_THRESHOLD) {
				bestPose = it->first;
			}
		}
	}
        
	return bestPose;
}

double ViaPoseStrategy::calcErrorToPose(const std::string& pose) const {

	ArmPoses rememberedPoses = model->getRememberedPoses();
	if (!rememberedPoses.count(pose)) {
		RSCERROR(logger, "Named pose is unknown: " << pose);
		return numeric_limits<double>::max();
	}

	map<string, double> angles = rememberedPoses[pose];
	map<string, double> joints = model->getJointAngles();

	double squareError = 0;

	// look only at first 3 joints, with descending weight
	// TODO if joint 2 points straight, joint 1 is not so important
	squareError += pow(angles["katana_motor1_pan_joint"]  - joints["katana_motor1_pan_joint"],  2) * 3;
	squareError += pow(angles["katana_motor2_lift_joint"] - joints["katana_motor2_lift_joint"], 2) * 2;
	squareError += pow(angles["katana_motor3_lift_joint"] - joints["katana_motor3_lift_joint"], 2) * 1;
	squareError /= 3;
	RSCDEBUG(logger, "Error to Pose: " << pose << " is: " << squareError);
	return squareError;

}
