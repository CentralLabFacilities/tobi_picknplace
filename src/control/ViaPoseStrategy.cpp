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
#include <ros/console.h>

using namespace std;
using namespace boost;

const double ERROR_THRESHOLD = 100;

const string RECOVER_POSE = "via_default";

ViaPoseStrategy::ViaPoseStrategy(const Model::Ptr& model) :
		model(model) {
}

ViaPoseStrategy::~ViaPoseStrategy() {
}

void ViaPoseStrategy::setTransitions(const std::vector<Transition>& transitions) {
	for (int i = 0; i < transitions.size(); i++) {
		ROS_DEBUG_STREAM("Add transition: " << transitions[i].source << " -> " << transitions[i].target<< " (w: " << transitions[i].weight << ")");
		Edge e(transitions[i].source, transitions[i].target, transitions[i].weight);
		dijkstraPlanner.addEdge(e);
	}
}

GraspReturnType ViaPoseStrategy::graspObject(ObjectShape obj, bool simulate) {
	model->openGripper(false);
	GraspReturnType ret = model->graspObject(obj, simulate);

	if (ret.result == GraspReturnType::ROBOT_CRASHED) {
		// recover !!!
		ROS_WARN_STREAM("  try to RECOVER !!");
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
        ROS_WARN_STREAM("Crash detected. Try to RECOVER !!");
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
		ROS_WARN_STREAM("  try to RECOVER !!");
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
		ROS_WARN_STREAM("  try to RECOVER !!");
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
bool ViaPoseStrategy::planTo(const string& poseName)
{
	MoveResult success = model->moveTo(poseName, true);
	if (success != SUCCESS) {
		ROS_ERROR_STREAM("Cannot find a plan to pose \"" << poseName << "\"");
		return false;
	} else {
		return true;
	}
}


GraspReturnType ViaPoseStrategy::placeObject(const string &surface, bool simulate) {
    GraspReturnType ret = model->placeObject(surface, simulate);

    if (ret.result == GraspReturnType::ROBOT_CRASHED) {
        // recover !!!
        ROS_WARN_STREAM("  try to RECOVER !!");
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

	ROS_INFO_STREAM("  recover to pose: " << RECOVER_POSE);
	MoveResult success = model->moveTo(RECOVER_POSE);
	if (success != SUCCESS) {
		return false;
	}
	return moveTo(poseName, false);
}

bool ViaPoseStrategy::moveTo(const std::string& poseName, bool withRecovery) {

    ROS_INFO_STREAM("Strategy for moveTo(" << poseName << ")");
	ROS_INFO_STREAM("looking for nearest pose...");
	string start = findNearestPose();
	ROS_INFO_STREAM("nearest pose: " << start);

	typedef vector<string> Path;
	Path path;

	if (start != poseName) {
        ROS_INFO_STREAM("calculate shortest path...");
        path = dijkstraPlanner.getShortestPath(start, poseName);
        if (path.empty()) {
            ROS_WARN_STREAM("no path found for start pose: " << start);
            return false;
        }

        ROS_INFO_STREAM("Path found:  ");
        for (Path::iterator pathIt = path.begin(); pathIt != path.end(); ++pathIt) {
            ROS_INFO_STREAM(" - " << *pathIt);
        }
	}

	if (calcErrorToPose(start) < 0.03) {
        ROS_DEBUG_STREAM("Start pose is reached");
    } else {
        ROS_WARN_STREAM(
                "!!!WARNING!!!! Current pose is not a known pose. Planning path to " << start);
        bool doPlanning = true;
        ROS_DEBUG_STREAM("Moving to start pose");
        MoveResult success = model->moveTo(start, doPlanning);
        if (success != SUCCESS) {
            ROS_ERROR_STREAM("Cannot find a plan to starting pose \"" << start << "\"");
            return false;
        }
    }

	bool moveGripper = true;
	if (model->isSomethingInGripper()) {
		ROS_WARN_STREAM("Something is in gripper. We dont move the gripper-joint");
		moveGripper = false;
	}

	// execute path
	for (Path::iterator pathIt = path.begin(); pathIt != path.end(); ++pathIt) {
		bool last = pathIt == path.end() - 1;
		string currentTargetPose = *pathIt;

		ROS_INFO_STREAM("  next pose: " << currentTargetPose);
		MoveResult success;
        bool doPlanning = false;
        success = model->moveTo(currentTargetPose, doPlanning);

		if (success == SUCCESS) {
			ROS_INFO_STREAM("    result: success");
		} else if (success == NOPLAN) {
			ROS_INFO_STREAM("    result: no plan");
			if (last) {
				return false;
			}
			ROS_INFO_STREAM("  skip pose");
		} else if (success == ENV_CHANGE) {
			ROS_INFO_STREAM("    result: env change");
			ROS_INFO_STREAM("  try again");
			pathIt--;
			continue;
		} else if (success == CRASH) {
			ROS_INFO_STREAM("    result: crash");
			if (last) {
				return false;
			}
			if (withRecovery) {
				ROS_INFO_STREAM("  recover from crash");
				return recoverAndMoveTo(poseName);
			} else {
				return false;
			}
		} else {
			ROS_INFO_STREAM("    result: other");
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
	ROS_DEBUG_STREAM("Poses we know: " << rememberedPoses.size());
	for (it = rememberedPoses.begin(); it != rememberedPoses.end(); ++it) {
		double squareError = calcErrorToPose(it->first);
		ROS_DEBUG_STREAM("square error to " << it->first << ": " << squareError);
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
		ROS_ERROR_STREAM("Named pose is unknown: " << pose);
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
	squareError += pow(angles["katana_motor4_lift_joint"] - joints["katana_motor4_lift_joint"], 2) * 0.5;
	squareError += pow(angles["katana_motor5_wrist_roll_joint"] - joints["katana_motor5_wrist_roll_joint"], 2) * 0.1;
	squareError /= 3;
	ROS_DEBUG_STREAM("Error to Pose: " << pose << " is: " << squareError);
	return squareError;

}
