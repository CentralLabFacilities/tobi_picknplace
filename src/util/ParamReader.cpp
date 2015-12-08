/*
 * ParamReader.cpp
 *
 *  Created on: Jul 12, 2015
 *      Author: lziegler
 */

#include "ParamReader.h"

using namespace std;

static const string MODEL = "katana";

static const string GROUP_ARM = "left_arm";
static const string GROUP_END_EFFECTOR = "left_hand";

static const string FRAME_ARM = "base_link";
static const string FRAME_GRIPPER = "left_palm";
static const string END_EFFECTOR = "end_effector";

static const string EEF_CMD_SCOPE = "/meka_roscontrol/left_hand_position_trajectory_controller/command";

static const string PLANNER_ID = "RRTConnectkConfigDefault";
static const int PLANNING_TIME = 120;

static const vector<double> GRASP_ROTATION = {0, -1,5708, 0};
static const vector<double> EEF_POSITION_CLOSED = {-0.495};
static const vector<double> EEF_POSITION_OPEN = {0.298};

static const vector<string> TOUCH_LINKS = {};

static const double GOAL_JOINT_TOLERANCE = 0.01;
static const double GOAL_POSITION_TOLERANCE = 0.02;
static const double GOAL_ORIENTATION_TOLERANCE = 0.5;

// KATANA SPECIFIC todo: organize in another way

static const double APPROACH_DESIRED_DISTANCE = 0.10;
static const double APPROACH_MIN_DISTANCE = 0.05;

static const double LIFTUP_DESIRED_DISTANCE = 0.02;
static const double LIFTUP_MIN_DISTANCE = 0.005;

static const double RETREAT_DESIRED_DISTANCE = 0.07;
static const double RETREAT_MIN_DISTANCE = 0.03;

static const double GRASP_THROUGH_DISTANCE = 0.03;

static const int GRIPPER_THRESHOLD_FORCE = 130;
static const int GRIPPER_THRESHOLD_DISTANCE = 130;

static const double ANGLE_PITCH_INC = M_PI / 16;
static const double ANGLE_PITCH_MAX = M_PI / 16; //2

static const double ANGLE_YAW_INC = M_PI / 8;
static const double ANGLE_YAW_MAX = M_PI / 8; //2

static const double HEIGHT_SKIP_TOP = 0.0;
static const double HEIGHT_SKIP_BOTTOM = 0.04;
static const double HEIGHT_INC = 0.01;

static const double TRANS_MAX = 0.0;
static const double TRANS_INC = 0.001;

static const double PLACE_X_INC = 0.015;
static const double PLACE_X_MAX = 0.08;
static const double PLACE_YZ_INC = 0.04;
static const double PLACE_Y_MAX = 0.10;
static const double PLACE_Z_MAX = 0.17;

static const double PLACE_AT_INC = 0.003;
static const double PLACE_AT_MAX = 0.01;
static const double PLACE_AT_ANGLE_PITCH_INC = M_PI / 16;
static const double PLACE_AT_ANGLE_PITCH_MAX = M_PI / 16;
static const double PLACE_AT_ANGLE_YAW_INC = M_PI / 16;
static const double PLACE_AT_ANGLE_YAW_MAX = M_PI / 16;

ParamReader::ParamReader():private_nh_("~") {

    private_nh_.param("model", model, MODEL);

    private_nh_.param("groupArm", groupArm, GROUP_ARM);
    private_nh_.param("groupEe", groupEef, GROUP_END_EFFECTOR);

    private_nh_.param("endEffector", endEffector, END_EFFECTOR);

	private_nh_.param("frameArm", frameArm, FRAME_ARM);
	private_nh_.param("frameGripper", frameGripper, FRAME_GRIPPER);

	private_nh_.param("eefCmdScope", eefCmdScope, EEF_CMD_SCOPE);

    private_nh_.param("plannerId", plannerId, PLANNER_ID);
    private_nh_.param("planningTime", planningTime, PLANNING_TIME);

	private_nh_.param("eefPosClosed", eefPosClosed, EEF_POSITION_CLOSED);
	private_nh_.param("eefPosOpen", eefPosOpen, EEF_POSITION_OPEN);

	private_nh_.param("touchLinks", touchLinks, TOUCH_LINKS);

    private_nh_.param("graspRot", graspRot, GRASP_ROTATION);

	private_nh_.param("goalJointTolerance", goalJointTolerance, GOAL_JOINT_TOLERANCE);
    private_nh_.param("goalPositionTolerance", goalPositionTolerance, GOAL_POSITION_TOLERANCE);
    private_nh_.param("goalOrientationTolerance", goalOrientationTolerance, GOAL_ORIENTATION_TOLERANCE);

	private_nh_.param("force_threshold", gripperThresholdForce, GRIPPER_THRESHOLD_FORCE);
	private_nh_.param("distance_threshold", gripperThresholdDistance, GRIPPER_THRESHOLD_DISTANCE);

	private_nh_.param("approachDesiredDistance", approachDesiredDistance, APPROACH_DESIRED_DISTANCE);
	private_nh_.param("approachMinDistance", approachMinDistance, APPROACH_MIN_DISTANCE);

	private_nh_.param("liftUpDesiredDistance", liftUpDesiredDistance, LIFTUP_DESIRED_DISTANCE);
	private_nh_.param("liftUpMinDistance", liftUpMinDistance, LIFTUP_MIN_DISTANCE);

	private_nh_.param("retreatDesiredDistance", retreatDesiredDistance, RETREAT_DESIRED_DISTANCE);
	private_nh_.param("retreatMinDistance", retreatMinDistance, RETREAT_MIN_DISTANCE);

	private_nh_.param("graspThroughDistance", graspThroughDistance, GRASP_THROUGH_DISTANCE);

	private_nh_.param("anglePitchInc", anglePitchInc, ANGLE_PITCH_INC);
	private_nh_.param("anglePitchMax", anglePitchMax, ANGLE_PITCH_MAX);

	private_nh_.param("angleYawInc", angleYawInc, ANGLE_YAW_INC);
	private_nh_.param("angleYawMax", angleYawMax, ANGLE_YAW_MAX);

	private_nh_.param("transInc", transInc, TRANS_INC);
	private_nh_.param("transMax", transMax, TRANS_MAX);

	private_nh_.param("placeXInc", placeXInc, PLACE_X_INC);
	private_nh_.param("placeXMax", placeXMax, PLACE_X_MAX);
	private_nh_.param("placeYZInc", placeYZInc, PLACE_YZ_INC);
	private_nh_.param("placeYMax", placeYMax, PLACE_Y_MAX);
	private_nh_.param("placeZMax", placeZMax, PLACE_Z_MAX);

	private_nh_.param("placeAtInc", placeAtInc, PLACE_AT_INC);
	private_nh_.param("placeAtMax", placeAtMax, PLACE_AT_MAX);
	private_nh_.param("placeAtAnglePitchInc", placeAtAnglePitchInc, PLACE_AT_ANGLE_PITCH_INC);
	private_nh_.param("placeAtAnglePitchMax", placeAtAnglePitchMax, PLACE_AT_ANGLE_PITCH_MAX);
	private_nh_.param("placeAtAngleYawInc", placeAtAngleYawInc, PLACE_AT_ANGLE_YAW_INC);
	private_nh_.param("placeAtAngleYawMax", placeAtAngleYawMax, PLACE_AT_ANGLE_YAW_MAX);

	private_nh_.param("heightSkipTop", heightSkipTop, HEIGHT_SKIP_TOP);
	private_nh_.param("heightSkipBottom", heightSkipBottom, HEIGHT_SKIP_BOTTOM);
	private_nh_.param("heightInc", heightInc, HEIGHT_INC);

}

ParamReader::~ParamReader() {
}

ParamReader& ParamReader::getParamReader() {
	return ParamReader::getInstanceBase();
}

ParamReader& ParamReader::getInstanceBase() {
	return rsc::patterns::Singleton<ParamReader>::getInstance();
}
