/*
 * Model.cpp
 *
 *  Created on: Nov 30, 2015
 *      Author: plueckin
 */

#include "Model.h"

#include <regex>
#include <ros/ros.h>
#include <moveit/move_group_pick_place_capability/capability_names.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/PlaceGoal.h>
#include <moveit_msgs/Grasp.h>
#include <eigen3/Eigen/src/Core/PlainObjectBase.h>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>

#include <thread>
#include <chrono>

#include "../grasping/CentroidGrasping.h"
#include "../interface/AGNIInterface.h"

using namespace std;
using namespace moveit;
using namespace actionlib;
using namespace moveit::planning_interface;

static const string NAME = "RosTools";

#define DEFAULT_SURFACE "surface0"

Model::Model() {

    if (ParamReader::getParamReader().graspGen == CENTROID_GRASP_NAME)
        graspGenerator = CentroidGrasping::Ptr(new CentroidGrasping());
    if (ParamReader::getParamReader().graspGen == AGNI_GRASP_NAME)
        graspGenerator = AGNIInterface::Ptr(new AGNIInterface());

    lastHeightAboveTable = 0.0;
    graspedObjectID = "";
    filterTypes = "all"; // all, top, side

    for (const string &i : ParamReader::getParamReader().touchLinks)
        touchlinks.push_back(i);

    groupArm = new moveit::planning_interface::MoveGroup(
            ParamReader::getParamReader().groupArm);
    groupArm->setPlanningTime(ParamReader::getParamReader().planningTime);
    groupArm->startStateMonitor();

    groupArm->setPlannerId(ParamReader::getParamReader().plannerId);
    groupArm->setPoseReferenceFrame(ParamReader::getParamReader().frameArm);

    groupArm->setGoalJointTolerance(ParamReader::getParamReader().goalJointTolerance);
    groupArm->setGoalPositionTolerance(ParamReader::getParamReader().goalPositionTolerance);
    groupArm->setGoalOrientationTolerance(ParamReader::getParamReader().goalOrientationTolerance);

    groupEe = new moveit::planning_interface::MoveGroup(
            ParamReader::getParamReader().groupEef);
    groupEe->startStateMonitor();

    for (vector<string>::const_iterator it = groupEe->getActiveJoints().begin();
            it != groupEe->getActiveJoints().end(); ++it) {
        printf("active joint '%s'\n", it->c_str());
    }

    pickActionClient.reset(
            new actionlib::SimpleActionClient<moveit_msgs::PickupAction>(nh,
            move_group::PICKUP_ACTION, false));
    placeActionClient.reset(
            new actionlib::SimpleActionClient<moveit_msgs::PlaceAction>(nh,
            move_group::PLACE_ACTION, false));

    rosTools.waitForAction(pickActionClient, ros::Duration(0, 0),
            move_group::PICKUP_ACTION);
    rosTools.waitForAction(placeActionClient, ros::Duration(0, 0),
            move_group::PLACE_ACTION);
}

void Model::stop() const {
    groupArm->stop();
}

vector<string> Model::getJointNames() const {
    ROS_INFO_NAMED(NAME, "### Invoked getJointNames ###");

    return groupArm->getJoints();
}

int Model::getNumJoints() const {
    ROS_INFO_NAMED(NAME, "### Invoked getNumJoints ###");

    return groupArm->getJoints().size();
}

vector<string> Model::getEefJointNames() const {
    ROS_INFO_NAMED(NAME, "### Invoked getJointNames ###");

    return groupEe->getActiveJoints();
}

map<string, double> Model::getJointAngles() const {
    ROS_DEBUG_NAMED(NAME, "### Invoked getJointAngles ###");

    map<string, double> jointAngles;
    vector<string> jointNames = groupArm->getJoints();
    vector<double> jointValues = groupArm->getCurrentJointValues();
    for (int i = 0; i < jointNames.size(); i++) {
        jointAngles[jointNames[i]] = jointValues[i];
    }
    return jointAngles;
}

void Model::setJointAngle(const string &joint, double angle) {
    ROS_DEBUG_NAMED(NAME, "### Invoked setJointAngle ###");

    groupArm->clearPoseTargets();
    groupArm->setStartStateToCurrentState();
    groupArm->setJointValueTarget(joint, angle);
    groupArm->move();
}

void Model::setFilterType(std::string type) {
    ROS_INFO_STREAM_NAMED(NAME, "Set FilterType to " << type);
    
    filterTypes = type;
}

void Model::setJointAngles(const map<string, double> &angles) {
    ROS_INFO_NAMED(NAME, "### Invoked setJointAngles ###");

    groupArm->clearPoseTargets();
    groupArm->setStartStateToCurrentState();
    for (map<string, double>::const_iterator i = angles.begin(); i != angles.end(); ++i) {
        groupArm->setJointValueTarget(i->first, i->second);
    }
    groupArm->move();
}

void Model::setJointAngles(const vector<double> &angles) {
    ROS_INFO_NAMED(NAME, "### Invoked setJointAngles ###");

    vector<string> joints = groupArm->getJoints();
    if (angles.size() < 0 || angles.size() > joints.size()) {
        ROS_ERROR("Requested number of joints wrong! (%i)", (int) angles.size());
        return;
    }

    groupArm->clearPoseTargets();
    groupArm->setStartStateToCurrentState();
    for (int i = 0; i < angles.size(); i++) {
        groupArm->setJointValueTarget(joints[i], angles[i]);
    }
    groupArm->move();
}

MoveResult Model::moveTo(const EefPose& pose, bool linear, bool orientation) {
    ROS_INFO_NAMED(NAME, "### Invoked moveTo (pose) ###");

    groupArm->clearPoseTargets();
    groupArm->setStartStateToCurrentState();

    if (orientation) {
        geometry_msgs::Pose p;
        p.position.x = pose.translation.xMeter;
        p.position.y = pose.translation.yMeter;
        p.position.z = pose.translation.zMeter;
        p.orientation.w = pose.rotation.qw;
        p.orientation.x = pose.rotation.qx;
        p.orientation.y = pose.rotation.qy;
        p.orientation.z = pose.rotation.qz;
        groupArm->setPoseTarget(p);
    } else {
        groupArm->setPositionTarget(pose.translation.xMeter, pose.translation.yMeter,
                pose.translation.zMeter);
    }

    return rosTools.moveResultFromMoveit(groupArm->move());
}

ArmPoses Model::getRememberedPoses() const {
    ROS_INFO_NAMED(NAME, "### Invoked getRememberedPoses ###");
    string planningGroup = groupArm->getName();
    ROS_DEBUG_STREAM("PLANNING GROUP NAME " << planningGroup);
    const robot_model::JointModelGroup* jmg =
            groupArm->getCurrentState()->getRobotModel()->getJointModelGroup(planningGroup);
    vector<string> names = jmg->getDefaultStateNames();
    ArmPoses poses;
    for (vector<string>::iterator it = names.begin(); it != names.end(); it++) {
        string name = *it;
        map<string, double> angles;
        jmg->getVariableDefaultPositions(name, angles);
        poses[name] = angles;
    }
    ROS_DEBUG_NAMED(NAME, "poses %d, names %d", (int) poses.size(), (int) names.size());
    return poses;
}

ArmPose Model::getRememberedPose(const std::string &name) const {
    const robot_model::JointModelGroup* jmg =
            groupArm->getCurrentState()->getRobotModel()->getJointModelGroup(groupArm->getName());
    map<string, double> angles;
    jmg->getVariableDefaultPositions(name, angles);
    return angles;
}

EefPose Model::getEefPose() const {
    ROS_INFO_NAMED(NAME, "### Invoked getEefPose ###");
    EefPose pose;
    geometry_msgs::PoseStamped ps = groupArm->getCurrentPose();

    ROS_DEBUG_STREAM("getEefPose() before transformation: " << ps.pose.position.x << "," << ps.pose.position.y << "," << ps.pose.position.z << "," << ps.header.frame_id);

    tfTransformer.transform(ps, ps, ParamReader::getParamReader().frameArm);

    ROS_DEBUG_STREAM("getEefPose() after transformation: " << ps.pose.position.x << "," << ps.pose.position.y << "," << ps.pose.position.z << "," << ps.header.frame_id);

    pose.translation.xMeter = ps.pose.position.x;
    pose.translation.yMeter = ps.pose.position.y;
    pose.translation.zMeter = ps.pose.position.z;
    pose.rotation.qw = ps.pose.orientation.w;
    pose.rotation.qx = ps.pose.orientation.x;
    pose.rotation.qy = ps.pose.orientation.y;
    pose.rotation.qz = ps.pose.orientation.z;
    pose.frame = ParamReader::getParamReader().frameArm;
    return pose;
}

int Model::findObjects() {
    ROS_INFO_NAMED(NAME, "### Invoked findObjects ###");

    vector<grasping_msgs::Object> grasps;
    grasps = graspGenerator->find_objects(false);
    return grasps.size();
}

vector<moveit_msgs::Grasp> Model::filtergrasps(const vector<moveit_msgs::Grasp> &grasps) {
    ROS_DEBUG_STREAM("Filter type is:" << filterTypes);

    vector<moveit_msgs::Grasp> filteredgrasps;
    for (moveit_msgs::Grasp i : grasps) {
        ROS_DEBUG_STREAM("Grasp.id: " << i.id);
        if (filterTypes == "side") {
            if (i.id.find("Side") != std::string::npos) {
                ROS_DEBUG_STREAM("Side: " << i.id);
                filteredgrasps.push_back(i);
            }
        }
        if (filterTypes == "top") {
            if (i.id.find("Top") != std::string::npos) {
                ROS_DEBUG_STREAM("Top: " << i.id);
                filteredgrasps.push_back(i);
            }
        }
        if (filterTypes == "all") {
            ROS_DEBUG_STREAM("All: " << i.id);
            filteredgrasps.push_back(i);
        }
    }
    return filteredgrasps;
}

GraspReturnType Model::graspObject(const string &obj, const string &surface, const vector<moveit_msgs::Grasp> &grasps, bool simulate, const string &startPose) {
    ROS_DEBUG_STREAM_NAMED(NAME, "Trying to pick object " << obj << " on " << surface);

    EefPose eefStart = getEefPose();
    GraspReturnType grt;

    vector<moveit_msgs::Grasp> filteredgrasps = filtergrasps(grasps);
    if (!pickActionClient) {
        ROS_ERROR_STREAM("Pick action client not found");
        grt.result = GraspReturnType::FAIL;
        return grt;
    }
    if (!pickActionClient->isServerConnected()) {
        ROS_ERROR_STREAM("Pick action server not connected");
        grt.result = GraspReturnType::FAIL;
        return grt;
    }

    if (!rosTools.getCollisionObjectByName(obj, lastGraspTried)) {
        ROS_ERROR_STREAM("Did not get collision object " << obj << " to grasp");
        lastGraspTried.id = "";
    }
    
    
    
    moveit_msgs::CollisionObject o;
    moveit_msgs::PickupGoal goal;
    if (rosTools.getCollisionObjectByName(DEFAULT_SURFACE, o)) { //surface = "surface0"; //FIXME: BAD HACK.
        goal = buildPickupGoal(obj, DEFAULT_SURFACE, filteredgrasps, simulate);
    } else {   
        ROS_ERROR_STREAM("surface " << surface << " not found");
        grt.result = GraspReturnType::FAIL;
        return grt;
    }

    pickActionClient->sendGoal(goal);
    if (!pickActionClient->waitForResult()) {
        ROS_INFO_STREAM("Pickup action returned early");
    }

    ROS_INFO("###########################");
    ros::Duration(0.25).sleep();
    ROS_DEBUG_STREAM("pickActionClient State " << pickActionClient->getState().text_);
    while(pickActionClient->getState() == SimpleClientGoalState::ACTIVE) {
        ROS_INFO("Grasp Active. Looping.");
        ros::Duration(0.25).sleep();
    }
    if (pickActionClient->getState() == SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Pick Action succeeded. (err_code: %d, state: %s)", pickActionClient->getResult()->error_code.val, pickActionClient->getState().toString().c_str());

        if (simulate || isSomethingInGripper()) {
            moveit_msgs::Grasp resultGrasp = pickActionClient->getResult()->grasp;
            std::vector<moveit_msgs::Grasp> executed_grasps;
            executed_grasps.push_back(resultGrasp);
            rosTools.display_grasps(executed_grasps);
            grt.point.xMeter = resultGrasp.grasp_pose.pose.position.x;
            grt.point.yMeter = resultGrasp.grasp_pose.pose.position.y;
            grt.point.zMeter = resultGrasp.grasp_pose.pose.position.z;
            grt.point.frame = resultGrasp.grasp_pose.header.frame_id;
            lastGraspPose = resultGrasp.grasp_pose;
            moveit_msgs::CollisionObject colSurface;
            graspedObjectID = obj;
            if (rosTools.getCollisionObjectByName(surface, colSurface)) {
                lastHeightAboveTable = abs(colSurface.primitive_poses[0].position.z - resultGrasp.grasp_pose.pose.position.z);
            } else {
                //todo defautl surface hack
                ROS_WARN_STREAM("(surface) for grasping with Name: " << surface << " try default: " << DEFAULT_SURFACE);
                if (rosTools.getCollisionObjectByName(DEFAULT_SURFACE, colSurface)) {
                    lastHeightAboveTable = abs(colSurface.primitive_poses[0].position.z - resultGrasp.grasp_pose.pose.position.z);
                }
            }
            grt.result = GraspReturnType::SUCCESS;
            ROS_INFO("  Grasped object at %.3f, %.3f, %.3f (frame: %s).", grt.point.xMeter, grt.point.yMeter, grt.point.zMeter, grt.point.frame.c_str());
        } else {
            ROS_WARN_STREAM("  Grasped no object: nothing in gripper!");
            //rosTools.detach_collision_object();
            //rosTools.remove_collision_object(obj);
            ros::spinOnce();
            ROS_INFO_STREAM("moving to start pose");
            moveTo(eefStart, false, false);
            grt.result = GraspReturnType::FAIL;
        }
    } else if (pickActionClient->getState() == SimpleClientGoalState::ABORTED) {
        ROS_WARN_STREAM("  Pick Action ABORTED (" << pickActionClient->getResult()->error_code.val << "): " << pickActionClient->getState().getText());
        rosTools.clear_octomap();
        ROS_INFO_STREAM("moving to start pose");
        moveTo(eefStart, false, false);
        grt.result = GraspReturnType::FAIL;
        if (pickActionClient->getResult()->error_code.val == MoveItErrorCode::PLANNING_FAILED) {
            grt.result = GraspReturnType::FAIL;
        }
    } else if (pickActionClient->getState() == SimpleClientGoalState::PENDING) {
        ROS_WARN_STREAM(
                "  Pick Action PENDING (" << pickActionClient->getResult()->error_code.val << "): " << pickActionClient->getState().getText());
        rosTools.clear_octomap();
        if (pickActionClient->getResult()->error_code.val == MoveItErrorCode::SUCCESS) {
            grt.result = GraspReturnType::ROBOT_CRASHED;
        }
    } else {
        ROS_WARN_STREAM(
                "  Pick Action failed: " << pickActionClient->getState().toString() << " (" << pickActionClient->getResult()->error_code.val << "): " << pickActionClient->getState().getText());
        grt.result = rosTools.graspResultFromMoveit(pickActionClient->getResult()->error_code);
    }
    ROS_INFO("###########################");

    if (grt.result == GraspReturnType::NO_RESULT) {
        grt.result = rosTools.graspResultFromMoveit(pickActionClient->getResult()->error_code);
        moveTo(eefStart, false, false);
    }


    if (grt.result != GraspReturnType::SUCCESS) {
        rosTools.detach_collision_object();
        moveTo(eefStart, false, false);
        //rosTools.remove_collision_object(obj);
    }

    ROS_INFO("GRASP RETURNED");
    return grt;
}

GraspReturnType Model::placeObject(const string &obj, const std::string &surface, std::vector<moveit_msgs::PlaceLocation> locations, bool simulate, const std::string &startPose) {
    ROS_INFO_STREAM_NAMED(NAME, "Trying to place object " << obj << " on " << surface);
    
    GraspReturnType grt; //TODO: rename PlaceReturnType

    EefPose eefStart = getEefPose();

    if (!placeActionClient) {
        ROS_ERROR_STREAM("Place action client not found");
        grt.result = GraspReturnType::FAIL;
        return grt;
    }
    if (!placeActionClient->isServerConnected()) {
        ROS_ERROR_STREAM("Place action server not connected");
        grt.result = GraspReturnType::FAIL;
        return grt;
    }

    rosTools.clear_octomap();

    moveit_msgs::CollisionObject o;
    moveit_msgs::PlaceGoal goal;

    for (int i = 0; i < 3; i++) {

        if (rosTools.getCollisionObjectByName(surface, o)) {
           
            goal = buildPlaceGoal(obj, surface, locations, simulate);
        } else {
            ROS_ERROR_STREAM("surface " << surface << " not found");   
            grt.result = GraspReturnType::FAIL;
            return grt;
        }
        
        ROS_ERROR_STREAM("Trying to place " << obj << " on " << surface);
        
        placeActionClient->sendGoal(goal);
        SimpleClientGoalState resultState = placeActionClient->getState();

        if (!placeActionClient->waitForResult()) {
            ROS_INFO_STREAM("Place action returned early");
        }

        bool isPending = resultState == SimpleClientGoalState::PENDING;
        bool isAborted = resultState == SimpleClientGoalState::ABORTED;
        bool isSuccess = resultState == SimpleClientGoalState::SUCCEEDED;

        bool errCodeInvalidGroup = placeActionClient->getResult()->error_code.val == MoveItErrorCode::INVALID_GROUP_NAME;
        bool errCodeSuccess = placeActionClient->getResult()->error_code.val == MoveItErrorCode::SUCCESS;
        bool errCodePlanInvalidated = placeActionClient->getResult()->error_code.val == MoveItErrorCode::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE;

        ROS_INFO("###########################");
        if (isSuccess) {
            ROS_INFO("Place Action succeeded.");
            moveit_msgs::PlaceLocation placeLoc = placeActionClient->getResult()->place_location;
            grt.point.xMeter = placeLoc.place_pose.pose.position.x;
            grt.point.yMeter = placeLoc.place_pose.pose.position.y;
            grt.point.zMeter = placeLoc.place_pose.pose.position.z;
            grt.point.frame = placeLoc.place_pose.header.frame_id;
            grt.result = GraspReturnType::SUCCESS;
            ROS_INFO("###########################");
            break;
        } else if (isPending && errCodeInvalidGroup) {
            ROS_WARN("  PENDING: attached object not preset! Try to fix by attaching default object.");
            attachDefaultObject();
            ROS_INFO("###########################");
            continue;
        } else if (isAborted || (isPending && (errCodeSuccess || errCodePlanInvalidated))) {
            ROS_WARN("  ABORTED: %s (%d): %s", resultState.toString().c_str(), placeActionClient->getResult()->error_code.val, resultState.getText().c_str());
            grt.result = GraspReturnType::ROBOT_CRASHED;
            rosTools.clear_octomap();
            break;
        } else {
            ROS_WARN("  Fail: %s (%d): %s", resultState.toString().c_str(), placeActionClient->getResult()->error_code.val, resultState.getText().c_str());
            grt.result = rosTools.graspResultFromMoveit(placeActionClient->getResult()->error_code);
            ROS_INFO("###########################");
            break;
        }
        break;
    }

    if (!isSomethingInGripper()) {
        rosTools.detach_collision_object();
        moveTo(eefStart, false, false);
    }

    return grt;
}

//TODO: params

moveit_msgs::PlaceGoal Model::buildPlaceGoal(const string &obj, 
        const string &surface,
        const vector<moveit_msgs::PlaceLocation>& locations, bool simulate) {
    moveit_msgs::PlaceGoal goal;
    goal.attached_object_name = obj;
    goal.allowed_touch_objects.push_back(obj);
    goal.group_name = groupArm->getName();
    goal.allowed_planning_time = groupArm->getPlanningTime();
    goal.support_surface_name = surface;
    goal.planner_id = ParamReader::getParamReader().plannerId;
    goal.place_eef = false;
    goal.place_locations = locations;
    goal.planning_options.plan_only = simulate;
    goal.planning_options.look_around = false;
    goal.planning_options.replan = false;
    goal.planning_options.replan_delay = 2.0;
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
    return goal;
}

//TODO: params

moveit_msgs::PickupGoal Model::buildPickupGoal(const string &obj,
        const string &supportSurface, const vector<moveit_msgs::Grasp> &grasps, bool simulate) {

    moveit_msgs::PickupGoal goal;
    goal.possible_grasps = grasps;
    goal.target_name = obj;
    goal.support_surface_name = supportSurface;

    for (const string &i : touchlinks) {
        goal.attached_object_touch_links.push_back(i);
    }
    
    ROS_INFO_STREAM("added " << goal.attached_object_touch_links.size() << " touch links");
     
    goal.group_name = groupArm->getName();
    goal.end_effector = ParamReader::getParamReader().endEffector;
    goal.allowed_planning_time = groupArm->getPlanningTime();
    goal.planner_id = ParamReader::getParamReader().plannerId;
    goal.planning_options.plan_only = simulate;
    goal.planning_options.look_around = false;
    goal.planning_options.replan = false;
    goal.planning_options.replan_delay = 2.0;
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
    return goal;
}

//TODO: params

void Model::attachDefaultObject() {
    ParamReader& params = ParamReader::getParamReader();
    moveit_msgs::AttachedCollisionObject attached_object;


    if (lastGraspTried.id != "") {
        ROS_INFO_STREAM("attach default using last tried object");
        attached_object.object = lastGraspTried;

        lastHeightAboveTable = lastGraspTried.primitive_poses[0].position.z + 0.01;
        graspedObjectID = lastGraspTried.id;
    } else {
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.07;
        primitive.dimensions[1] = 0.07;
        primitive.dimensions[2] = 0.07;
        attached_object.object.primitives.push_back(primitive);

        geometry_msgs::Pose pose;
        pose.orientation.w = 1.0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.position.x = 0.05;
        pose.position.y = 0;
        pose.position.z = 0;
        attached_object.object.primitive_poses.push_back(pose);

        attached_object.object.id = rosTools.getDefaultObjectName();
        lastHeightAboveTable = 0.10;
        graspedObjectID = rosTools.getDefaultObjectName();
    }


    attached_object.object.operation = attached_object.object.ADD;
    attached_object.object.header.frame_id = ParamReader::getParamReader().frameGripper;

    attached_object.link_name = params.frameGripper;
    attached_object.touch_links = touchlinks;

    //groupEe->attachObject(rosTools.getDefaultObjectName(), params.frameGripper, touchlinks);

    rosTools.attach_collision_object(attached_object);

    ros::WallDuration sleep_time(1);
    sleep_time.sleep();
}

void Model::fillGrasp(moveit_msgs::Grasp& grasp) {

    ParamReader& params = ParamReader::getParamReader();
    if (params.robot == "tobi") {
        grasp.pre_grasp_approach.direction.vector.x = 1.0;
        grasp.pre_grasp_approach.direction.vector.y = 0.0;
        grasp.pre_grasp_approach.direction.vector.z = 0.0;
    } else if (params.robot == "meka") {
        grasp.pre_grasp_approach.direction.vector.x = 0.0;
        grasp.pre_grasp_approach.direction.vector.y = 0.0;
        grasp.pre_grasp_approach.direction.vector.z = 1.0;
    } else {
        ROS_ERROR("No known robot name, robot name should be tobi or meka");
    }

    grasp.pre_grasp_approach.direction.header.stamp = ros::Time::now();
    grasp.pre_grasp_approach.direction.header.frame_id = groupArm->getEndEffectorLink();
    grasp.pre_grasp_approach.min_distance = params.approachMinDistance;
    grasp.pre_grasp_approach.desired_distance = params.approachDesiredDistance;

    // direction: lift up
    grasp.post_grasp_retreat.direction.vector.z = 1.0;
    grasp.post_grasp_retreat.direction.vector.x = -1.0;
    grasp.post_grasp_retreat.direction.header.stamp = ros::Time::now();
    grasp.post_grasp_retreat.direction.header.frame_id = params.frameArm; // frameArm = base_link for meka
    grasp.post_grasp_retreat.min_distance = params.liftUpMinDistance;
    grasp.post_grasp_retreat.desired_distance = params.liftUpDesiredDistance;

    // open on approach and close when reached
    grasp.pre_grasp_posture = generate_open_eef_msg();
    grasp.grasp_posture = generate_close_eef_msg();
}

void Model::fillPlace(moveit_msgs::PlaceLocation& pl) {

    ParamReader& params = ParamReader::getParamReader();
   
    
    // place down in base_link ?
    if (params.robot == "tobi") {
        pl.pre_place_approach.direction.vector.x = 0.0;
        pl.pre_place_approach.direction.vector.y = 0.0;
        pl.pre_place_approach.direction.vector.z = 0.0;
    } else if (params.robot == "meka") {
        pl.pre_place_approach.direction.vector.x = 0.0;
        pl.pre_place_approach.direction.vector.y = 0.0;
        pl.pre_place_approach.direction.vector.z = -1.0;//meka->hand_tool_frame_left = z is in direction of the palm, x in negative direction of the thumb if sticking out.
    } else {
        ROS_ERROR("No known robot name, robot name should be tobi or meka");
    }
 
    pl.pre_place_approach.direction.header.stamp = ros::Time::now();
    pl.pre_place_approach.direction.header.frame_id = params.frameArm;
    pl.pre_place_approach.min_distance = params.approachMinDistance;
    pl.pre_place_approach.desired_distance = params.approachDesiredDistance;
    // retreat in negative hand direction

    if (params.robot == "tobi") {
        pl.post_place_retreat.direction.vector.x = -1.0;
        pl.post_place_retreat.direction.vector.y = 0.0;
        pl.post_place_retreat.direction.vector.z = 0.0;
    } else if (params.robot == "meka") {
        pl.post_place_retreat.direction.vector.x = 0.0;
        pl.post_place_retreat.direction.vector.y = 0.0;
        pl.post_place_retreat.direction.vector.z = -1.0;
    } else {
        ROS_ERROR("No known robot name, robot name should be tobi or meka");
    }
    pl.post_place_retreat.direction.header.stamp = ros::Time::now();
    pl.post_place_retreat.direction.header.frame_id = groupArm->getEndEffectorLink(); // retreat allways away from object and not up in world coordinates
    pl.post_place_retreat.min_distance = params.liftUpMinDistance;
    pl.post_place_retreat.desired_distance = params.liftUpDesiredDistance;

    pl.post_place_posture = generate_open_eef_msg();
}

std::vector<moveit_msgs::PlaceLocation> Model::generate_place_locations(
        const string &surface) {

    geometry_msgs::Quaternion orientMsg = lastGraspPose.pose.orientation;
    /**  tf::Quaternion orientation = tf::Quaternion();
      if (orientation.w() == 0.0f && orientation.x() == 0.0f
              && orientation.y() == 0.0f && orientation.z() == 0.0f) {
          ROS_INFO_STREAM("Use default orientation");
          orientation = tf::createQuaternionFromRPY(-M_PI_2, 0, 0);
      }**/
    if (lastHeightAboveTable == 0.0) {
        lastHeightAboveTable = 0.15;
        ROS_INFO_STREAM("Use default lastHeightAboveTable: " << lastHeightAboveTable);
    }

    std::vector<moveit_msgs::PlaceLocation> pls;

    moveit_msgs::CollisionObject colSurface; // "surface" is the object where the attached object should be placed on
    bool success = rosTools.getCollisionObjectByName(surface, colSurface);

    if (!success) {
        ROS_WARN_STREAM("No CollisionObject for Placing with Name: " << surface << " try default: " << DEFAULT_SURFACE);
        success = rosTools.getCollisionObjectByName(DEFAULT_SURFACE, colSurface);
        if (!success) {
            ROS_ERROR_STREAM("No CollisionObject for Placing with Name: " << DEFAULT_SURFACE);
            return pls;
        }
    }



    float surfaceSizeX = 0;
    float surfaceSizeY = 0;
    float surfaceSizeZ = 0;

    if (colSurface.primitives[0].type == shape_msgs::SolidPrimitive::CYLINDER) {
        ROS_INFO_STREAM("Place on a Cylinder");
        surfaceSizeX = colSurface.primitives[0].dimensions[1]*2; //radius * 2
        surfaceSizeY = colSurface.primitives[0].dimensions[1]*2; //radius * 2
        surfaceSizeZ = colSurface.primitives[0].dimensions[0]; //height
    } else if (colSurface.primitives[0].type == shape_msgs::SolidPrimitive::BOX) {
        ROS_INFO_STREAM("Place on a Box");
        surfaceSizeX = colSurface.primitives[0].dimensions[0]; //x
        surfaceSizeY = colSurface.primitives[0].dimensions[1]; //y
        surfaceSizeZ = colSurface.primitives[0].dimensions[2]; //z
    } else {
        ROS_INFO_STREAM("Place on default Parameter, no Sphere or Box, x,y,z 0.1 0.1 0.01");
        surfaceSizeX = 0.1;
        surfaceSizeY = 0.1;
        surfaceSizeZ = 0.01;
    }

    if (surfaceSizeX < 0)
        surfaceSizeX = 0;
    if (surfaceSizeY < 0)
        surfaceSizeY = 0;

    ROS_DEBUG_STREAM("Placesize X: " << surfaceSizeX << "PlaceSize Y: " << surfaceSizeY);

    float surfaceCenterX = colSurface.primitive_poses[0].position.x;
    float surfaceCenterY = colSurface.primitive_poses[0].position.y;
    float surfaceCenterZ = colSurface.primitive_poses[0].position.z;
    
    ROS_INFO_STREAM("Place object in frame " << lastGraspTried.header.frame_id);
    moveit_msgs::PlaceLocation pl;
    //create only one in the middle.
    pl.place_pose.header.frame_id = lastGraspTried.header.frame_id; //"base_link";//colSurface.header.frame_id; //baselink for clafu
    pl.place_pose.pose.position.x = surfaceCenterX;
    pl.place_pose.pose.position.y = surfaceCenterY;
    double placeHeight = surfaceCenterZ + (lastHeightAboveTable + 0.05) + surfaceSizeZ / 2;
    pl.place_pose.pose.position.z = placeHeight;

    pl.place_pose.pose.orientation = lastGraspPose.pose.orientation;
    
    //pl.place_pose.pose.orientation.w = 1; Johannes: Does this make sense on the meka? Flips place poses for tobi
    fillPlace(pl);
    pl.id = "orig";
    pls.push_back(pl);

    double xSteps = 10.0; //TODO: Parameters for theses values
    double ySteps = 10.0;
    double xStepSize = ((surfaceSizeX/2) * (1.0-(4.0/xSteps))) / xSteps;
    double yStepSize = ((surfaceSizeY/2) * (1.0-(4.0/ySteps))) / ySteps;
    ROS_INFO_STREAM("surfaceCenterX" << surfaceCenterX << "\n" << "surfaceCenterY" << surfaceCenterY << "\n");
    ROS_INFO_STREAM("surfaceSizeX" << surfaceSizeX << "\n" << "surfaceSizeY" << surfaceSizeY << "\n" << "xStepSize" << xStepSize << "\n" << "yStepSize" << yStepSize << "\n");
    int rotation = 10;
    for (int x = -10; x <= xSteps; x++) {
        for (int y = -10; y <= ySteps; y++) {
            //vary location of place
            moveit_msgs::PlaceLocation pl;
            pl.place_pose.header.frame_id = lastGraspTried.header.frame_id;
            pl.place_pose.pose.position.x = surfaceCenterX + xStepSize * x;
            pl.place_pose.pose.position.y = surfaceCenterY + yStepSize * y;
            pl.place_pose.pose.position.z = placeHeight;
            pl.place_pose.pose.orientation = lastGraspPose.pose.orientation;
            pl.id = "x" + std::to_string(xSteps) + "y" + std::to_string(ySteps);

            fillPlace(pl);
            pls.push_back(pl);

            //vary orientation of gripper only around base_link's z axis to keep object in same orientation
            Eigen::Affine3d placePose;
            tf2::fromMsg(pl.place_pose.pose, placePose);
            Eigen::Vector3d rotationCenter;
            tf2::fromMsg(pl.place_pose.pose.position, rotationCenter);
            for (int r = -10; r < rotation; r++) {
                moveit_msgs::PlaceLocation plr;
                plr.place_pose.header.frame_id = lastGraspTried.header.frame_id;
                double angle = (M_PI / 2) * r / rotation;
                Eigen::Affine3d A = Eigen::Translation3d(rotationCenter) * Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()) * Eigen::Translation3d(-rotationCenter);
                A = A * placePose;
                plr.place_pose.pose = Eigen::toMsg(A);
                plr.id = pl.id + "var" + std::to_string(r);
                pls.push_back(plr);
            }
        }
    }
    return pls;
}

std::string Model::getSurfaceByHeight(const float h) {
    moveit_msgs::CollisionObject o;
    //todo ugh
    std::regex e("surface[0-9]");
    if (rosTools.getCollisionObjectByHeight(h, o, e)) {
        return o.id;
    } else {
        return "";
    }

}

