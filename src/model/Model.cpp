/*
 * Model.cpp
 *
 *  Created on: Nov 30, 2015
 *      Author: plueckin
 */

#include "Model.h"
#include <ros/ros.h>
#include <moveit/move_group_pick_place_capability/capability_names.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/PlaceGoal.h>
#include <moveit_msgs/Grasp.h>
#include <eigen3/Eigen/src/Core/PlainObjectBase.h>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>

#include "../grasping/CentroidGrasping.h"
#include "../interface/AGNIInterface.h"

using namespace std;
using namespace moveit;
using namespace actionlib;
using namespace moveit::planning_interface;

Model::Model() {

    if (ParamReader::getParamReader().graspGen == CENTROID_GRASP_NAME)
        graspGenerator = CentroidGrasping::Ptr(new CentroidGrasping());
    if (ParamReader::getParamReader().graspGen == AGNI_GRASP_NAME)
        graspGenerator = AGNIInterface::Ptr(new AGNIInterface());

    lastHeightAboveTable = 0.0;
    graspedObjectID = "";

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
    ROS_DEBUG("Invoked getJointNames");

    return groupArm->getJoints();
}

int Model::getNumJoints() const {
    ROS_DEBUG("Invoked getNumJoints");

    return groupArm->getJoints().size();
}

vector<string> Model::getEefJointNames() const {
    ROS_DEBUG("Invoked getJointNames");

    return groupEe->getActiveJoints();
}

map<string, double> Model::getJointAngles() const {
    ROS_DEBUG("Invoked getJointAngles");

    map<string, double> jointAngles;
    vector<string> jointNames = groupArm->getJoints();
    vector<double> jointValues = groupArm->getCurrentJointValues();
    for (int i = 0; i < jointNames.size(); i++) {
        jointAngles[jointNames[i]] = jointValues[i];
    }
    return jointAngles;
}

void Model::setJointAngle(const string &joint, double angle) {
    ROS_INFO("### Invoked setJointAngle ###");

    groupArm->clearPoseTargets();
    groupArm->setStartStateToCurrentState();
    groupArm->setJointValueTarget(joint, angle);
    groupArm->move();
}

void Model::setJointAngles(const map<string, double> &angles) {
    ROS_INFO("### Invoked setJointAngles ###");

    groupArm->clearPoseTargets();
    groupArm->setStartStateToCurrentState();
    for (map<string, double>::const_iterator i = angles.begin(); i != angles.end(); ++i) {
        groupArm->setJointValueTarget(i->first, i->second);
    }
    groupArm->move();
}

void Model::setJointAngles(const vector<double> &angles) {
    ROS_INFO("### Invoked setJointAngles ###");

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
    ROS_INFO("### Invoked moveTo (pose) ###");

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
    ROS_DEBUG("Invoked getRememberedPoses");
    string planningGroup = groupArm->getName();
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
    ROS_DEBUG("poses %d, names %d", (int) poses.size(), (int) names.size());
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
    ROS_DEBUG("Invoked getEefPose");
    EefPose pose;
    geometry_msgs::PoseStamped ps = groupArm->getCurrentPose();

    ROS_INFO_STREAM("getEefPose() 1: " << ps.pose.position.x << "," << ps.pose.position.y << "," << ps.pose.position.z << "," << ps.header.frame_id);

    tfTransformer.transform(ps, ps, ParamReader::getParamReader().frameArm);

    ROS_INFO_STREAM("getEefPose() 2: " << ps.pose.position.x << "," << ps.pose.position.y << "," << ps.pose.position.z << "," << ps.header.frame_id);

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
    ROS_INFO("Invoked findObjects");

    vector<grasping_msgs::Object> grasps;
    grasps = graspGenerator->find_objects(false);
    return grasps.size();
}

GraspReturnType Model::graspObject(const string &obj, const string &surface, const vector<moveit_msgs::Grasp> &grasps, bool simulate, const string &startPose) {

    //ROS_DEBUG("Trying to pick object %s on %s (height: %.3f).", obj, surface, tableHeightArmCoords);

    GraspReturnType grt;

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

    moveit_msgs::PickupGoal goal = buildPickupGoal(obj, surface, grasps, simulate);
    pickActionClient->sendGoal(goal);
    if (!pickActionClient->waitForResult()) {
        ROS_INFO_STREAM("Pickup action returned early");
    }

    ROS_INFO("###########################");
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

            rosTools.getCollisionObjectByName(surface, colSurface);
            lastHeightAboveTable = colSurface.primitive_poses[0].position.z - resultGrasp.grasp_pose.pose.position.z;
            graspedObjectID = obj;
            grt.result = GraspReturnType::SUCCESS;
            ROS_INFO("  Grasped object at %.3f, %.3f, %.3f (frame: %s).", grt.point.xMeter, grt.point.yMeter, grt.point.zMeter, grt.point.frame.c_str());
        } else {
            ROS_WARN_STREAM("  Grasped no object: nothing in gripper!");
            grt.result = GraspReturnType::FAIL;
        }
    } else if (pickActionClient->getState() == SimpleClientGoalState::ABORTED) {
        ROS_WARN_STREAM("  Pick Action ABORTED (" << pickActionClient->getResult()->error_code.val << "): " << pickActionClient->getState().getText());
        rosTools.clear_octomap();
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
    }


    if (grt.result != GraspReturnType::SUCCESS) {
        rosTools.remove_collision_object(obj);
        rosTools.detach_collision_object();
    }

    return grt;
}

GraspReturnType Model::placeObject(const std::string &surface, std::vector<moveit_msgs::PlaceLocation> locations, bool simulate, const std::string &startPose) {

    GraspReturnType grt;

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

    rosTools.clear_octomap();

    moveit_msgs::PlaceGoal goal = buildPlaceGoal(surface, locations, simulate);


    for (int i = 0; i < 3; i++) {
        placeActionClient->sendGoal(goal);

        if (!placeActionClient->waitForResult()) {
            ROS_INFO_STREAM("Place action returned early");
        }
        SimpleClientGoalState resultState = placeActionClient->getState();

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
    }
    return grt;
}

//TODO: params

moveit_msgs::PlaceGoal Model::buildPlaceGoal(const string &surface,
        const vector<moveit_msgs::PlaceLocation>& locations, bool simulate) {
    moveit_msgs::PlaceGoal goal;
    goal.attached_object_name = graspedObjectID;
    goal.allowed_touch_objects.push_back(graspedObjectID);
    goal.group_name = groupArm->getName();
    goal.allowed_planning_time = groupArm->getPlanningTime();
    goal.support_surface_name = surface;
    goal.planner_id = ParamReader::getParamReader().plannerId;
    goal.place_eef = true;
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

    for (const string &i : touchlinks)
        goal.attached_object_touch_links.push_back(i);
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

    ROS_INFO("Publishing default object!");
    //moveit_msgs::CollisionObject attachDefaultObj;

    //attachDefaultObj.header.frame_id = params.frameGripper;
    //attachDefaultObj.id = rosTools.getDefaultObjectName();

    ObjectShape shape;
    shape.heightMeter = 0.05;
    shape.widthMeter = 0.05;
    shape.depthMeter = 0.05;
    shape.center.frame = params.frameGripper;
    rosTools.publish_collision_object(rosTools.getDefaultObjectName(), shape, 0.5);

    groupEe->attachObject(rosTools.getDefaultObjectName(), params.frameGripper, touchlinks);

    ros::spinOnce();
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
    grasp.pre_grasp_approach.direction.header.frame_id = params.frameGripper;
    grasp.pre_grasp_approach.min_distance = params.approachMinDistance;
    grasp.pre_grasp_approach.desired_distance = params.approachDesiredDistance;

    // direction: lift up
    grasp.post_grasp_retreat.direction.vector.z = 1.0;
    grasp.post_grasp_retreat.direction.vector.x = -1.0;
    grasp.post_grasp_retreat.direction.header.stamp = ros::Time::now();
    grasp.post_grasp_retreat.direction.header.frame_id = params.frameArm; //base_link!
    grasp.post_grasp_retreat.min_distance = params.liftUpMinDistance;
    grasp.post_grasp_retreat.desired_distance = params.liftUpDesiredDistance;

    // open on approach and close when reached
    if (grasp.pre_grasp_posture.points.size() == 0) {
        grasp.pre_grasp_posture = generate_open_eef_msg();
    }
    if (grasp.grasp_posture.points.size() == 0) {
        grasp.grasp_posture = generate_close_eef_msg();
    }

}

void Model::fillPlace(moveit_msgs::PlaceLocation& pl) {

    ParamReader& params = ParamReader::getParamReader();

    // place down in base_link
    pl.pre_place_approach.direction.vector.z = -1.0;
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
    pl.post_place_retreat.direction.header.frame_id = params.frameGripper;
    pl.post_place_retreat.min_distance = params.liftUpMinDistance;
    pl.post_place_retreat.desired_distance = params.liftUpDesiredDistance;

    pl.post_place_posture = generate_open_eef_msg();
}

std::vector<moveit_msgs::PlaceLocation> Model::generate_place_locations(
        const string &surface) {

    ROS_INFO_STREAM(
            "generate_place_locations(): lastGraspPose:" << lastGraspPose << " - lastHeightAboveTable: " << lastHeightAboveTable);
    geometry_msgs::Quaternion orientMsg = lastGraspPose.pose.orientation;
    /**  tf::Quaternion orientation = tf::Quaternion();
      if (orientation.w() == 0.0f && orientation.x() == 0.0f
              && orientation.y() == 0.0f && orientation.z() == 0.0f) {
          ROS_INFO_STREAM("Use default orientation");
          orientation = tf::createQuaternionFromRPY(-M_PI_2, 0, 0);
      }
      if (lastHeightAboveTable == 0.0) {
          lastHeightAboveTable = -0.15;
          ROS_INFO_STREAM("Use default lastHeightAboveTable: " << lastHeightAboveTable);
      }**/

    std::vector<moveit_msgs::PlaceLocation> pls;

    moveit_msgs::CollisionObject colSurface;
    bool success = rosTools.getCollisionObjectByName(surface, colSurface);

    if (!success) {
        ROS_ERROR_STREAM("No Plane with Name: " << surface);
        return pls;
    }
    colSurface.primitive_poses[0].position.x;

    float rounds = 5;
    int place_rot = 8;
    int rotation = 7;

    float surfaceSizeX = colSurface.primitives[0].dimensions[0];
    float surfaceSizeY = colSurface.primitives[0].dimensions[1];
    float surfaceSizeZ = colSurface.primitives[0].dimensions[2];
    float surfaceCenterX = colSurface.primitive_poses[0].position.x;
    float surfaceCenterY = colSurface.primitive_poses[0].position.y;
    float surfaceCenterZ = colSurface.primitive_poses[0].position.z;

    for (int x = 0; x < rounds; x++) {
        for (int y = 0; y < place_rot; y++) {
            moveit_msgs::PlaceLocation pl;
            pl.place_pose.header.frame_id = colSurface.header.frame_id;
            float param = y * 2 * M_PI / place_rot;
            pl.place_pose.pose.position.x = surfaceCenterX + surfaceSizeX / 2 * (x / rounds) * sin(param);
            pl.place_pose.pose.position.y = surfaceCenterY + surfaceSizeY / 2 * (x / rounds) * cos(param);
            pl.place_pose.pose.position.z = surfaceCenterZ - lastHeightAboveTable + surfaceSizeZ / 2;
            Eigen::Quaternionf quat(orientMsg.w, orientMsg.x, orientMsg.y, orientMsg.z);

            for (int r = 0; r < rotation; r++) {
                float rot = 2 * M_PI * r / rotation;
                Eigen::Quaternionf rotate(Eigen::AngleAxisf(rot, Eigen::Vector3f::UnitZ()));
                Eigen::Matrix3f result = (rotate.toRotationMatrix() * quat.toRotationMatrix());
                Eigen::Quaternionf quatresult(result);
                quatresult.normalize();
                pl.place_pose.pose.orientation.x = quatresult.x();
                pl.place_pose.pose.orientation.y = quatresult.y();
                pl.place_pose.pose.orientation.z = quatresult.z();
                pl.place_pose.pose.orientation.w = quatresult.w();
                ROS_DEBUG_STREAM("x: " << pl.place_pose.pose.orientation.x << " y: " << pl.place_pose.pose.orientation.y
                        << " z: " << pl.place_pose.pose.orientation.z << " w: " << pl.place_pose.pose.orientation.y);
                fillPlace(pl);
                pls.push_back(pl);
            }
        }
    }
    return pls;
}
