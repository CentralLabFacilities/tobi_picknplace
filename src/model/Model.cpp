/*
 * Model.cpp
 *
 *  Created on: Nov 30, 2015
 *      Author: plueckin
 */

#include "Model.h"
#include <ros/ros.h>

using namespace std;

void Model::addListener(ModelListener* listener) {
    listeners.push_back(listener);
}

void Model::removeListener(ModelListener* listener) {
    vector<ModelListener*>::iterator i;
    for (i = listeners.begin(); i != listeners.end(); ++i) {
        if (listener == *i) {
            listeners.erase(i);
            return;
        }
    }
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
        ROS_ERROR("Requested number of joints wrong! (%i)", (int )angles.size());
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
    ROS_DEBUG("poses %d, names %d", (int )poses.size(), (int )names.size());
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

    tfTransformer.transform(ps, ps, ParamReader::getParamReader().frameOriginArm);

    ROS_INFO_STREAM("getEefPose() 2: " << ps.pose.position.x << "," << ps.pose.position.y << "," << ps.pose.position.z << "," << ps.header.frame_id);

    pose.translation.xMeter = ps.pose.position.x;
    pose.translation.yMeter = ps.pose.position.y;
    pose.translation.zMeter = ps.pose.position.z;
    pose.rotation.qw = ps.pose.orientation.w;
    pose.rotation.qx = ps.pose.orientation.x;
    pose.rotation.qy = ps.pose.orientation.y;
    pose.rotation.qz = ps.pose.orientation.z;
    pose.frame = ParamReader::getParamReader().frameOriginArm;
    return pose;
}

std::vector<moveit_msgs::Grasp> Model::generate_grasps_angle_trans(ObjectShape shape) {
    tfTransformer.transform(shape, shape, ParamReader::getParamReader().frameOriginArm);
    return graspGenerator.generate_grasps_angle_trans(shape.center.xMeter, shape.center.yMeter, shape.center.zMeter, shape.heightMeter);
}

std::vector<moveit_msgs::Grasp> Model::generate_grasps_angle_trans(moveit_msgs::CollisionObject shape) {
    tfTransformer.transform(shape, shape, ParamReader::getParamReader().frameOriginArm);
    return graspGenerator.generate_grasps_angle_trans(shape.primitive_poses[0].position.x, shape.primitive_poses[0].position.y, shape.primitive_poses[0].position.z, shape.primitives[0].dimensions[0]);
}
