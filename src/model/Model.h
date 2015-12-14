/*
 * Model.h
 *
 *  Created on: Apr 29, 2015
 *      Author: lziegler
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <vector>

#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include "ModelTypes.h"

#include "../grasping/GraspGenerator.h"
#include "../util/ParamReader.h"
#include "../util/TransformerTF.h"
#include "../util/RosTools.h"

class Model {
public:
    typedef boost::shared_ptr<Model> Ptr;

    Model();
    virtual ~Model() {
    }

    virtual int getNumJoints() const;
    virtual std::vector<std::string> getJointNames() const;
    virtual std::vector<std::string> getEefJointNames() const;
    virtual std::map<std::string, double> getJointAngles() const;
    virtual void setJointAngle(const std::string &joint, double angle);
    virtual void setJointAngles(const std::map<std::string, double> &angle);
    virtual void setJointAngles(const std::vector<double> &angles);

    virtual void openEef(bool withSensors) = 0;
    virtual void closeEef(bool withSensors) = 0;

    virtual void motorsOn() = 0;
    virtual void motorsOff() = 0;

    virtual void fillGrasp(moveit_msgs::Grasp& grasp) = 0;

    virtual EefPose getEefPose() const;
    virtual ArmPoses getRememberedPoses() const;
    virtual ArmPose getRememberedPose(const std::string &name) const;
    virtual void stop() const;

    virtual bool isSomethingInGripper() const = 0;
    virtual SensorReadings getGripperSensors() const = 0;

    virtual MoveResult moveTo(const EefPose &pose, bool linear,
            bool orientation);
    virtual MoveResult moveTo(const std::string &poseName,
            bool plan = true) = 0;

    virtual GraspReturnType graspObject(const std::string &obj,
            const std::string &surface, bool simulate,
            const std::string &startPose = "") = 0;
    virtual GraspReturnType graspObject(ObjectShape object, bool simulate,
            const std::string &startPose = "") = 0;
    virtual GraspReturnType graspObject(const std::string &obj,
            const std::string &surface,
            const std::vector<moveit_msgs::Grasp> &grasps,
            double tableHeightArmCoords, bool simulate,
            const std::string &startPose);

    virtual GraspReturnType placeObject(ObjectShape obj, bool simulate,
            const std::string &startPose = "") = 0;
    virtual GraspReturnType placeObject(EefPose obj, bool simulate,
            const std::string &startPose = "") = 0;
    virtual GraspReturnType placeObject(const std::string &surface,
            bool simulate, const std::string &startPose = "") = 0;

    virtual GraspReturnType placeObject(const std::string &surface,
            std::vector<moveit_msgs::PlaceLocation> placeLocation,
            bool simulate, const std::string &startPose = "");

protected:

    RosTools rosTools;

    ros::NodeHandle nh;

    boost::scoped_ptr<actionlib::SimpleActionClient<moveit_msgs::PickupAction> > pickActionClient;
    boost::scoped_ptr<actionlib::SimpleActionClient<moveit_msgs::PlaceAction> > placeActionClient;

    std::vector<std::string> touchlinks;
    std::string frame;

    moveit::planning_interface::MoveGroup *groupArm;
    moveit::planning_interface::MoveGroup *groupEe;

    GraspGenerator graspGenerator;
    TransformerTF tfTransformer;

    double lastHeightAboveTable;
    geometry_msgs::PoseStamped lastGraspPose;

    template<typename T>
    void waitForAction(const T &action, const ros::Duration &wait_for_server,
            const std::string &name) {
        ROS_DEBUG("Waiting for MoveGroup action server (%s)...", name.c_str());

        // in case ROS time is published, wait for the time data to arrive
        ros::Time start_time = ros::Time::now();
        while (start_time == ros::Time::now()) {
            ros::WallDuration(0.01).sleep();
            ros::spinOnce();
        }

        // wait for the server (and spin as needed)
        if (wait_for_server == ros::Duration(0, 0)) {
            while (nh.ok() && !action->isServerConnected()) {
                ros::WallDuration(0.02).sleep();
                ros::spinOnce();
            }
        } else {
            ros::Time final_time = ros::Time::now() + wait_for_server;
            while (nh.ok() && !action->isServerConnected()
                    && final_time > ros::Time::now()) {
                ros::WallDuration(0.02).sleep();
                ros::spinOnce();
            }
        }

        if (!action->isServerConnected())
            throw std::runtime_error(
                    "Unable to connect to move_group action server within allotted time (2)");
        else
            ROS_DEBUG("Connected to '%s'", name.c_str());
    }

private:

    void attachDefaultObject();

    moveit_msgs::PlaceGoal buildPlaceGoal(const std::string &surface,
            const std::vector<moveit_msgs::PlaceLocation>& locations,
            bool simulate);
    moveit_msgs::PickupGoal buildPickupGoal(const std::string &obj,
            const std::string &surface,
            const std::vector<moveit_msgs::Grasp> &grasps, bool simulate);

};

