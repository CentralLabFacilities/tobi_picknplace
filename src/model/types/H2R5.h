/*
 * H2R5.h
 *
 *  Created on: Nov 30, 2015
 *      Author: plueckin
 */

#pragma once

#include "../Model.h"

#define H2R5_NAME "h2r5"

class H2R5: public Model {
public:
    typedef boost::shared_ptr<H2R5> Ptr;

    H2R5();
    virtual ~H2R5();

    virtual MoveResult moveTo(const std::string &poseName, bool plan);

    virtual void openEef(bool withSensors);
    virtual void closeEef(bool withSensors);

    virtual void motorsOn() {};
    virtual void motorsOff() {};

    virtual bool isSomethingInGripper() const {return true;};
    virtual SensorReadings getGripperSensors() const {};

    virtual GraspReturnType graspObject(ObjectShape obj, bool simulate,
            const std::string &startPose = "");
    virtual GraspReturnType graspObject(const std::string &obj,
            const std::string &surface, bool simulate,
            const std::string &startPose = "");

    virtual GraspReturnType placeObject(ObjectShape obj, bool simulate,
            const std::string &startPose = "");
    virtual GraspReturnType placeObject(EefPose obj, bool simulate,
            const std::string &startPose = "");
    virtual GraspReturnType placeObject(const std::string &surface,
            bool simulate, const std::string &startPose = "");

private:

    RosTools rosTools;

    ros::Publisher target_publisher; //todo: put in rostools

    ros::NodeHandle nh;

    std::vector<moveit_msgs::PlaceLocation> generate_place_locations(
            EefPose obj);
    std::vector<moveit_msgs::PlaceLocation> generate_place_locations(
            ObjectShape shape);

    std::vector<moveit_msgs::Grasp> generate_grasps_angle_trans(
            ObjectShape shape);
    std::vector<moveit_msgs::Grasp> generate_grasps_angle_trans(
            moveit_msgs::CollisionObject shape);

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

};
