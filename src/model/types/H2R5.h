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

    virtual void fillGrasp(moveit_msgs::Grasp& grasp);

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

    std::vector<moveit_msgs::PlaceLocation> generate_place_locations(
            EefPose obj);
    std::vector<moveit_msgs::PlaceLocation> generate_place_locations(
            ObjectShape shape);

    std::vector<moveit_msgs::Grasp> generate_grasps_angle_trans(
            ObjectShape shape);
    std::vector<moveit_msgs::Grasp> generate_grasps_angle_trans(
            moveit_msgs::CollisionObject shape);

    trajectory_msgs::JointTrajectory generate_close_eef_msg();
    trajectory_msgs::JointTrajectory generate_open_eef_msg();

};
