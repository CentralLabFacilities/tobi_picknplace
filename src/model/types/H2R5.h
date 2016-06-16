/*
 * H2R5.h
 *
 *  Created on: Nov 30, 2015
 *      Author: plueckin
 */

#pragma once

#include "../Model.h"

class H2R5: public Model {
public:
    typedef boost::shared_ptr<H2R5> Ptr;

    H2R5();
    ~H2R5();

    virtual MoveResult moveTo(const std::string &poseName, bool plan);

    virtual void openEef(bool withSensors);
    virtual void closeEef(bool withSensors);

    virtual void motorsOn() {};
    virtual void motorsOff() {};

    virtual bool isSomethingInGripper() const {return true;};
    virtual SensorReadings getGripperSensors() const {};

    //virtual GraspReturnType graspObject(ObjectShape obj, bool simulate,
    //        const std::string &startPose = "");
    virtual GraspReturnType graspObject(const std::string &obj,
            const std::string &surface, bool simulate,
            const std::string &startPose = "");

    virtual GraspReturnType placeObject(ObjectShape obj, bool simulate,
            const std::string &startPose = "");
    virtual GraspReturnType placeObject(EefPose obj, bool simulate,
            const std::string &startPose = "");
    virtual GraspReturnType placeObject(const std::string &surface,
            bool simulate, const std::string &startPose = "");
    virtual trajectory_msgs::JointTrajectory generate_close_eef_msg();
    virtual trajectory_msgs::JointTrajectory generate_open_eef_msg();
private:

    RosTools rosTools;

    ros::Publisher target_publisher; //todo: put in rostools

    std::vector<moveit_msgs::PlaceLocation> generate_place_locations(
            EefPose obj);
    std::vector<moveit_msgs::PlaceLocation> generate_place_locations(
            ObjectShape shape);
    //std::vector<moveit_msgs::PlaceLocation> generate_place_locations(
    //        const std::string &surface);
};
