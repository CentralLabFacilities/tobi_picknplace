/*
 * AGNIInterface.h
 *
 *  Created on: Dec 10, 2015
 *      Author: plueckin
 */

#pragma once

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <grasping_msgs/GraspPlanningAction.h>

#include "../util/RosTools.h"
#include "../grasping/GraspGenerator.h"

#define AGNI_GRASP_NAME "agni"

class AGNIInterface: public GraspGenerator {
public:
	typedef boost::shared_ptr<AGNIInterface> Ptr;

    AGNIInterface();
    virtual ~AGNIInterface();

    virtual std::vector<grasping_msgs::Object> find_objects(bool plan_grasps);

    virtual std::vector<moveit_msgs::Grasp> generate_grasps(grasping_msgs::Object object);
    virtual std::vector<moveit_msgs::Grasp> generate_grasps(std::string object_name);
    virtual std::vector<moveit_msgs::Grasp> generate_grasps(moveit_msgs::CollisionObject object);
    virtual std::vector<moveit_msgs::Grasp> generate_grasps(ObjectShape shape);
        
    virtual std::vector<moveit_msgs::PlaceLocation> generate_place_locations(double x, double y, double z, double w, double h, double d, tf::Quaternion targetOrientation);


private:
	RosTools rosTools;
	std::string action_name_;
	ros::Publisher pub_markers;
	ros::ServiceClient grasp_viz_client;

	ros::NodeHandle nh_;

	boost::scoped_ptr<actionlib::SimpleActionClient<grasping_msgs::GraspPlanningAction> > cl_agni;

};
