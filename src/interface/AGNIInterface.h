/*
 * AGNIInterface.h
 *
 *  Created on: Dec 10, 2015
 *      Author: plueckin
 */

#pragma once

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <grasping_msgs/FindGraspableObjectsAction.h>

#include "../util/RosTools.h"
#include "../grasping/GraspGenerator.h"

#define AGNI_GRASP_NAME "agni"

class AGNIInterface: public GraspGenerator {
public:
	typedef boost::shared_ptr<AGNIInterface> Ptr;

    AGNIInterface();
    virtual ~AGNIInterface();

    virtual std::vector<moveit_msgs::Grasp> generate_grasps(std::string object_name);
	virtual std::vector<moveit_msgs::Grasp> generate_grasps(moveit_msgs::CollisionObject object);
	virtual std::vector<moveit_msgs::Grasp> generate_grasps(ObjectShape shape);

private:
	RosTools rosTools;
	std::string action_name_;
	ros::Publisher pub_markers;
	ros::ServiceClient grasp_viz_client;

	ros::NodeHandle nh_;

	boost::scoped_ptr<actionlib::SimpleActionClient<grasping_msgs::FindGraspableObjectsAction> > client;

	std::vector<moveit_msgs::Grasp> generate_grasps();

	void display_grasps(const std::vector<grasping_msgs::GraspableObject>& grasps);

};
