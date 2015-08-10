/*
 * RosTools.h
 *
 *  Created on: Jul 12, 2015
 *      Author: lziegler
 */

#pragma once

#include <ros/ros.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/PlaceLocation.h>
#include <moveit/move_group_interface/move_group.h>
#include "../model/ModelTypes.h"
#include "TransformerTF.h"

class RosTools {
public:
	RosTools();
	virtual ~RosTools();

	MoveResult moveResultFromMoveit(moveit::planning_interface::MoveItErrorCode errorCode);
	GraspReturnType::GraspResult graspResultFromMoveit(moveit::planning_interface::MoveItErrorCode errorCode);

	void publish_collision_object(ObjectShape shape, double sleep_seconds);
	void remove_collision_object();
	void detach_collision_object();
	void attach_collision_object();
	bool has_attached_object();
	void publish_grasps_as_markerarray(std::vector<moveit_msgs::Grasp> grasps);
	void publish_place_locations_as_markerarray(std::vector<moveit_msgs::PlaceLocation> loc);

	void clear_octomap(double sleep_seconds = 1.0);

	std::string getDefaultObjectName() const;

private:
	TransformerTF tfTransformer;

	ros::NodeHandle nh;
	ros::Publisher grasps_marker;
	ros::Publisher object_publisher;
	ros::Publisher object_att_publisher;

	ros::ServiceClient clearOctomapClient;
};
