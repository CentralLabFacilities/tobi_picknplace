/*
 * GraspGenerator.h
 *
 *  Created on: Jul 2, 2015
 *      Author: lziegler
 */

#pragma once

#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/PlaceLocation.h>
#include <tf/tf.h>
#include <vector>

class GraspGenerator {
public:
	GraspGenerator();
	virtual ~GraspGenerator();

	virtual std::vector<moveit_msgs::Grasp> generate_grasps(std::string object_name) = 0;
	virtual std::vector<moveit_msgs::Grasp> generate_grasps(moveit_msgs::CollisionObject object) = 0;

	virtual std::vector<moveit_msgs::PlaceLocation> generate_placeloc()

	virtual std::vector<moveit_msgs::PlaceLocation> generate_placeloc_angle_only(double x, double y, double z);
	virtual std::vector<moveit_msgs::PlaceLocation> generate_placeloc_angle_trans(double x, double y, double z);
	virtual std::vector<moveit_msgs::PlaceLocation> generate_place_locations(double x, double y, double z, tf::Quaternion targetOrientation);
	virtual std::vector<moveit_msgs::PlaceLocation> generate_place_locations(double x, double y, double z, double w, double h, double d, tf::Quaternion targetOrientation);
	virtual moveit_msgs::Grasp build_grasp(tf::Transform t);
	virtual moveit_msgs::PlaceLocation build_place_location(tf::Transform t);



};

