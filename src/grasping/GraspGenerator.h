/*
 * GraspGenerator.h
 *
 *  Created on: Jul 2, 2015
 *      Author: lziegler
 */

#pragma once

#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/PlaceLocation.h>
#include <grasping_msgs/Object.h>
#include <tf/tf.h>
#include <vector>

#include "../util/RosTools.h"

class GraspGenerator {
public:
	typedef boost::shared_ptr<GraspGenerator> Ptr;

	GraspGenerator(){};
	virtual ~GraspGenerator(){};

	virtual std::vector<grasping_msgs::Object> find_objects(bool plan_grasps) = 0;

	virtual std::vector<moveit_msgs::Grasp> generate_grasps(grasping_msgs::Object object) = 0;
	virtual std::vector<moveit_msgs::Grasp> generate_grasps(std::string object_name) = 0;
	virtual std::vector<moveit_msgs::Grasp> generate_grasps(moveit_msgs::CollisionObject object) = 0;
	virtual std::vector<moveit_msgs::Grasp> generate_grasps(ObjectShape shape) = 0;

	virtual std::vector<moveit_msgs::PlaceLocation> generate_placeloc_angle_only(double x, double y, double z) {};
	virtual std::vector<moveit_msgs::PlaceLocation> generate_placeloc_angle_trans(double x, double y, double z) {};
	virtual std::vector<moveit_msgs::PlaceLocation> generate_place_locations(double x, double y, double z, tf::Quaternion targetOrientation) {};
	virtual std::vector<moveit_msgs::PlaceLocation> generate_place_locations(double x, double y, double z, double w, double h, double d, tf::Quaternion targetOrientation) {};

	virtual std::string getName() { return name; }

protected:
	std::string name;

};

