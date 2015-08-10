/*
 * TransformerTF.h
 *
 *  Created on: Jul 2, 2015
 *      Author: lziegler
 */

#pragma once

#include "../model/ModelTypes.h"

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

class TransformerTF {
public:
	TransformerTF();
	virtual ~TransformerTF();

	bool transform(double x, double y, double z, const std::string &from, double &xOut,double &yOut,double &zOut, const std::string &to) const;
	bool transform(const EefPose &pose, EefPose &poseOut, const std::string &to) const;
	bool transform(const ObjectShape &object, ObjectShape &objectOut, const std::string &to) const;
	bool transform(const geometry_msgs::PoseStamped &pose, geometry_msgs::PoseStamped &poseOut, const std::string &to) const;
	bool transform(const geometry_msgs::Vector3Stamped &pose, geometry_msgs::Vector3Stamped &poseOut, const std::string &to) const;

private:
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener;
};

