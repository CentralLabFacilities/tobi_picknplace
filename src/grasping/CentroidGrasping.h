/*
 * CentroidGrasping.h
 *
 *  Created on: Jul 2, 2015
 *      Author: lziegler
 *  Created on: Dec 10, 2015
 *      Author: plueckin
 */

#pragma once

#include "GraspGenerator.h"

class CentroidGrasping: GraspGenerator {
public:
    CentroidGrasping();
    virtual ~CentroidGrasping();

    /**
     * ALL COORDINATES MUST BE IN ARM COORDINATES
     */
    std::vector<moveit_msgs::Grasp> generate_grasps_angle_only(double x, double y, double z);
    std::vector<moveit_msgs::Grasp> generate_grasps_angle_trans(double x, double y, double z,
            double height);

private:
    std::vector<moveit_msgs::PlaceLocation> generate_placeloc_angle_only(double x, double y, double z);
    std::vector<moveit_msgs::PlaceLocation> generate_placeloc_angle_trans(double x, double y, double z);
    std::vector<moveit_msgs::PlaceLocation> generate_place_locations(double x, double y, double z, tf::Quaternion targetOrientation);
    std::vector<moveit_msgs::PlaceLocation> generate_place_locations(double x, double y, double z, double w, double h, double d, tf::Quaternion targetOrientation);
    moveit_msgs::Grasp build_grasp(tf::Transform t);
    moveit_msgs::PlaceLocation build_place_location(tf::Transform t);
};
