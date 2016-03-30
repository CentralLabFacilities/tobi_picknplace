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

#define CENTROID_GRASP_NAME "centroid"

class CentroidGrasping: public GraspGenerator {
public:
	typedef boost::shared_ptr<CentroidGrasping> Ptr;

    CentroidGrasping();
    virtual ~CentroidGrasping();

    /**
     * ALL COORDINATES MUST BE IN ARM COORDINATES
     */

    virtual std::vector<grasping_msgs::Object> find_objects(bool plan_grasps);

    virtual std::vector<moveit_msgs::Grasp> generate_grasps(grasping_msgs::Object object) {};
    virtual std::vector<moveit_msgs::Grasp> generate_grasps(std::string name) {};
    virtual std::vector<moveit_msgs::Grasp> generate_grasps(ObjectShape shape);
    virtual std::vector<moveit_msgs::Grasp> generate_grasps(moveit_msgs::CollisionObject object);

    virtual std::vector<moveit_msgs::PlaceLocation> generate_placeloc_angle_only(double x, double y, double z);
    virtual std::vector<moveit_msgs::PlaceLocation> generate_placeloc_angle_trans(double x, double y, double z);
    virtual std::vector<moveit_msgs::PlaceLocation> generate_place_locations(double x, double y, double z, tf::Quaternion targetOrientation);
    virtual std::vector<moveit_msgs::PlaceLocation> generate_place_locations(double x, double y, double z, double w, double h, double d, tf::Quaternion targetOrientation);

private:
    moveit_msgs::Grasp build_grasp(tf::Transform t);
    moveit_msgs::PlaceLocation build_place_location(tf::Transform t);

    std::vector<moveit_msgs::Grasp> generate_grasps(double x, double y, double z, double height);
    std::vector<moveit_msgs::Grasp> generate_grasps_angle_only(double x, double y, double z);
};
