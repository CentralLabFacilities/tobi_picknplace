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
#include <moveit_msgs/PlanningScene.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h> 
#include "../model/ModelTypes.h"
#include "TransformerTF.h"
#include <grasping_msgs/Object.h> 
#include <shape_msgs/Plane.h>
#include <geometry_msgs/Pose.h>

class RosTools {
private:
    TransformerTF tfTransformer;

    ros::NodeHandle nh;
    ros::Publisher grasps_marker;
    ros::Publisher grasps_marker_red;
    ros::Publisher grasps_marker_green;
    ros::Publisher grasps_marker_white;

    ros::Publisher object_publisher;
    ros::Publisher object_att_publisher;

    ros::ServiceClient clearOctomapClient;
    ros::ServiceClient grasp_viz_client;

    ros::Subscriber scene_subscriber;
	ros::Publisher scene_publisher;

	std::vector<moveit_msgs::CollisionObject> manipulationObjects;
	std::vector<moveit_msgs::CollisionObject> manipulationSurfaces;

    mutable boost::mutex sceneMutex;
    moveit_msgs::PlanningScene currentPlanningScene;
    moveit::planning_interface::PlanningSceneInterface planningInterface;


	void removeFromManipulationObjects(const std::string& id);

public:
	RosTools();
	virtual ~RosTools();

	MoveResult moveResultFromMoveit(moveit::planning_interface::MoveItErrorCode errorCode);
	GraspReturnType::GraspResult graspResultFromMoveit(moveit::planning_interface::MoveItErrorCode errorCode);

	void clear_collision_objects(bool with_surface = false);
	void publish_collision_object(grasping_msgs::Object msg);
	
	void publish_collision_object(const std::string &id, ObjectShape shape, double sleep_seconds);
	void remove_collision_object(const std::string id);
	void detach_collision_object();
	void attach_collision_object(moveit_msgs::AttachedCollisionObject& attached_object);
	bool has_attached_object();
	void publish_grasps_as_markerarray(std::vector<moveit_msgs::Grasp> grasps, std::string color = "blue");
	void clear_grasps_markerarray();
	void display_grasps(const std::vector<moveit_msgs::Grasp> &grasps);
	void clear_grasps();
	void publish_place_locations_as_markerarray(std::vector<moveit_msgs::PlaceLocation> loc);

	void clear_octomap(double sleep_seconds = 1.0);

	std::string getDefaultObjectName() const;

	bool getCollisionObjectByName(const std::string &id, moveit_msgs::CollisionObject &obj);
	bool getCollisionObjectByHeigth(const double &h, moveit_msgs::CollisionObject &obj, const std::string& nameFilter = "");
	bool getGraspingObjectByName(const std::string &name, grasping_msgs::Object &msg);
	grasping_msgs::Object convertMoveItToGrasping(moveit_msgs::CollisionObject obj);

	template<typename T>
	void waitForAction(const T &action, const ros::Duration &wait_for_server,
			const std::string &name) {
		ROS_DEBUG("Waiting for action server (%s)...", name.c_str());

		// in case ROS time is published, wait for the time data to arrive
		ros::Time start_time = ros::Time::now();
		while (start_time == ros::Time::now()) {
			ros::WallDuration(0.01).sleep();
			ros::spinOnce();
		}

		// wait for the server (and spin as needed)
		if (wait_for_server == ros::Duration(0, 0)) {
			while (nh.ok() && !action->isServerConnected()) {
				ros::WallDuration(0.02).sleep();
				ros::spinOnce();
			}
		} else {
			ros::Time final_time = ros::Time::now() + wait_for_server;
			while (nh.ok() && !action->isServerConnected()
					&& final_time > ros::Time::now()) {
				ros::WallDuration(0.02).sleep();
				ros::spinOnce();
			}
		}

		if (!action->isServerConnected())
			throw std::runtime_error(
					"Unable to connect to action server within allotted time (2)");
		else
			ROS_DEBUG("Connected to '%s'", name.c_str());
	}

private:

    void sceneCallback(const moveit_msgs::PlanningScene& currentScene);
};
