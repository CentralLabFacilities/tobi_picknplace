/*
 * AGNIInterface.cpp
 *
 *  Created on: Dec 10, 2015
 *      Author: plueckin
 */

#include "AGNIInterface.h"
#include <stdio.h>

AGNIInterface::AGNIInterface() {

	client.reset(
			new actionlib::SimpleActionClient<grasping_msgs::FindGraspableObjectsAction>(
					nh_, "grasp_manager", false));

	rosTools.waitForAction(client, ros::Duration(0, 0), "grasp_manager");

	//todo: output whether server is connected to, timeout, etc, introduce checks

}

AGNIInterface::~AGNIInterface() {
	// TODO Auto-generated destructor stub
}

std::vector<moveit_msgs::Grasp> AGNIInterface::generate_grasps(
		std::string name) {
	return generate_grasps();
}

std::vector<moveit_msgs::Grasp> AGNIInterface::generate_grasps(
		moveit_msgs::CollisionObject object) {
	return generate_grasps();
}

std::vector<moveit_msgs::Grasp> AGNIInterface::generate_grasps(
		ObjectShape shape) {
	return generate_grasps();
}

std::vector<moveit_msgs::Grasp> AGNIInterface::generate_grasps() {
	if (!client) {
		ROS_ERROR_STREAM("AGNI Client not found");
	}
	if (!client->isServerConnected()) {
		ROS_ERROR_STREAM("AGNI Client not connected");
	}

	grasping_msgs::FindGraspableObjectsGoal goal;
	goal.plan_grasps = true;

	client->sendGoal(goal);
	client->waitForResult();

	grasping_msgs::FindGraspableObjectsResult::ConstPtr results = client->getResult();

	if(!results->objects.size()) {
		ROS_ERROR_STREAM("No objects found!");
		return std::vector<moveit_msgs::Grasp>();
	}

	if(!results->objects.at(0).grasps.size()) {
		ROS_ERROR_STREAM("No grasps for object found!");
		return std::vector<moveit_msgs::Grasp>();
	}

	return results->objects.at(0).grasps;

	/*for (grasping_msgs::GraspableObject o : results) {
		for (moveit_msgs::Grasp g : o.grasps) {
		}
	}*/
}

