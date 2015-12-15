/*
 * AGNIInterface.cpp
 *
 *  Created on: Dec 10, 2015
 *      Author: plueckin
 */

#include "AGNIInterface.h"

#include <stdio.h>
#include <grasp_viewer/DisplayGrasps.h>

#include "../util/ParamReader.h"

AGNIInterface::AGNIInterface() {

	client.reset(
			new actionlib::SimpleActionClient<grasping_msgs::FindGraspableObjectsAction>(
					nh_, "grasp_manager", false));

	rosTools.waitForAction(client, ros::Duration(0, 0), "grasp_manager");

	std::string service = "/display_grasp";

	pub_markers = nh_.advertise<visualization_msgs::Marker>("/primitive_marker", 10);
	ros::service::waitForService(service);
	grasp_viz_client = nh_.serviceClient<grasp_viewer::DisplayGrasps>(service);

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

	display_grasps(results->objects);

	return results->objects.at(0).grasps;

	/*for (grasping_msgs::GraspableObject o : results) {
		for (moveit_msgs::Grasp g : o.grasps) {
		}
	}*/
}


void AGNIInterface::display_grasps(const std::vector<grasping_msgs::GraspableObject>& grasps) {

	uint i = 0;
	for(grasping_msgs::GraspableObject o : grasps) {
		visualization_msgs::Marker marker;

		shape_msgs::SolidPrimitive shape = o.object.primitives[0];
		geometry_msgs::Pose pose = o.object.primitive_poses[0];

		if(shape.type == shape_msgs::SolidPrimitive::CYLINDER) {
			marker.type = visualization_msgs::Marker::CYLINDER;
			marker.scale.x = 2 * shape.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
			marker.scale.y = 2 * shape.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
			marker.scale.z = shape.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT];
		}
		if(shape.type == shape_msgs::SolidPrimitive::BOX) {
			marker.type = visualization_msgs::Marker::CUBE;
			marker.scale.x = 2 * shape.dimensions[shape_msgs::SolidPrimitive::BOX_X];
			marker.scale.y = 2 * shape.dimensions[shape_msgs::SolidPrimitive::BOX_Y];
			marker.scale.z = 2 * shape.dimensions[shape_msgs::SolidPrimitive::BOX_Z];
		}
		if(shape.type == shape_msgs::SolidPrimitive::SPHERE) {
			marker.type = visualization_msgs::Marker::SPHERE;
			marker.scale.x = 2 * shape.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS];
			marker.scale.y = 2 * shape.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS];
			marker.scale.z = 2 * shape.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS];
		}

		marker.header.frame_id = ParamReader::getParamReader().frameArm;
		marker.color.r = 0.0;
		marker.color.g = 0.5;
		marker.color.b = 0.0;
		marker.color.a = 0.5;
		marker.pose = pose;
		marker.id = i;

		pub_markers.publish(marker);

		grasp_viewer::DisplayGraspsRequest disp_req; //note: also possible to use displaygrasps.request...
		grasp_viewer::DisplayGraspsResponse disp_res;

		disp_req.grasps = o.grasps;
		grasp_viz_client.call(disp_req, disp_res);

		i++;
	}

}
