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
#include <grasping_msgs/Object.h>

using namespace std;

AGNIInterface::AGNIInterface() {

    name = AGNI_GRASP_NAME;

	//string service = "/display_grasp";

	pub_markers = nh_.advertise<visualization_msgs::Marker>("/primitive_marker", 10);
	//ros::service::waitForService(service);
	//grasp_viz_client = nh_.serviceClient<grasp_viewer::DisplayGrasps>(service);

}

AGNIInterface::~AGNIInterface() {
	// TODO Auto-generated destructor stub
}

vector<grasping_msgs::Object> AGNIInterface::find_objects(bool plan_grasps = false) {

    vector<grasping_msgs::Object> graspable_objects;

    return graspable_objects;
}

vector<moveit_msgs::Grasp> AGNIInterface::generate_grasps(grasping_msgs::Object object) {

    vector<moveit_msgs::Grasp> grasps; 
    return grasps;

}

vector<moveit_msgs::Grasp> AGNIInterface::generate_grasps(std::string name) {

    grasping_msgs::Object object;
    if(!rosTools.getGraspingObjectByName(name, object)){
      vector<moveit_msgs::Grasp> grasps;
      ROS_DEBUG_STREAM("No CollisionObject matching with name: " << name);
      return grasps;
    }
    return generate_grasps(object);
}


vector<moveit_msgs::Grasp> AGNIInterface::generate_grasps(
		moveit_msgs::CollisionObject object) {
    return generate_grasps(object.id);
}

vector<moveit_msgs::Grasp> AGNIInterface::generate_grasps(
		ObjectShape shape) {
	return generate_grasps("dummy");
}

std::vector<moveit_msgs::PlaceLocation> AGNIInterface::generate_place_locations(double x, double y, double z, double w, double h, double d, tf::Quaternion targetOrientation) {
    std::vector<moveit_msgs::PlaceLocation> pls;
    
    
    //TODO: fill method;
    
    return pls;
}



void AGNIInterface::display_primitives(const vector<grasping_msgs::GraspableObject>& grasps) {

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
			marker.scale.x = shape.dimensions[shape_msgs::SolidPrimitive::BOX_X];
			marker.scale.y = shape.dimensions[shape_msgs::SolidPrimitive::BOX_Y];
			marker.scale.z = shape.dimensions[shape_msgs::SolidPrimitive::BOX_Z];
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
		i++;
	}

}
