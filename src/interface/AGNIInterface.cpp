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

using namespace std;

AGNIInterface::AGNIInterface() {
    name = AGNI_GRASP_NAME;

	cl_object_fitter.reset(
			new actionlib::SimpleActionClient<grasping_msgs::FindGraspableObjectsAction>(
					nh_, ParamReader::getParamReader().fitterNode, false));

	cl_agni.reset(
	            new actionlib::SimpleActionClient<grasping_msgs::GraspPlanningAction>(
	                    nh_, ParamReader::getParamReader().graspNode, false));
	rosTools.waitForAction(cl_object_fitter, ros::Duration(0, 0), ParamReader::getParamReader().fitterNode);
	rosTools.waitForAction(cl_agni, ros::Duration(0, 0), ParamReader::getParamReader().graspNode);

	if (cl_object_fitter->isServerConnected())
	    ROS_INFO_STREAM("Object fitter server connected!");

	if (cl_agni->isServerConnected())
        ROS_INFO_STREAM("Grasp manager server connected!");

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

    if (!cl_object_fitter) {
        ROS_ERROR_STREAM("Object fitter client not found");
        return graspable_objects;
    }

    if (!cl_object_fitter->isServerConnected()) {
        ROS_ERROR_STREAM("Object fitter client not connected");
        return graspable_objects;
    }

    grasping_msgs::FindGraspableObjectsGoal goal;
    goal.plan_grasps = plan_grasps;

    cl_object_fitter->sendGoal(goal);

    if(!cl_object_fitter->waitForResult(ros::Duration(10, 0))) { // wait for 10 seconds
        ROS_ERROR_STREAM("Object fitter timeout");
        return graspable_objects;
    }

    grasping_msgs::FindGraspableObjectsResult::ConstPtr results = cl_object_fitter->getResult();

    if(!results->objects.size()) {
        ROS_ERROR_STREAM("No objects found");
        return graspable_objects;
    }

    for(grasping_msgs::GraspableObject obj: results->objects) {
        graspable_objects.push_back(obj.object);
        if(plan_grasps && !obj.grasps.size()) {
            ROS_WARN_STREAM("No grasps for object " << obj.object.name << " found");
        }
    }

    ROS_DEBUG_STREAM("Found " << graspable_objects.size() << " objects.");

    display_primitives(results->objects);

    return graspable_objects;
}

vector<moveit_msgs::Grasp> AGNIInterface::generate_grasps(grasping_msgs::Object object) {

    vector<moveit_msgs::Grasp> grasps;

    if (!cl_agni) {
        ROS_ERROR_STREAM("AGNI client not found");
        return grasps;
    }

    if (!cl_agni->isServerConnected()) {
        ROS_ERROR_STREAM("AGNI client not connected");
        return grasps;
    }

    grasping_msgs::GraspPlanningGoal goal;
    

    goal.object = object;
    goal.group_name = ParamReader::getParamReader().groupArm;

    cl_agni->sendGoal(goal);

    if(!cl_agni->waitForResult(ros::Duration(15, 0))) { // wait for 15 seconds
        ROS_ERROR_STREAM("AGNI timeout");
        return grasps;
    }

    grasping_msgs::GraspPlanningResult::ConstPtr results = cl_agni->getResult();

    if(!results->grasps.size()) {
        ROS_ERROR_STREAM("No grasps found!");
        return grasps;
    }

    return results->grasps;

}

vector<moveit_msgs::Grasp> AGNIInterface::generate_grasps(std::string name) {

    grasping_msgs::Object object;
    object.name = name;

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
