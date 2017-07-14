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

	cl_agni.reset(
	            new actionlib::SimpleActionClient<grasping_msgs::GraspPlanningAction>(
	                    nh_, ParamReader::getParamReader().graspNode, false));

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
    rosTools.clear_grasps();
    rosTools.clear_grasps_markerarray();


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
    ROS_INFO_STREAM("Number of generated grasps: " <<results->grasps.size());
    return results->grasps;

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