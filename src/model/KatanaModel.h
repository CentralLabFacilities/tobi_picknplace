/*
 * KatanaModel.h
 *
 *  Created on: Apr 29, 2015
 *      Author: lziegler
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <vector>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>
#include <katana_msgs/JointMovementAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>

#include "Model.h"
#include "GraspGenerator.h"
#include "../util/TransformerTF.h"
#include "../util/RosTools.h"

class KatanaModel: public Model {
public:
	typedef boost::shared_ptr<KatanaModel> Ptr;

	KatanaModel();
	virtual ~KatanaModel();

	void addListener(ModelListener* listener);
	void removeListener(ModelListener* listener);

	virtual std::map<std::string, double> getJointAngles() const;
	virtual std::vector<std::string> getJointNames() const;
    virtual void setJointAngle(const std::string &joint, double angle);
    virtual void setJointAngles(const std::map<std::string, double> &angle);
    virtual void setJointAngles(const std::vector<double> &angles);
	virtual int getNumJoints() const;

	virtual void openGripper(bool withSensors);
	virtual void closeGripper(bool withSensors);
	virtual void moveToGripper(double target, bool withSensors);

	virtual void motorsOn();
	virtual void motorsOff();

	virtual EefPose getEefPose() const;
	virtual ArmPoses getRememberedPoses() const;
	virtual ArmPose getRememberedPose(const std::string &name) const;
	virtual MoveResult moveTo(const EefPose &pose, bool linear, bool orientation);
	virtual MoveResult moveTo(const std::string &poseName, bool plan);
	virtual void stop() const;

	virtual bool isSomethingInGripper() const;
	virtual SensorReadings getGripperSensors() const;

	virtual GraspReturnType graspObject(ObjectShape obj, bool simulate, const std::string &startPose="");
	virtual GraspReturnType placeObject(ObjectShape obj, bool simulate, const std::string &startPose="");
	virtual GraspReturnType placeObject(EefPose obj, bool simulate, const std::string &startPose="");

private:
	std::vector<ModelListener*> listeners;
	RosTools rosTools;
	planning_scene_monitor::CurrentStateMonitorPtr scene;

	ros::NodeHandle nh;
	moveit::planning_interface::MoveGroup *groupArm;
	moveit::planning_interface::MoveGroup *groupGripper;
	moveit::planning_interface::PlanningSceneInterface *planningScene;
	boost::scoped_ptr<actionlib::SimpleActionClient<moveit_msgs::PickupAction> > pickActionClient;
	boost::scoped_ptr<actionlib::SimpleActionClient<moveit_msgs::PlaceAction> > placeActionClient;
	boost::scoped_ptr<actionlib::SimpleActionClient<katana_msgs::JointMovementAction> > movementActionClient;

	ros::Subscriber sensor_subscriber;

	GraspGenerator graspGenerator;
	TransformerTF tfTransformer;

	geometry_msgs::PoseStamped lastGraspPose;
	double lastHeightAboveTable;

	mutable boost::mutex sensorMutex;
	std::map<std::string, short> currentSensorReadings;

	void sensorCallback(const sensor_msgs::JointStatePtr& sensorReadings);

	std::vector<moveit_msgs::Grasp> generate_grasps_angle_trans(ObjectShape shape);
	std::vector<moveit_msgs::PlaceLocation> generate_place_locations(EefPose obj);
	std::vector<moveit_msgs::PlaceLocation> generate_place_locations(ObjectShape shape);

	void attachDefaultObject();
	moveit_msgs::PlaceGoal buildPlaceGoal(const std::vector<moveit_msgs::PlaceLocation>& locations,
			bool simulate);
	katana_msgs::JointMovementGoal buildMovementGoal(const std::string &poseName);

	template<typename T>
	void waitForAction(const T &action, const ros::Duration &wait_for_server,
			const std::string &name) {
		ROS_DEBUG("Waiting for MoveGroup action server (%s)...", name.c_str());

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
			while (nh.ok() && !action->isServerConnected() && final_time > ros::Time::now()) {
				ros::WallDuration(0.02).sleep();
				ros::spinOnce();
			}
		}

		if (!action->isServerConnected())
			throw std::runtime_error(
					"Unable to connect to move_group action server within allotted time (2)");
		else
			ROS_DEBUG("Connected to '%s'", name.c_str());
	}


};
