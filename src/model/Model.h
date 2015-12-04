/*
 * Model.h
 *
 *  Created on: Apr 29, 2015
 *      Author: lziegler
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <vector>

#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include "ModelListener.h"
#include "ModelTypes.h"

#include "GraspGenerator.h"
#include "../util/ParamReader.h"
#include "../util/TransformerTF.h"
#include "../util/RosTools.h"

class Model {
public:
    typedef boost::shared_ptr<Model> Ptr;

    Model() {
    }
    virtual ~Model() {
    }

    virtual void addListener(ModelListener* listener);
    virtual void removeListener(ModelListener* listener);

    virtual int getNumJoints() const;
    virtual std::vector<std::string> getJointNames() const;
    virtual std::map<std::string, double> getJointAngles() const;
    virtual void setJointAngle(const std::string &joint, double angle);
    virtual void setJointAngles(const std::map<std::string, double> &angle);
    virtual void setJointAngles(const std::vector<double> &angles);

    virtual void openEef(bool withSensors) = 0;
    virtual void closeEef(bool withSensors) = 0;

    virtual void motorsOn() = 0;
    virtual void motorsOff() = 0;

    virtual EefPose getEefPose() const;
    virtual ArmPoses getRememberedPoses() const;
    virtual ArmPose getRememberedPose(const std::string &name) const;
    virtual void stop() const;

    virtual bool isSomethingInGripper() const = 0;
    virtual SensorReadings getGripperSensors() const = 0;

    virtual MoveResult moveTo(const EefPose &pose, bool linear,
            bool orientation);
    virtual MoveResult moveTo(const std::string &poseName,
            bool plan = true) = 0;

    virtual GraspReturnType graspObject(const std::string &obj,
            const std::string &surface, bool simulate,
            const std::string &startPose = "") = 0;
    virtual GraspReturnType graspObject(ObjectShape object, bool simulate,
            const std::string &startPose = "") = 0;
    virtual GraspReturnType graspObject(const std::string &obj,
            const std::string &surface,
            const std::vector<moveit_msgs::Grasp> &grasps,
            double tableHeightArmCoords, bool simulate,
            const std::string &startPose);

    virtual GraspReturnType placeObject(ObjectShape obj, bool simulate,
            const std::string &startPose = "") = 0;
    virtual GraspReturnType placeObject(EefPose obj, bool simulate,
            const std::string &startPose = "") = 0;
    virtual GraspReturnType placeObject(const std::string &surface,
            bool simulate, const std::string &startPose = "") = 0;

    virtual GraspReturnType placeObject(const std::string &surface,
            std::vector<moveit_msgs::PlaceLocation> placeLocation,
            bool simulate, const std::string &startPose = "");

protected:
    RosTools rosTools;

    boost::scoped_ptr<actionlib::SimpleActionClient<moveit_msgs::PickupAction> > pickActionClient;
    boost::scoped_ptr<actionlib::SimpleActionClient<moveit_msgs::PlaceAction> > placeActionClient;

    std::vector<ModelListener*> listeners;
    std::vector<std::string> touchlinks;
    std::string frame;

    moveit::planning_interface::MoveGroup *groupArm;
    moveit::planning_interface::MoveGroup *groupEe;

    GraspGenerator graspGenerator;
    TransformerTF tfTransformer;

    double lastHeightAboveTable;
    geometry_msgs::PoseStamped lastGraspPose;

private:

    void attachDefaultObject();

    moveit_msgs::PlaceGoal buildPlaceGoal(const std::string &surface,
            const std::vector<moveit_msgs::PlaceLocation>& locations,
            bool simulate);
    moveit_msgs::PickupGoal buildPickupGoal(const std::string &obj,
            const std::string &surface,
            const std::vector<moveit_msgs::Grasp> &grasps, bool simulate);

};

