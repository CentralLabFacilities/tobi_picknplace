/*
 * ServiceInterface.cpp
 *
 *  Created on: Apr 29, 2015
 *      Author: lziegler
 */

#include "ServiceInterface.h"

#include <ros/ros.h>
#include "biron_posture_execution_msgs/BironPostureExecution.h"

/**
 * Internal class for private function. Hides implementation details in compilation unit.
 */
class ServiceInterface::Private {
public:

    Private(const std::string &service_name) :
            listener(new EmptyControlInterfaceListener()) {
            ROS_INFO_STREAM("creating service " << service_name);
            service_ptr_.reset(new ros::ServiceServer(nh.advertiseService(service_name, &Private::posture_callback, this)));
    }

    Private() :
            listener(new EmptyControlInterfaceListener()) {
    }

    Private(ControlInterfaceListener* listener) :
            listener(listener) {
    }

    virtual ~Private() {
    }

    ControlInterfaceListener* listener;
    ros::NodeHandle nh;
    boost::shared_ptr<ros::ServiceServer> service_ptr_;

    const std::string METHOD_PLACE_ON_SURFACE = "placeObjectOnSurface";
    const std::string METHOD_OPEN = "openGripper";
    const std::string METHOD_OPEN_TOUCH = "openGripperWhenTouching";
    const std::string METHOD_CLOSE = "closeGripper";
    const std::string METHOD_PLAN = "planToPose";
    const std::string METHOD_GRIPPER_SENSORS = "getGripperSensors";
    const std::string METHOD_IS_SOMETHING_IN_GRIPPER = "isSomethingInGripper";


    bool posture_callback(biron_posture_execution_msgs::BironPostureExecution::Request &request, biron_posture_execution_msgs::BironPostureExecution::Response &response) {


        std::string method = request.method;
        std::string args = request.args;
        ROS_DEBUG_STREAM("posture service callback: "<<method<<"("<<args<<")");

        if(request.method == METHOD_PLACE_ON_SURFACE){
            ROS_INFO_STREAM("Method: " << METHOD_PLACE_ON_SURFACE);
            GraspReturnType grt = listener->requestPlaceObject(request.args, false);

            if(grt.result == GraspReturnType::SUCCESS){
                response.success = true;
            } else {
                response.success = false;
            }
        }else if(request.method == METHOD_IS_SOMETHING_IN_GRIPPER){
            ROS_INFO_STREAM("Method: " << METHOD_IS_SOMETHING_IN_GRIPPER);
            response.success = listener->requestIsSomethingInGripper();
        } else if(request.method == METHOD_OPEN){
            ROS_INFO_STREAM("Method: " << METHOD_OPEN);
            listener->requestOpenGripper(false);
            response.success = true;
        } else if(request.method == METHOD_OPEN_TOUCH){
            ROS_INFO_STREAM("Method: " << METHOD_OPEN_TOUCH);
            listener->requestOpenGripper(true);
            response.success = true;
        } else if(request.method == METHOD_CLOSE){
            ROS_INFO_STREAM("Method: " << METHOD_CLOSE);
            listener->requestCloseGripper(false);
            response.success = true;
        } else if(request.method == METHOD_PLAN){
            ROS_INFO_STREAM("Method: " << METHOD_PLAN);
            bool success = executePose(request.args);
            response.success = success;
        } else if(request.method == METHOD_GRIPPER_SENSORS){
            ROS_INFO_STREAM("Method: " << METHOD_GRIPPER_SENSORS);
            SensorReadings sensorReadings = listener->requestGripperSensors();

            std::vector<double> data;
            std::string   keys[] = {"katana_r_inside_near_force_sensor",
                               "katana_r_inside_far_force_sensor",
                               "katana_r_outside_distance_sensor",
                               "katana_r_tip_distance_sensor",
                               "katana_r_inside_near_distance_sensor",
                               "katana_r_inside_far_distance_sensor",

                               "katana_l_inside_near_force_sensor",
                               "katana_l_inside_far_force_sensor",
                               "katana_l_outside_distance_sensor",
                               "katana_l_tip_distance_sensor",
                               "katana_l_inside_near_distance_sensor",
                               "katana_l_inside_far_distance_sensor",

                               "katana_wrist_middle_distance_sensor"};
            for (int i = 0; i < 13; i++) {
                std::string sensor = keys[i];
                double reading = sensorReadings.at(sensor);
                ROS_INFO_STREAM(sensor << "  : " << reading);
                data.push_back(reading);
            }
            response.sensordata = data;
        }

        return true;
    }

    bool executePose(std::string group_and_pose){

        std::stringstream ss(group_and_pose);
        std::string item;

        std::vector<std::string> tokens;

        while (std::getline(ss, item, ';')) {
            tokens.push_back(item);
        }

        std::string group_name = tokens[0];
        std::string pose_name = tokens[1];

        ROS_INFO_STREAM("Planning group " << group_name << " to pose " << pose_name);

        moveit::planning_interface::MoveGroupInterface m(group_name);
        bool pose_found = m.setNamedTarget(pose_name);

        if(!pose_found){
            ROS_ERROR_STREAM("Pose " << pose_name << " is not known for group " << group_name);
            return false;
        }

        moveit::planning_interface::MoveGroupInterface::Plan p;

        bool plan_found = m.plan(p);

        if(!plan_found) {
            ROS_ERROR_STREAM("No plan found to pose " << pose_name);
        }

        m.execute(p);

        return true;
    }

};

ServiceInterface::ServiceInterface(const std::string &service_name) : d(new Private(service_name)) {}

ServiceInterface::~ServiceInterface() {
}

void ServiceInterface::setListener(ControlInterfaceListener* listener) {
    d->listener = listener;
}

void ServiceInterface::removeListener() {
    d->listener = new EmptyControlInterfaceListener();
}

