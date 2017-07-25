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
            bool success = listener->requestPlanTo(request.args);
            response.success = success;
        } else if(request.method == METHOD_GRIPPER_SENSORS){
            ROS_INFO_STREAM("Method: " << METHOD_GRIPPER_SENSORS);
            SensorReadings sensorReadings = listener->requestGripperSensors();
            /*
            double forceRightInsideNear = data[0];
            double forceRightInsideFar = data[1];
            double infraredRightOutside = data[2];
            double infraredRightFront = data[3];
            double infraredRightInsideNear = data[4];
            double infraredRightInsideFar = data[5];
            double forceLeftInsideNear = data[6];
            double forceLeftInsideFar = data[7];
            double infraredLeftOutside = data[8];
            double infraredLeftFront = data[9];
            double infraredLeftInsideNear = data[10];
            double infraredLeftInsideFar = data[11];
            double infraredMiddle = data[12];

            double[] data = new double[13];
            for (int i = 0; i < sensorReadings.name.size(); i++) {
                string sensor = sensorReadings.name[i];
                double reading = sensorReadings.position[i];
                ROS_INFO_STREAM(sensor << "  : " << reading);
                data[i] = reading;
            }
            */
        }

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

