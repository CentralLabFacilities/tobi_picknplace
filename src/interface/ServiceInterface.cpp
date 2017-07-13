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

    const std::string METHOD_PLACE_ON_SURFACE = "placeOnSurface";
    const std::string METHOD_OPEN = "openGripper";
    const std::string METHOD_OPEN_TOUCH = "openGripperWhenTouching";
    const std::string METHOD_CLOSE = "closeGripper";
    const std::string METHOD_PLAN = "planToPose";


    bool posture_callback(biron_posture_execution_msgs::BironPostureExecution::Request &request, biron_posture_execution_msgs::BironPostureExecution::Response &response) {
        ROS_DEBUG_STREAM("posture service callback");

        if(request.method == METHOD_PLACE_ON_SURFACE){
            ROS_INFO_STREAM("Method: " << METHOD_PLACE_ON_SURFACE);
            GraspReturnType grt = listener->requestPlaceObject(request.args, false);

            if(grt.result == GraspReturnType::SUCCESS){
                response.success = true;
            } else {
                response.success = false;
            }

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

