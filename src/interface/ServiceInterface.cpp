#include "ServiceInterface.h"


ServiceInterface::ServiceInterface(const std::string &serviceName) :
        serviceName(serviceName) {

    ROS_INFO_STREAM("creating Service Listener");
    servicePtr.reset(new ros::ServiceServer(nodeHandle.advertiseService(serviceName,&ServiceInterface::postureCallback,this)));
}

ServiceInterface::~ServiceInterface() {
}

void ServiceInterface::setListener(ControlInterfaceListener* listener) {
    this->listener = listener;
}

void ServiceInterface::removeListener() {
    this->listener = new EmptyControlInterfaceListener();
}

bool ServiceInterface::postureCallback(tobi_picknplace::BironPostureExecution::Request &request, tobi_picknplace::BironPostureExecution::Response &response){

    std::string method = request.method;
    std::string args = request.args;


    ROS_INFO_STREAM("method call: "+method);
    ROS_INFO_STREAM("args: "+args);

   /*
    //GraspReturnType grt;
    if(method=="planToPose"){
        boost::shared_ptr<bool> success(new bool(false));
        ROS_INFO_STREAM("Invoked findObj");
        this->listener->requestFindObjects();
        ROS_INFO_STREAM("Invoked planToPose");

        *success = listener->requestPlanTo(args);
    }else{

        ROS_INFO_STREAM("not planning");
    }
    */
    return true;
}
