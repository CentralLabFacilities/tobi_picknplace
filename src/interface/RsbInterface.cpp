/*
 * RsbInterface.cpp
 *
 *  Created on: Apr 29, 2015
 *      Author: lziegler
 */

#include "RsbInterface.h"
#include "../model/ModelTypes.h"

#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/smart_ptr/make_shared_object.hpp>

#include <rsc/misc/SignalWaiter.h>

#include <ros/ros.h>

#define BOOST_SIGNALS_NO_DEPRECATION_WARNING
#include <rsb/Factory.h>
#include <rsb/patterns/LocalServer.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsb/converter/Repository.h>

#include <rst/kinematics/JointAngles.pb.h>
#include <rst/geometry/Pose.pb.h>
#include <rst/geometry/BoundingBox3DFloat.pb.h>
#include <rst/generic/Dictionary.pb.h>

using namespace std;

using namespace rsb;
using namespace rsb::patterns;
using namespace rsb::converter;

using namespace rst::kinematics;
using namespace rst::geometry;
using namespace rst::generic;

/**
 * An adapter to use boost functions for callbacks.
 * Remark: Remove this as soon as RSB 0.12 is in use. It comes with an own implementation!!!
 */
template<class RequestType, class ReplyType>
class FunctionCallback: public LocalServer::CallbackBase {
public:
    /**
     * Type of functions that can be accepted.
     */
    typedef boost::function<
            typename boost::shared_ptr<ReplyType>(
                    typename boost::shared_ptr<RequestType>)> FunctionType;

    explicit FunctionCallback(FunctionType function,
            const std::string& requestType = rsc::runtime::typeName(
                    typeid(RequestType)), const std::string& replyType =
                    rsc::runtime::typeName(typeid(ReplyType))) :
            CallbackBase(requestType, replyType), function(function) {
    }

private:
    EventPtr intlCall(const std::string& /*methodName*/, EventPtr request) {
        boost::shared_ptr<RequestType> argument = boost::static_pointer_cast<
                RequestType>(request->getData());
        boost::shared_ptr<ReplyType> result = function(argument);
        EventPtr reply(new Event());
        reply->setType(getReplyType());
        reply->setData(result);
        return reply;
    }
    FunctionType function;
};

/**
 * Internal class for private function. Hides implementation details in compilation unit.
 */
class RsbInterface::Private {
public:
    Private() :
            listener(new EmptyControlInterfaceListener()) {
    }
    Private(ControlInterfaceListener* listener) :
            listener(listener) {
    }
    virtual ~Private() {
    }

    LocalServerPtr server;
    ControlInterfaceListener* listener;

    boost::shared_ptr<JointAngles> listAngles() {
        ROS_DEBUG_STREAM("Invoked ListAngles");
        map<string, double> joints = listener->requestJointAngles();

        boost::shared_ptr<JointAngles> angles(new JointAngles());
        for (map<string, double>::iterator i = joints.begin();
                i != joints.end(); ++i) {
            angles->add_angles(i->second);
            ROS_DEBUG_STREAM(
                    "joint " << i->first << " with value " << i->second);
        }
        return angles;
    }

    boost::shared_ptr<void> moveJoints(boost::shared_ptr<JointAngles> input) {
        ROS_DEBUG_STREAM("Invoked moveJoints");
        if (listener->requestNumJoints() != input->angles_size()) {
            ROS_ERROR_STREAM(
                    "Size in MoveJoints: " << input->angles_size() << "does not match armSize: " << listener->requestNumJoints());
        } else {
            vector<double> angles;
            for (int i = 0; i < input->angles_size(); i++) {
                //model->setJointAngle(i, (double) input->angles(i));
                angles.push_back((double) input->angles(i));
            }
            listener->requestMoveJointAngles(angles);
        }
        return boost::shared_ptr<void>();
    }

    boost::shared_ptr<bool> goTo(boost::shared_ptr<rst::geometry::Pose> input,
            bool linear, bool orientation) {
        boost::shared_ptr<bool> sucess(new bool(true));
        ROS_DEBUG_STREAM("Invoked gotToLinear");
        try {
            *sucess = listener->requestMoveTo(convert(input), linear,
                    orientation);
        } catch (std::exception& e) {
            *sucess = false;
            ROS_ERROR_STREAM(e.what());
            return sucess;
        }
        return sucess;
    }

    boost::shared_ptr<bool> goToLinear(
            boost::shared_ptr<rst::geometry::Pose> input) {
        ROS_DEBUG_STREAM("Invoked gotToLinear");
        return goTo(input, true, false);
    }

    boost::shared_ptr<bool> goToNonLinear(
            boost::shared_ptr<rst::geometry::Pose> input) {
        ROS_DEBUG_STREAM("Invoked gotToNonLinear");
        return goTo(input, false, false);
    }

    boost::shared_ptr<bool> goToLinearOrient(
            boost::shared_ptr<rst::geometry::Pose> input) {
        ROS_DEBUG_STREAM("Invoked gotToLinearOrient");
        return goTo(input, true, true);
    }

    boost::shared_ptr<bool> goToNonLinearOrient(
            boost::shared_ptr<rst::geometry::Pose> input) {
        ROS_DEBUG_STREAM("Invoked gotToNonLinearOrient");
        return goTo(input, false, true);
    }

    boost::shared_ptr<rst::geometry::Pose> getPosition() {
        ROS_DEBUG_STREAM("Invoked getPosition");
        EefPose position = listener->requestEefPose();
        boost::shared_ptr<rst::geometry::Pose> poseOut = convert(position);
        ROS_DEBUG_STREAM("Received following pose: " << poseOut->DebugString());
        return poseOut;
    }

    boost::shared_ptr<void> openGripper() {
        ROS_DEBUG_STREAM("Invoked openGripper");
        listener->requestOpenGripper(false);
        return boost::shared_ptr<void>();
    }

    boost::shared_ptr<void> openGripperWhenTouching() {
        ROS_DEBUG_STREAM("Invoked openGripperWhenTouching");
        listener->requestOpenGripper(true);
        return boost::shared_ptr<void>();
    }

    boost::shared_ptr<std::string> findNearestPose() {
        ROS_DEBUG_STREAM("Invoked findNearestPose");
        std::string ret = listener->requestNearestPose();
        return boost::make_shared<std::string>(ret.c_str());
    }

    boost::shared_ptr<void> closeGripper() {
        ROS_DEBUG_STREAM("Invoked closeGripper");
        listener->requestCloseGripper(false);
        return boost::shared_ptr<void>();
    }
    
    

    boost::shared_ptr<void> closeGripperByForce() {
        ROS_DEBUG_STREAM("Invoked closeGripperByForce");
        listener->requestCloseGripper(true);
        return boost::shared_ptr<void>();
    }

    boost::shared_ptr<void> motorsOn() {
        ROS_DEBUG_STREAM("Invoked motorsOn");
        listener->requestMotorsOn();
        return boost::shared_ptr<void>();
    }
    boost::shared_ptr<void> motorsOff() {
        ROS_DEBUG_STREAM("Invoked motorsOff");
        listener->requestMotorsOff();
        return boost::shared_ptr<void>();
    }

    boost::shared_ptr<bool> isSomethingInGripper() {
        boost::shared_ptr<bool> success(new bool(true));
        ROS_DEBUG_STREAM("Invoked isSomethingInGripper");
        if (!listener->requestIsSomethingInGripper()) {
            *success = false;
        }
        return success;
    }

    boost::shared_ptr<Dictionary> getGripperSensors() {
        ROS_DEBUG_STREAM("Invoked getGripperSensors");
        map<string, short> sensorValues = listener->requestGripperSensors();
        return convert(sensorValues);
    }
    
    void requestFindObjects() {
        //TODO
    }

    boost::shared_ptr<Dictionary> listPoses() {
        ROS_DEBUG_STREAM("Invoked listPoses");

        boost::shared_ptr<Dictionary> output(new rst::generic::Dictionary());

        rst::generic::Value* newVal;
        KeyValuePair* key;

        ArmPoses poses = listener->requestPoses();

        for (ArmPoses::iterator poseIt = poses.begin(); poseIt != poses.end();
                ++poseIt) {
            string poseName = poseIt->first;
            ArmPose poseValues = poseIt->second;

            //add new keyValuepair in the array
            key = output->mutable_entries()->Add();
            //set name of key
            key->set_key(poseName);
            key->mutable_value()->set_type(rst::generic::Value_Type_ARRAY);

            ROS_DEBUG_STREAM("Adding Pose %s" << poseName);

            //set the angle of the actions
            newVal = key->mutable_value()->mutable_array()->Add();
            newVal->set_type(rst::generic::Value_Type_INT);
            newVal->set_int_(poseValues.size());
            for (ArmPose::iterator f = poseValues.begin();
                    f != poseValues.end(); ++f) {
                newVal = key->mutable_value()->mutable_array()->Add();
                newVal->set_type(rst::generic::Value_Type_DOUBLE);
                newVal->set_double_(f->second);
            }
        }

        return output;
    }

    boost::shared_ptr<bool> setPose(boost::shared_ptr<string> input) {
        boost::shared_ptr<bool> success(new bool(false));
        ROS_DEBUG_STREAM("Invoked setPose");
        *success = listener->requestMoveTo(*input);
        return success;
    }
	
    boost::shared_ptr<bool> planToPose(boost::shared_ptr<string> input) {
        boost::shared_ptr<bool> success(new bool(false));
        ROS_DEBUG_STREAM("Invoked planToPose");
        *success = listener->requestPlanTo(*input);
        return success;
    }

    boost::shared_ptr<void> findObjects() {
       ROS_DEBUG_STREAM("Invoked findObjects");
       listener->requestFindObjects();
       return boost::shared_ptr<void>();
    }

    /**boost::shared_ptr<Dictionary> isObjectGraspable(
            boost::shared_ptr<BoundingBox3DFloat> input) {
        ObjectShape objectToGrasp;
        ROS_DEBUG_STREAM("Invoked isObjectGraspable");
        GraspReturnType grt = listener->requestGraspObject(convert(input),
                true);
        return convert(grt);
    }**/
    boost::shared_ptr<Dictionary> isObjectNameGraspable(
            boost::shared_ptr<string> input) {
        ROS_DEBUG_STREAM("Invoked isObjectGraspable");
        return graspObjectName(input, true);
    }

    /**boost::shared_ptr<Dictionary> graspObject(
            boost::shared_ptr<BoundingBox3DFloat> input) {
        ROS_DEBUG_STREAM("Invoked graspObject: " << input->DebugString());
        GraspReturnType grt = listener->requestGraspObject(convert(input),
                false);
        return convert(grt);
    }**/

    boost::shared_ptr<Dictionary> graspObjectName(
            boost::shared_ptr<string> input) {
        ROS_DEBUG_STREAM("Invoked graspObjectName: " << *input);
        return graspObjectName(input, false);
    }

    boost::shared_ptr<Dictionary> graspObjectName(
            boost::shared_ptr<string> input, bool sim) {
        vector<string> items;
        boost::algorithm::split(items, *input,
                boost::algorithm::is_any_of(";,"),
                boost::algorithm::token_compress_on);
        GraspReturnType grt;
        if (items.size() == 0) {
            grt = listener->requestGraspObject("", "surface0", sim);
        } else if (items.size() == 1) {
            grt = listener->requestGraspObject(items[0], "surface0", sim);
        } else {
            grt = listener->requestGraspObject(items[0], items[1], sim);
        }
        return convert(grt);
    }

    boost::shared_ptr<Dictionary> placeObject(
            boost::shared_ptr<rst::geometry::Pose> input) {
        ROS_DEBUG_STREAM("Invoked placeObject");
        GraspReturnType grt = listener->requestPlaceObject(convert(input),
                false);
        return convert(grt);
    }

    /**boost::shared_ptr<Dictionary> placeObjectInRegion(
            boost::shared_ptr<BoundingBox3DFloat> input) {
        ROS_DEBUG_STREAM("Invoked placeObject");
        GraspReturnType grt = listener->requestPlaceObject(convert(input),
                false);
        return convert(grt);
    }**/

    boost::shared_ptr<Dictionary> placeObjectOnSurface(
            boost::shared_ptr<string> input) {
        ROS_DEBUG_STREAM("Invoked placeObject");
        GraspReturnType grt = listener->requestPlaceObject(*input, false);
        return convert(grt);
    }

    boost::shared_ptr<Dictionary> isObjectPlaceable(
            boost::shared_ptr<rst::geometry::Pose> input) {
        ROS_DEBUG_STREAM("Invoked isObjectPlaceable");
        GraspReturnType grt = listener->requestPlaceObject(convert(input),
                true);
        return convert(grt);
    }

    boost::shared_ptr<string> echo(boost::shared_ptr<string> input) {
        ROS_DEBUG_STREAM("Echo: " << *input);
        return input;
    }

    boost::shared_ptr<Dictionary> convert(const GraspReturnType &grt) {
        boost::shared_ptr<rst::generic::Dictionary> output(
                new rst::generic::Dictionary());
        rst::generic::KeyValuePair* key;

        key = output->mutable_entries()->Add();
        key->set_key("x");
        key->mutable_value()->set_type(rst::generic::Value_Type_DOUBLE);
        key->mutable_value()->set_double_(grt.point.xMeter);

        key = output->mutable_entries()->Add();
        key->set_key("y");
        key->mutable_value()->set_type(rst::generic::Value_Type_DOUBLE);
        key->mutable_value()->set_double_(grt.point.yMeter);

        key = output->mutable_entries()->Add();
        key->set_key("z");
        key->mutable_value()->set_type(rst::generic::Value_Type_DOUBLE);
        key->mutable_value()->set_double_(grt.point.zMeter);

        key = output->mutable_entries()->Add();
        key->set_key("rating");
        key->mutable_value()->set_type(rst::generic::Value_Type_DOUBLE);
        key->mutable_value()->set_double_(grt.rating);

        key = output->mutable_entries()->Add();
        key->set_key("result");
        key->mutable_value()->set_type(rst::generic::Value_Type_STRING);
        key->mutable_value()->set_string(
                GraspReturnType::resultToString(grt.result));
        return output;
    }

    boost::shared_ptr<Dictionary> convert(const map<string, short> &readings) {

        boost::shared_ptr<Dictionary> output(new Dictionary());

        KeyValuePair* key = output->mutable_entries()->Add();
        key->set_key("TimeStamp");
        key->mutable_value()->set_type(rst::generic::Value_Type_INT);
        key->mutable_value()->set_int_(Timestamp::currentTimeMillis());

        map<string, short>::const_iterator i;
        for (i = readings.begin(); i != readings.end(); ++i) {
            key = output->mutable_entries()->Add();
            key->set_key(i->first);
            key->mutable_value()->set_type(rst::generic::Value_Type_INT);
            key->mutable_value()->set_int_(i->second);
            ROS_DEBUG_STREAM(i->first << " is " << i->second);
        }
        return output;
    }
    EefPose convert(boost::shared_ptr<rst::geometry::Pose> input) {
        EefPose pose;
        pose.translation.xMeter = input->translation().x();
        pose.translation.yMeter = input->translation().y();
        pose.translation.zMeter = input->translation().z();
        pose.translation.frame = input->translation().frame_id();
        pose.rotation.qw = input->rotation().qw();
        pose.rotation.qx = input->rotation().qx();
        pose.rotation.qy = input->rotation().qy();
        pose.rotation.qz = input->rotation().qz();
        pose.rotation.frame = input->rotation().frame_id();
        pose.frame = input->translation().frame_id();
        ROS_DEBUG_STREAM("frame pose: " << pose.frame);
        return pose;
    }

    boost::shared_ptr<Pose> convert(const EefPose &input) {
        boost::shared_ptr<rst::geometry::Pose> retPose(
                new rst::geometry::Pose());
        retPose->mutable_translation()->set_x(input.translation.xMeter);
        retPose->mutable_translation()->set_y(input.translation.yMeter);
        retPose->mutable_translation()->set_z(input.translation.zMeter);
        retPose->mutable_translation()->set_frame_id(input.frame.c_str());
        retPose->mutable_rotation()->set_qw(input.rotation.qw);
        retPose->mutable_rotation()->set_qx(input.rotation.qx);
        retPose->mutable_rotation()->set_qy(input.rotation.qy);
        retPose->mutable_rotation()->set_qz(input.rotation.qz);
        retPose->mutable_rotation()->set_frame_id(input.frame.c_str());
        return retPose;
    }

    /**ObjectShape convert(boost::shared_ptr<BoundingBox3DFloat> input) {
        ObjectShape objectToGrasp;
        objectToGrasp.widthMeter = input->width();
        objectToGrasp.heightMeter = input->height();
        objectToGrasp.depthMeter = input->depth();
        objectToGrasp.center.xMeter = input->transformation().translation().x();
        objectToGrasp.center.yMeter = input->transformation().translation().y();
        objectToGrasp.center.zMeter = input->transformation().translation().z();
        objectToGrasp.center.frame =
                input->transformation().translation().frame_id();
        return objectToGrasp;
    }**/
};

RsbInterface::RsbInterface(const string &serverScope) :
        serverScope(serverScope), d(new Private()) {
    init();
}

RsbInterface::~RsbInterface() {
}

void RsbInterface::setListener(ControlInterfaceListener* listener) {
    d->listener = listener;
}

void RsbInterface::removeListener() {
    d->listener = new EmptyControlInterfaceListener();
}

#define CREATE_CALLBACK_0(ReplyType, Function) \
	LocalServer::CallbackPtr(new FunctionCallback<void, ReplyType>(boost::bind(&Private::Function, d.get())))

#define CREATE_CALLBACK_1(RequestType, ReplyType, Function) \
	LocalServer::CallbackPtr(new FunctionCallback<RequestType, ReplyType>(boost::bind(&Private::Function, d.get(), _1)))

#define CREATE_PB_CONVERTER(Type) \
	boost::shared_ptr<rsb::converter::ProtocolBufferConverter<Type> >( \
		new rsb::converter::ProtocolBufferConverter<Type>())



//class VoidVoidCallback: public LocalServer::Callback<void, void> {
//    void call(const std::string& /*methodName*/) {
//        std::cout << "void-void method called" << std::endl;
//    }
//};


void RsbInterface::init() {

    ROS_DEBUG_STREAM("registering methods");

    // add converters
    converterRepository<string>()->registerConverter(
            CREATE_PB_CONVERTER(rst::geometry::Pose));
    converterRepository<string>()->registerConverter(
            CREATE_PB_CONVERTER(JointAngles));
    converterRepository<string>()->registerConverter(
            CREATE_PB_CONVERTER(Dictionary));
    converterRepository<string>()->registerConverter(
            CREATE_PB_CONVERTER(BoundingBox3DFloat));

    rsc::misc::initSignalWaiter();
    
    Factory& factory = getFactory();
    d->server = factory.createLocalServer(serverScope);

    
    //d->server->registerMethod("closeGripper", LocalServer::CallbackPtr(new VoidVoidCallback()));
    
    d->server->registerMethod("closeGripper",
            CREATE_CALLBACK_0(void, closeGripper));
    d->server->registerMethod("listAngles",
            CREATE_CALLBACK_0(JointAngles, listAngles));
    d->server->registerMethod("moveJoints",
            CREATE_CALLBACK_1(JointAngles, void, moveJoints));
    //d->server->registerMethod("goto", CREATE_CALLBACK(string, string, echo));
    d->server->registerMethod("gotoLinear",
            CREATE_CALLBACK_1(rst::geometry::Pose, bool, goToLinear));
    d->server->registerMethod("gotoNonLinear",
            CREATE_CALLBACK_1(rst::geometry::Pose, bool, goToNonLinear));
    d->server->registerMethod("gotoLinearOrient",
            CREATE_CALLBACK_1(rst::geometry::Pose, bool, goToLinearOrient));
    d->server->registerMethod("gotoNonLinearOrient",
            CREATE_CALLBACK_1(rst::geometry::Pose, bool, goToNonLinearOrient));
    d->server->registerMethod("getPosition",
            CREATE_CALLBACK_0(rst::geometry::Pose, getPosition));
    d->server->registerMethod("findNearestPose",
            CREATE_CALLBACK_0(string, findNearestPose));
    //d->server->registerMethod("listActions", CREATE_CALLBACK(string, string, echo));
    d->server->registerMethod("listPoses",
            CREATE_CALLBACK_0(Dictionary, listPoses));
    d->server->registerMethod("setMovement",
            CREATE_CALLBACK_1(string, bool, setPose));
    d->server->registerMethod("planToPose",
            CREATE_CALLBACK_1(string, bool, planToPose));
    d->server->registerMethod("setPose",
            CREATE_CALLBACK_1(string, bool, setPose));
    //d->server->registerMethod("setCarryMovement", CREATE_CALLBACK(string, string, echo));
    d->server->registerMethod("openGripper",
            CREATE_CALLBACK_0(void, openGripper));
    d->server->registerMethod("openGripperWhenTouching",
            CREATE_CALLBACK_0(void, openGripperWhenTouching));
    d->server->registerMethod("closeGripperByForce",
            CREATE_CALLBACK_0(void, closeGripperByForce));
    d->server->registerMethod("motorsOff", CREATE_CALLBACK_0(void, motorsOff));
    d->server->registerMethod("motorsOn", CREATE_CALLBACK_0(void, motorsOn));
    //d->server->registerMethod("freeze", CREATE_CALLBACK(string, string, echo));
    //d->server->registerMethod("unblock", CREATE_CALLBACK(string, string, echo));
    d->server->registerMethod("getGripperSensors",
            CREATE_CALLBACK_0(Dictionary, getGripperSensors));
    d->server->registerMethod("isSomethingInGripper",
            CREATE_CALLBACK_0(bool, isSomethingInGripper));
    //d->server->registerMethod("calculateGraspablePose", CREATE_CALLBACK(string, string, echo));
    //d->server->registerMethod("setObstacles", CREATE_CALLBACK(string, string, echo));
    d->server->registerMethod("findObjects",
            CREATE_CALLBACK_0(void, findObjects));
    //d->server->registerMethod("isObjectGraspable",
    //        CREATE_CALLBACK_1(BoundingBox3DFloat, Dictionary,
    //                isObjectGraspable));
    d->server->registerMethod("isObjectNameGraspable",
            CREATE_CALLBACK_1(string, Dictionary, isObjectNameGraspable));
    //d->server->registerMethod("graspObject",
    //        CREATE_CALLBACK_1(BoundingBox3DFloat, Dictionary, graspObject));
    d->server->registerMethod("graspObjectName",
            CREATE_CALLBACK_1(string, Dictionary, graspObjectName));
    //d->server->registerMethod("graspObjectOrientation", CREATE_CALLBACK(string, string, echo));
    d->server->registerMethod("placeObjectAt",
            CREATE_CALLBACK_1(rst::geometry::Pose, Dictionary, placeObject));
    //d->server->registerMethod("placeObjectInRegion",
    //        CREATE_CALLBACK_1(BoundingBox3DFloat, Dictionary,
    //                placeObjectInRegion));
    d->server->registerMethod("placeObjectOnSurface",
            CREATE_CALLBACK_1(string, Dictionary, placeObjectOnSurface));
    //d->server->registerMethod("placeObjectAtExact", CREATE_CALLBACK(string, string, echo));
    d->server->registerMethod("isPlaceable",
            CREATE_CALLBACK_1(rst::geometry::Pose, Dictionary,
                    isObjectPlaceable));
    //d->server->registerMethod("wipingMovement", CREATE_CALLBACK(string, string, echo));
    //d->server->registerMethod("joggingMovement", CREATE_CALLBACK(string, string, echo));
    //d->server->registerMethod("findLeverAndMoveDown", CREATE_CALLBACK(string, string, echo));

}
