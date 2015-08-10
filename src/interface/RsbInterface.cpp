/*
 * RsbInterface.cpp
 *
 *  Created on: Apr 29, 2015
 *      Author: lziegler
 */

#include "RsbInterface.h"
#include "../model/ModelTypes.h"
#include <boost/bind.hpp>
#include <ros/ros.h>

#define BOOST_SIGNALS_NO_DEPRECATION_WARNING
#include <rsc/logging/Logger.h>
#include <rsb/Factory.h>
#include <rsb/patterns/LocalServer.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsb/converter/Repository.h>

#include <rst/kinematics/JointAngles.pb.h>
#include <rst/geometry/Pose.pb.h>
#include <rst/geometry/BoundingBox3DFloat.pb.h>
#include <rst/generic/Dictionary.pb.h>
#include <boost/smart_ptr/make_shared_object.hpp>

using namespace std;
using namespace boost;

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
			typename boost::shared_ptr<ReplyType>(typename boost::shared_ptr<RequestType>)> FunctionType;

	explicit FunctionCallback(FunctionType function, const std::string& requestType =
			rsc::runtime::typeName(typeid(RequestType)), const std::string& replyType =
			rsc::runtime::typeName(typeid(ReplyType))) :
			CallbackBase(requestType, replyType), function(function) {
	}

private:
	EventPtr intlCall(const std::string& /*methodName*/, EventPtr request) {
		boost::shared_ptr<RequestType> argument = boost::static_pointer_cast<RequestType>(
				request->getData());
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

	static rsc::logging::LoggerPtr logger;
	LocalServerPtr server;
	ControlInterfaceListener* listener;

	shared_ptr<JointAngles> listAngles() {
		RSCDEBUG(logger, "Invoked ListAngles");
		vector<double> joints = listener->requestJointAngles();

		shared_ptr<JointAngles> angles(new JointAngles());
		for (int i = 0; i < joints.size(); ++i) {
			angles->add_angles(joints[i]);
			RSCDEBUG(logger, "joint " << i << " with value " << joints[i]);
		}
		return angles;
	}

	shared_ptr<void> moveJoints(shared_ptr<JointAngles> input) {
		RSCDEBUG(logger, "Invoked moveJoints");
		if (listener->requestNumJoints() != input->angles_size()) {
			RSCERROR(logger,
					"Size in MoveJoints: " << input->angles_size() << "does not match armSize: " << listener->requestNumJoints());
		} else {
			vector<double> angles;
			for (int i = 0; i < input->angles_size(); i++) {
				//model->setJointAngle(i, (double) input->angles(i));
				angles.push_back((double) input->angles(i));
			}
			listener->requestMoveJointAngles(angles);
		}
		return shared_ptr<void>();
	}

	shared_ptr<bool> goTo(shared_ptr<Pose> input, bool linear, bool orientation) {
		shared_ptr<bool> sucess(new bool(true));
		RSCDEBUG(logger, "Invoked gotToLinear");
		try {
			*sucess = listener->requestMoveTo(convert(input), linear, orientation);
		} catch (std::exception& e) {
			*sucess = false;
			RSCERROR(logger, e.what());
			return sucess;
		}
		return sucess;
	}

	shared_ptr<bool> goToLinear(shared_ptr<Pose> input) {
		RSCDEBUG(logger, "Invoked gotToLinear");
		return goTo(input, true, false);
	}

	shared_ptr<bool> goToNonLinear(shared_ptr<Pose> input) {
		RSCDEBUG(logger, "Invoked gotToNonLinear");
		return goTo(input, false, false);
	}

	shared_ptr<bool> goToLinearOrient(shared_ptr<Pose> input) {
		RSCDEBUG(logger, "Invoked gotToLinearOrient");
		return goTo(input, true, true);
	}

	shared_ptr<bool> goToNonLinearOrient(shared_ptr<Pose> input) {
		RSCDEBUG(logger, "Invoked gotToNonLinearOrient");
		return goTo(input, false, true);
	}

	shared_ptr<Pose> getPosition() {
		RSCDEBUG(logger, "Invoked getPosition");
		EefPose position = listener->requestEefPose();
		shared_ptr<Pose> poseOut = convert(position);
		RSCDEBUG(logger, "Received following pose: " << poseOut->DebugString());
		return poseOut;
	}

	shared_ptr<void> openGripper() {
		RSCDEBUG(logger, "Invoked openGripper");
		listener->requestOpenGripper(false);
		return shared_ptr<void>();
	}

	shared_ptr<void> openGripperWhenTouching() {
		RSCDEBUG(logger, "Invoked openGripperWhenTouching");
		listener->requestOpenGripper(true);
		return shared_ptr<void>();
	}
        
        
        shared_ptr<std::string> findNearestPose() {
		RSCDEBUG(logger, "Invoked findNearestPose");
		std::string ret = listener->requestNearestPose();
		return boost::make_shared<std::string>(ret.c_str());
	}
        
	shared_ptr<void> closeGripper() {
		RSCDEBUG(logger, "Invoked closeGripper");
		listener->requestCloseGripper(false);
		return shared_ptr<void>();
	}

	shared_ptr<void> closeGripperByForce() {
		RSCDEBUG(logger, "Invoked closeGripperByForce");
		listener->requestCloseGripper(true);
		return shared_ptr<void>();
	}
        
        

	shared_ptr<void> motorsOn() {
		RSCDEBUG(logger, "Invoked motorsOn");
		listener->requestMotorsOn();
		return shared_ptr<void>();
	}
	shared_ptr<void> motorsOff() {
		RSCDEBUG(logger, "Invoked motorsOff");
		listener->requestMotorsOff();
		return shared_ptr<void>();
	}

	shared_ptr<bool> isSomethingInGripper() {
		shared_ptr<bool> success(new bool(true));
		RSCDEBUG(logger, "Invoked isSomethingInGripper");
		if (!listener->requestIsSomethingInGripper()) {
			*success = false;
		}
		return success;
	}

	shared_ptr<Dictionary> getGripperSensors() {
		RSCDEBUG(logger, "Invoked getGripperSensors");
		map<string, short> sensorValues = listener->requestGripperSensors();
		return convert(sensorValues);
	}

	shared_ptr<Dictionary> listPoses() {
		RSCDEBUG(logger, "Invoked listPoses");

		shared_ptr<Dictionary> output(new rst::generic::Dictionary());

		rst::generic::Value* newVal;
		KeyValuePair* key;

		Poses poses = listener->requestPoses();

		for (Poses::iterator poseIt = poses.begin(); poseIt != poses.end(); ++poseIt) {
			string poseName = poseIt->first;
			vector<double> poseValues = poseIt->second;

			//add new keyValuepair in the array
			key = output->mutable_entries()->Add();
			//set name of key
			key->set_key(poseName);
			key->mutable_value()->set_type(rst::generic::Value_Type_ARRAY);

			RSCDEBUG(logger, "Adding Pose %s" << poseName);

			//set the angle of the actions
			newVal = key->mutable_value()->mutable_array()->Add();
			newVal->set_type(rst::generic::Value_Type_INT);
			newVal->set_int_(poseValues.size());
			for (int f = 0; f < poseValues.size(); f++) {
				newVal = key->mutable_value()->mutable_array()->Add();
				newVal->set_type(rst::generic::Value_Type_DOUBLE);
				newVal->set_double_(poseValues[f]);
			}
		}

		return output;
	}

	shared_ptr<bool> setPose(shared_ptr<string> input) {
		shared_ptr<bool> success(new bool(false));
		RSCDEBUG(logger, "Invoked setPose");
		*success = listener->requestMoveTo(*input);
		return success;
	}

	shared_ptr<Dictionary> isObjectGraspable(shared_ptr<BoundingBox3DFloat> input) {
		ObjectShape objectToGrasp;
		RSCDEBUG(logger, "Invoked isObjectGraspable");
		GraspReturnType grt = listener->requestGraspObject(convert(input), true);
		return convert(grt);
	}

	shared_ptr<Dictionary> graspObject(shared_ptr<BoundingBox3DFloat> input) {
		RSCDEBUG(logger, "Invoked graspObject: " << input->DebugString());
		GraspReturnType grt = listener->requestGraspObject(convert(input), false);
		return convert(grt);
	}
	shared_ptr<Dictionary> placeObject(shared_ptr<Pose> input) {
		RSCDEBUG(logger, "Invoked placeObject");
		GraspReturnType grt = listener->requestPlaceObject(convert(input), false);
		return convert(grt);
	}
	shared_ptr<Dictionary> placeObjectInRegion(shared_ptr<BoundingBox3DFloat> input) {
		RSCDEBUG(logger, "Invoked placeObject");
		GraspReturnType grt = listener->requestPlaceObject(convert(input), false);
		return convert(grt);
	}
	shared_ptr<Dictionary> isObjectPlaceable(shared_ptr<Pose> input) {
		RSCDEBUG(logger, "Invoked isObjectPlaceable");
		GraspReturnType grt = listener->requestPlaceObject(convert(input), true);
		return convert(grt);
	}
	shared_ptr<string> echo(shared_ptr<string> input) {
		RSCDEBUG(logger, "Echo: " << *input);
		return input;
	}

	shared_ptr<Dictionary> convert(const GraspReturnType &grt) {
		boost::shared_ptr<rst::generic::Dictionary> output(new rst::generic::Dictionary());
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
		key->mutable_value()->set_string(GraspReturnType::resultToString(grt.result));
		return output;
	}

	shared_ptr<Dictionary> convert(const map<string, short> &readings) {

		shared_ptr<Dictionary> output(new Dictionary());

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
			RSCDEBUG(logger, i->first << " is " << i->second);
		}
		return output;
	}
	EefPose convert(shared_ptr<Pose> input) {
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
		RSCDEBUG(logger, "frame pose: " << pose.frame);
		return pose;
	}

	shared_ptr<Pose> convert(const EefPose &input) {
		boost::shared_ptr<Pose> retPose(new Pose());
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

	ObjectShape convert(shared_ptr<BoundingBox3DFloat> input) {
		ObjectShape objectToGrasp;
		objectToGrasp.widthMeter = input->width();
		objectToGrasp.heightMeter = input->height();
		objectToGrasp.depthMeter = input->depth();
		objectToGrasp.center.xMeter = input->transformation().translation().x();
		objectToGrasp.center.yMeter = input->transformation().translation().y();
		objectToGrasp.center.zMeter = input->transformation().translation().z();
		objectToGrasp.center.frame = input->transformation().translation().frame_id();
		return objectToGrasp;
	}
};

rsc::logging::LoggerPtr RsbInterface::Private::logger = rsc::logging::Logger::getLogger(
		"picknplace.RsbInterface");

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

void RsbInterface::init() {

	RSCDEBUG(d->logger, "registering methods");

	// add converters
	converterRepository<string>()->registerConverter(CREATE_PB_CONVERTER(Pose));
	converterRepository<string>()->registerConverter(CREATE_PB_CONVERTER(JointAngles));
	converterRepository<string>()->registerConverter(CREATE_PB_CONVERTER(Dictionary));
	converterRepository<string>()->registerConverter(CREATE_PB_CONVERTER(BoundingBox3DFloat));

	Factory& factory = getFactory();
	d->server = factory.createLocalServer(serverScope);

	d->server->registerMethod("listAngles", CREATE_CALLBACK_0(JointAngles, listAngles));
	d->server->registerMethod("moveJoints", CREATE_CALLBACK_1(JointAngles, void, moveJoints));
	//d->server->registerMethod("goto", CREATE_CALLBACK(string, string, echo));
	d->server->registerMethod("gotoLinear", CREATE_CALLBACK_1(Pose, bool, goToLinear));
	d->server->registerMethod("gotoNonLinear", CREATE_CALLBACK_1(Pose, bool, goToNonLinear));
	d->server->registerMethod("gotoLinearOrient", CREATE_CALLBACK_1(Pose, bool, goToLinearOrient));
	d->server->registerMethod("gotoNonLinearOrient", CREATE_CALLBACK_1(Pose, bool, goToNonLinearOrient));
	d->server->registerMethod("getPosition", CREATE_CALLBACK_0(Pose, getPosition));
	d->server->registerMethod("findNearestPose", CREATE_CALLBACK_0(string, findNearestPose));
	//d->server->registerMethod("listActions", CREATE_CALLBACK(string, string, echo));
	d->server->registerMethod("listPoses", CREATE_CALLBACK_0(Dictionary, listPoses));
	d->server->registerMethod("setMovement", CREATE_CALLBACK_1(string, bool, setPose));
	d->server->registerMethod("setPose", CREATE_CALLBACK_1(string, bool, setPose));
	//d->server->registerMethod("setCarryMovement", CREATE_CALLBACK(string, string, echo));
	d->server->registerMethod("openGripper", CREATE_CALLBACK_0(void, openGripper));
	d->server->registerMethod("openGripperWhenTouching",
			CREATE_CALLBACK_0(void, openGripperWhenTouching));
	d->server->registerMethod("closeGripper", CREATE_CALLBACK_0(void, closeGripper));
	d->server->registerMethod("closeGripperByForce", CREATE_CALLBACK_0(void, closeGripperByForce));
	d->server->registerMethod("motorsOff", CREATE_CALLBACK_0(void, motorsOff));
	d->server->registerMethod("motorsOn", CREATE_CALLBACK_0(void, motorsOn));
	//d->server->registerMethod("freeze", CREATE_CALLBACK(string, string, echo));
	//d->server->registerMethod("unblock", CREATE_CALLBACK(string, string, echo));
	d->server->registerMethod("getGripperSensors", CREATE_CALLBACK_0(Dictionary, getGripperSensors));
	d->server->registerMethod("isSomethingInGripper", CREATE_CALLBACK_0(bool, isSomethingInGripper));
	//d->server->registerMethod("calculateGraspablePose", CREATE_CALLBACK(string, string, echo));
	//d->server->registerMethod("setObstacles", CREATE_CALLBACK(string, string, echo));
	d->server->registerMethod("isObjectGraspable", CREATE_CALLBACK_1(BoundingBox3DFloat, Dictionary, isObjectGraspable));
	d->server->registerMethod("graspObject", CREATE_CALLBACK_1(BoundingBox3DFloat, Dictionary, graspObject));
	//d->server->registerMethod("graspObjectOrientation", CREATE_CALLBACK(string, string, echo));
	d->server->registerMethod("placeObjectAt", CREATE_CALLBACK_1(Pose, Dictionary, placeObject));
	d->server->registerMethod("placeObjectInRegion", CREATE_CALLBACK_1(BoundingBox3DFloat, Dictionary, placeObjectInRegion));
	//d->server->registerMethod("placeObjectAtExact", CREATE_CALLBACK(string, string, echo));
	d->server->registerMethod("isPlaceable", CREATE_CALLBACK_1(Pose, Dictionary, isObjectPlaceable));
	//d->server->registerMethod("wipingMovement", CREATE_CALLBACK(string, string, echo));
	//d->server->registerMethod("joggingMovement", CREATE_CALLBACK(string, string, echo));
	//d->server->registerMethod("findLeverAndMoveDown", CREATE_CALLBACK(string, string, echo));

}
