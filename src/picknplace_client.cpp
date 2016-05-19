/* 
 * File:   RemoteRsbClient.cpp
 * Author: mholland
 * 
 * Created on April 25, 2014, 4:50 PM
 */
#include <rst/kinematics/JointAngles.pb.h>
#include <rst/geometry/Pose.pb.h>
#include <rst/geometry/Translation.pb.h>
#include <rst/geometry/Rotation.pb.h>
#include <rst/geometry/Shape3DFloat.pb.h>
#include <rst/geometry/PointCloud3DFloat.pb.h>
#include <rst/math/Vec3DDouble.pb.h>
#include <rst/geometry/BoundingBox3DFloat.pb.h>
#include <rst/geometry/PointCloud3DFloat.pb.h>
#include <rst/generic/Dictionary.pb.h>

#include <boost/program_options.hpp>
#include <boost/lexical_cast.hpp>

#define BOOST_SIGNALS_NO_DEPRECATION_WARNING
#include <rsb/Factory.h>
#include <rsb/patterns/RemoteServer.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsb/Exception.h>

#include <rsc/misc/SignalWaiter.h>

using namespace rsb;
using namespace rsb::patterns;
using namespace boost::program_options;
using namespace rst::kinematics;
using namespace rst::geometry;
using namespace std;

options_description desc("Allowed options");
variables_map vm;
rsb::patterns::RemoteServerPtr remoteServer;

void callServerMethod(int argc, char* argv[]) {

    try {
        //handle cmdline args with boost::programoptions
        desc.add_options()
                ("help", "shows help message")

                ("getPosition", "returns the position of the arms endEffector")
				
                ("listAngles", "returns the current angles of the KatanArms joints")

                ("setMovement", "Move the arm to a predefined Position")
                ("setPose", "Move the arm to a predefined Pose")
				("planToPose", "Move the arm to a predifined position (with planning)")

                ("moveJoints", "moves the arm to the given joint angles\nargs: m0 m1 m2 m3 m4 eef")
                ("gotoLinear", "moves the end effector to pose. Translation in m, quaternion in radiant\n"
                "args: x y z [qw qx qy qz]")
                ("gotoNonLinear", "moves the end effector to pose. Translation in m, quaternion in radiant\n"
                "args: x y z [qw qx qy qz]")

                ("motorsOff", "deactivates the motors. Attention the arm will fall down!")
                ("motorsOn", "activates the motors.")

                ("openGripper", "opens the gripper")
                ("openGripperWhenTouching", "the gripper opens when someone is touching it")
                ("closeGripper", "closes the Gripper")
                ("closeGripperByForce", "may the force be with you!")
                ("getGripperSensors", "prints the IR values of the grippers IR Sensors")
                ("isSomethingInGripper", "returns true if something is in the gripper")
                ("findNearestPose", "no arguments needed; seems to return string with nearest pose (description has to be revised)")
				("listPoses", "list available poses")

                ("isObjectGraspable", "returns if the object is graspable")
                ("graspObject", "tries to grasp the Object specified through a BoundingBox\nargs:w h d x y z")
                ("graspObjectOrientation", "tries to grasp the Object specified through a BoundingBox. Trys to grasp from the top\nargs:w h d x y z")
                ("placeObjectAt", "places the object at the position given\n"
                "args: x y z phi theta psi")
                ("placeObjectAtExact", "places the object at the position given\n"
                "args: x y z phi theta psi")
		("graspObjectName", "grasp the Object by Name\n"
		"args: nothing, object or object;surface" )
		("findObjects", "returns graspable Object");

        //store(parse_command_line(argc, argv, desc), vm);

        store(command_line_parser(argc, argv).options(desc).style(
                command_line_style::unix_style ^ command_line_style::allow_short
                ).run(), vm);
        notify(vm);
    } catch (boost::program_options::error &e) {
        std::cerr << e.what() << " use --help" << std::endl;
    }

    if (vm.count("help")) {
        std::cout << desc << "\n";

    } else if (vm.count("listAngles")) {
        boost::shared_ptr< void > request;
        boost::shared_ptr< JointAngles > result
                = remoteServer->call< JointAngles >("listAngles", request);

        std::cout << "listAngles:\n" << result->DebugString() << std::endl;

    } else if (vm.count("moveJoints")) {
        if (argc != 6 + 2) {
            std::cout << "Too many/few Joints entered! use --help for mor information! argc = " << argc << std::endl;
        } else {
            boost::shared_ptr<JointAngles> angles(JointAngles::default_instance().New());
            float tmp;
            for (int i = 2; i < argc; i++) {

                tmp = boost::lexical_cast< float >(argv[i]);
                angles->add_angles(tmp);
            }
            std::cout << angles->DebugString() << std::endl;
            boost::shared_ptr< void > result;
            result = remoteServer->call< void >("moveJoints", angles);
        }

    } else if (vm.count("gotoLinear")) {
        if (argc != 7 + 2 && argc != 3 + 2) {
            std::cout << "Too many/few coordinates/joints entered! use --help for more information " << argc << std::endl;
        } else {
            boost::shared_ptr< Pose > pose(Pose::default_instance().New());

            pose->mutable_translation()->set_x(boost::lexical_cast< double >(argv[2]));
            pose->mutable_translation()->set_y(boost::lexical_cast< double >(argv[3]));
            pose->mutable_translation()->set_z(boost::lexical_cast< double >(argv[4]));

            boost::shared_ptr< bool > result;
            if (argc == 7 + 2) {
				pose->mutable_rotation()->set_qw(boost::lexical_cast< double >(argv[5]));
				pose->mutable_rotation()->set_qx(boost::lexical_cast< double >(argv[6]));
				pose->mutable_rotation()->set_qy(boost::lexical_cast< double >(argv[7]));
				pose->mutable_rotation()->set_qz(boost::lexical_cast< double >(argv[8]));
				result = remoteServer->call< bool >("gotoLinearOrient", pose);
            } else if (argc == 3 + 2) {
            	pose->mutable_rotation()->set_qw(0.0);
				pose->mutable_rotation()->set_qx(0.0);
				pose->mutable_rotation()->set_qy(0.0);
				pose->mutable_rotation()->set_qz(0.0);
				result = remoteServer->call< bool >("gotoLinear", pose);
            }
            if (*result == true) {
                std::cout << "Moved gripper to position:\n" << pose->DebugString() << std::endl;
            } else {
                std::cout << "Failed to move gripper to position:\n" << pose->DebugString() << std::endl;
            }
        }

    } else if (vm.count("gotoNonLinear")) {
        if (argc != 7 + 2) {
            std::cout << "Too many/few coordinates/joints entered! use --help for more information" << std::endl;
        } else {
            boost::shared_ptr< Pose > pose(Pose::default_instance().New());

            pose->mutable_translation()->set_x(boost::lexical_cast< double >(argv[2]));
            pose->mutable_translation()->set_y(boost::lexical_cast< double >(argv[3]));
            pose->mutable_translation()->set_z(boost::lexical_cast< double >(argv[4]));

            boost::shared_ptr< bool > result;
			if (argc == 7 + 2) {
				pose->mutable_rotation()->set_qw(boost::lexical_cast< double >(argv[5]));
				pose->mutable_rotation()->set_qx(boost::lexical_cast< double >(argv[6]));
				pose->mutable_rotation()->set_qy(boost::lexical_cast< double >(argv[7]));
				pose->mutable_rotation()->set_qz(boost::lexical_cast< double >(argv[8]));
				result = remoteServer->call< bool >("gotoNonLinearOrient", pose);
			} else if (argc == 3 + 2) {
				pose->mutable_rotation()->set_qw(0.0);
				pose->mutable_rotation()->set_qx(0.0);
				pose->mutable_rotation()->set_qy(0.0);
				pose->mutable_rotation()->set_qz(0.0);
				result = remoteServer->call< bool >("gotoNonLinear", pose);
			}
            if (*result == true) {
                std::cout << "Moved gripper to position:\n" << pose->DebugString() << std::endl;
            } else {
                std::cout << "Failed to move gripper to position:\n" << pose->DebugString() << std::endl;
            }
        }

    } else if (vm.count("getPosition")) {
        boost::shared_ptr< void > request;
        boost::shared_ptr< Pose > result = remoteServer->call< Pose >("getPosition", request);

        double qw = result->mutable_rotation()->qw();
        double qx = result->mutable_rotation()->qx();
        double qy = result->mutable_rotation()->qy();
        double qz = result->mutable_rotation()->qz();

        // x,y,z are given in m
        double x = result->mutable_translation()->x();
        double y = result->mutable_translation()->y();
        double z = result->mutable_translation()->z();

        std::cout << "Current Position : translation m: x = " << x << " y = " <<
                y << " z = " << z << "\n rotation rad: qw=" << qw << " qx=" << qx <<" qy=" << qy <<" qz=" << qz << std::endl;

    } else if (vm.count("motorsOff")) {
        boost::shared_ptr< void > request;
        boost::shared_ptr< void > result = remoteServer->call< void >("motorsOff", request);

    } else if (vm.count("motorsOn")) {
        boost::shared_ptr< void > request;
        boost::shared_ptr< void > result = remoteServer->call< void >("motorsOn", request);

    } else if (vm.count("setMovement")) {

        boost::shared_ptr< std::string > request(new std::string(argv[2]));

        boost::shared_ptr< bool > result = remoteServer->call< bool >("setMovement", request);
        if (*result == true) {
            std::cout << "setMovment success!" << std::endl;
        } else {
            std::cout << "set Movement failed. Check Arguments" << std::endl;
        }
    } else if (vm.count("setPose")) {

        boost::shared_ptr< std::string > request(new std::string(argv[2]));

        boost::shared_ptr< bool > result = remoteServer->call< bool >("setPose", request);
        if (*result == true) {
            std::cout << "setPose success!" << std::endl;
        } else {
            std::cout << "set Pose failed. Check Arguments" << std::endl;
        }

    } else if (vm.count("planToPose")) {
        boost::shared_ptr< std::string > request(new std::string(argv[2]));
        boost::shared_ptr< bool > result = remoteServer->call< bool >("planToPose", request);
        if (*result == true) {
            std::cout << "plan to pose success!" << std::endl;
        } else {
            std::cout << "plan to Pose failed. Check Arguments" << std::endl;
        }
		
	} else if (vm.count("findNearestPose")) {

        boost::shared_ptr< void > request;
        boost::shared_ptr< std::string > result = remoteServer->call< std::string >("findNearestPose", request);
    } else if (vm.count("listPoses")) {
    	boost::shared_ptr< void > request;
		boost::shared_ptr< rst::generic::Dictionary > result = remoteServer->call< rst::generic::Dictionary >("listPoses", request);

		std::cout << "list Poses: \n";
		for (int i = 0; i < result->entries_size(); i++) {
			std::cout << *result->mutable_entries(i)->mutable_key() << "\n";
		}

    }else if (vm.count("openGripper")) {
        boost::shared_ptr< void > request;
        boost::shared_ptr< void > result = remoteServer->call< void >("openGripper", request);

    } else if (vm.count("closeGripper")) {
        boost::shared_ptr< void > request;
        boost::shared_ptr< void > result = remoteServer->call< void >("closeGripper", request);

    } else if (vm.count("openGripperWhenTouching")) {
        boost::shared_ptr< void > request;
        boost::shared_ptr< void > result = remoteServer->call< void >("openGripperWhenTouching", request);

    } else if (vm.count("closeGripperByForce")) {
        boost::shared_ptr< void > request;
        boost::shared_ptr< void > result = remoteServer->call< void >("closeGripperByForce", request);

    } else if (vm.count("getGripperSensors")) {
        boost::shared_ptr< void > request;
        boost::shared_ptr< rst::generic::Dictionary > result = remoteServer->call< rst::generic::Dictionary >("getGripperSensors", request);
        int value;

        std::cout << "IR Sensors: \n";
        for (int i = 0; i < result->entries_size(); i++) {
            value = (int) result->mutable_entries(i)->mutable_value()->int_();
            std::cout << *result->mutable_entries(i)->mutable_key() << ": " << value << std::endl;
        }

    } else if (vm.count("isSomethingInGripper")) {
        boost::shared_ptr< void > request;
        boost::shared_ptr< bool > result = remoteServer->call< bool >("isSomethingInGripper", request);
        if (*result == true) {
            std::cout << "Something is in the Gripper!" << std::endl;
        } else {
            std::cout << "Nothing in the Gripper!" << std::endl;
        }
    } else if (vm.count("isObjectGraspable")) {
        if (argc != 8) {
            std::cout << "Too many/few coordinates entered-6 (width,height,depth,x,y,z) are expected! use --help for more information " << argc << std::endl;
        } else {

            boost::shared_ptr< rst::geometry::BoundingBox3DFloat> object(new rst::geometry::BoundingBox3DFloat);

            object->set_width(boost::lexical_cast< float >(argv[2]));
            object->set_height(boost::lexical_cast< float >(argv[3]));
            object->set_depth(boost::lexical_cast< float >(argv[4]));

            object->mutable_transformation()->mutable_rotation()->set_qw(0.0);
            object->mutable_transformation()->mutable_rotation()->set_qx(0.0);
            object->mutable_transformation()->mutable_rotation()->set_qy(0.0);
            object->mutable_transformation()->mutable_rotation()->set_qz(0.0);

            object->mutable_transformation()->mutable_translation()->set_x(boost::lexical_cast<double>(argv[5]));
            object->mutable_transformation()->mutable_translation()->set_y(boost::lexical_cast<double>(argv[6]));
            object->mutable_transformation()->mutable_translation()->set_z(boost::lexical_cast<double>(argv[7]));

            boost::shared_ptr<rst::generic::Dictionary> result;
            result = remoteServer->call<rst::generic::Dictionary>("isObjectGraspable", object);
            std::cout << "GraspReturnType: \n" << result->DebugString() << std::endl;
        }
    } else if (vm.count("graspObjectName")){
        if (argc >= 4) {
            std::cout << "Too many/few arguments entered nothing, object or object;surface is expected! use --help for more information " << argc << std::endl;
        } else {
	    boost::shared_ptr< std::string > request(new std::string(argv[2]));
            boost::shared_ptr<rst::generic::Dictionary> result;
            result = remoteServer->call<rst::generic::Dictionary>("graspObjectName", request);
            std::cout << "GraspReturnType: \n" << result->DebugString() << std::endl;
        }
    } else if (vm.count("findObjects")) {
        boost::shared_ptr< void > request;
        boost::shared_ptr< void > result = remoteServer->call< void >("findObjects", request);
    }
    else if (vm.count("graspObject")) {
        if (argc != 8) {
            std::cout << "Too many/few coordinates entered-6 (width,height,depth,x,y,z) are expected! use --help for more information " << argc << std::endl;
        } else {

            boost::shared_ptr< rst::geometry::BoundingBox3DFloat> object(new rst::geometry::BoundingBox3DFloat);

            object->set_width(boost::lexical_cast< float >(argv[2]));
            object->set_height(boost::lexical_cast< float >(argv[3]));
            object->set_depth(boost::lexical_cast< float >(argv[4]));

            object->mutable_transformation()->mutable_rotation()->set_qw(0.0);
            object->mutable_transformation()->mutable_rotation()->set_qx(0.0);
            object->mutable_transformation()->mutable_rotation()->set_qy(0.0);
            object->mutable_transformation()->mutable_rotation()->set_qz(0.0);
            object->mutable_transformation()->mutable_translation()->set_x(boost::lexical_cast<double>(argv[5]));
            object->mutable_transformation()->mutable_translation()->set_y(boost::lexical_cast<double>(argv[6]));
            object->mutable_transformation()->mutable_translation()->set_z(boost::lexical_cast<double>(argv[7]));

            boost::shared_ptr<rst::generic::Dictionary> result;
            result = remoteServer->call<rst::generic::Dictionary>("graspObject", object, 60);
            std::cout << "GraspReturnType: \n" << result->DebugString() << std::endl;
        }
    } else if (vm.count("graspObjectOrientation")) {
        if (argc != 8) {
            std::cout << "Too many/few coordinates entered-6 (width,height,depth,x,y,z) are expected! use --help for more information " << argc << std::endl;
        } else {

            boost::shared_ptr< rst::geometry::BoundingBox3DFloat> object(new rst::geometry::BoundingBox3DFloat);

            object->set_width(boost::lexical_cast< float >(argv[2]));
            object->set_height(boost::lexical_cast< float >(argv[3]));
            object->set_depth(boost::lexical_cast< float >(argv[4]));

            object->mutable_transformation()->mutable_rotation()->set_qw(0.0);
            object->mutable_transformation()->mutable_rotation()->set_qx(0.0);
            object->mutable_transformation()->mutable_rotation()->set_qy(0.0);
            object->mutable_transformation()->mutable_rotation()->set_qz(0.0);

            object->mutable_transformation()->mutable_translation()->set_x(boost::lexical_cast<double>(argv[5]));
            object->mutable_transformation()->mutable_translation()->set_y(boost::lexical_cast<double>(argv[6]));
            object->mutable_transformation()->mutable_translation()->set_z(boost::lexical_cast<double>(argv[7]));

            boost::shared_ptr<rst::generic::Dictionary> result;
            result = remoteServer->call<rst::generic::Dictionary>("graspObjectOrientation", object);
            std::cout << "GraspReturnType: \n" << result->DebugString() << std::endl;
        }
    } else if (vm.count("placeObjectAt")) {
        if (argc != 8) {
            std::cout << "Too many/few coordinates entered! use --help for more information " << argc << std::endl;
        } else {
            boost::shared_ptr< rst::geometry::Pose > object(Pose::default_instance().New());

            object->mutable_translation()->set_x(boost::lexical_cast< double >(argv[2]));
            object->mutable_translation()->set_y(boost::lexical_cast< double >(argv[3]));
            object->mutable_translation()->set_z(boost::lexical_cast< double >(argv[4]));

            object->mutable_rotation()->set_qw((double) 1);
            object->mutable_rotation()->set_qx(boost::lexical_cast< double >(argv[5]));
            object->mutable_rotation()->set_qy(boost::lexical_cast< double >(argv[6]));
            object->mutable_rotation()->set_qz(boost::lexical_cast< double >(argv[7]));

            boost::shared_ptr<rst::generic::Dictionary> result;
            result = remoteServer->call<rst::generic::Dictionary>("placeObjectAt", object);
            std::cout << "GraspablePose: \n" << result->DebugString() << std::endl;
        }
    } else if (vm.count("placeObjectAtExact")) {
        if (argc != 8) {
            std::cout << "Too many/few coordinates entered! use --help for more information " << argc << std::endl;
        } else {
            boost::shared_ptr< rst::geometry::Pose > object(Pose::default_instance().New());

            object->mutable_translation()->set_x(boost::lexical_cast< double >(argv[2]));
            object->mutable_translation()->set_y(boost::lexical_cast< double >(argv[3]));
            object->mutable_translation()->set_z(boost::lexical_cast< double >(argv[4]));

            object->mutable_rotation()->set_qw((double) 1);
            object->mutable_rotation()->set_qx(boost::lexical_cast< double >(argv[5]));
            object->mutable_rotation()->set_qy(boost::lexical_cast< double >(argv[6]));
            object->mutable_rotation()->set_qz(boost::lexical_cast< double >(argv[7]));

            boost::shared_ptr<rst::generic::Dictionary> result;
            result = remoteServer->call<rst::generic::Dictionary>("placeObjectAtExact", object);
            std::cout << "GraspablePose: \n" << result->DebugString() << std::endl;
        }
    } else if (vm.count("testGrasping")) {
        boost::shared_ptr<rst::geometry::Shape3DFloat> obstacles(Shape3DFloat::default_instance().New());
        rst::geometry::BoundingBox3DFloat* box1 = obstacles->add_box();
        rst::geometry::BoundingBox3DFloat* box2 = obstacles->add_box();
        rst::geometry::BoundingBox3DFloat* box3 = obstacles->add_box();

        boost::shared_ptr<rst::geometry::BoundingBox3DFloat> boxGrasp(BoundingBox3DFloat::default_instance().New());


        box1->mutable_transformation()->mutable_translation()->set_x(0);
        box1->mutable_transformation()->mutable_translation()->set_y(0);
        box1->mutable_transformation()->mutable_translation()->set_z(40);
        box1->mutable_transformation()->mutable_rotation()->set_qw(0);
        box1->mutable_transformation()->mutable_rotation()->set_qx(0);
        box1->mutable_transformation()->mutable_rotation()->set_qy(0);
        box1->mutable_transformation()->mutable_rotation()->set_qz(0);
        box1->set_depth(0.400);
        box1->set_width(0.400);
        box1->set_height(0.500);

        box2->mutable_transformation()->mutable_translation()->set_x(40);
        box2->mutable_transformation()->mutable_translation()->set_y(0);
        box2->mutable_transformation()->mutable_translation()->set_z(40);
        box2->mutable_transformation()->mutable_rotation()->set_qw(0);
        box2->mutable_transformation()->mutable_rotation()->set_qx(0);
        box2->mutable_transformation()->mutable_rotation()->set_qy(0);
        box2->mutable_transformation()->mutable_rotation()->set_qz(0);
        box2->set_depth(0.400);
        box2->set_width(0.400);
        box2->set_height(0.500);

        box3->mutable_transformation()->mutable_translation()->set_x(40);
        box3->mutable_transformation()->mutable_translation()->set_y(40);
        box3->mutable_transformation()->mutable_translation()->set_z(40);
        box3->mutable_transformation()->mutable_rotation()->set_qw(0);
        box3->mutable_transformation()->mutable_rotation()->set_qx(0);
        box3->mutable_transformation()->mutable_rotation()->set_qy(0);
        box3->mutable_transformation()->mutable_rotation()->set_qz(0);
        box3->set_depth(0.400);
        box3->set_width(0.400);
        box3->set_height(0.500);

        boxGrasp->mutable_transformation()->mutable_translation()->set_x(0);
        boxGrasp->mutable_transformation()->mutable_translation()->set_y(0);
        boxGrasp->mutable_transformation()->mutable_translation()->set_z(30);
        boxGrasp->mutable_transformation()->mutable_rotation()->set_qw(0);
        boxGrasp->mutable_transformation()->mutable_rotation()->set_qx(0);
        boxGrasp->mutable_transformation()->mutable_rotation()->set_qy(0);
        boxGrasp->mutable_transformation()->mutable_rotation()->set_qz(0);
        boxGrasp->set_depth(0.400);
        boxGrasp->set_width(0.400);
        boxGrasp->set_height(0.500);

        std::cout << "Sending obstacles: \n" << obstacles->DebugString() << std::endl;
        remoteServer->call<rst::generic::Dictionary>("setObstacles", obstacles);

        boost::shared_ptr<rst::generic::Dictionary> result;
        std::cout << "Sending graspObject: \n" << boxGrasp->DebugString() << std::endl;
        result = remoteServer->call<rst::generic::Dictionary>("graspObject", boxGrasp);
        std::cout << "GRT: \n" << result->DebugString() << std::endl;
    } else {
        std::cout << "wrong Arguments! Use: ArmControlRemoteServer --help" << std::endl;
    }

}

int main(int argc, char **argv) {

	rsc::misc::initSignalWaiter();

    try {
        //register converters
        boost::shared_ptr< rsb::converter::ProtocolBufferConverter<JointAngles> >
                converter(new rsb::converter::ProtocolBufferConverter<JointAngles>());
        rsb::converter::converterRepository<std::string>()->registerConverter(converter);

        boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::Pose> >
                converter2(new rsb::converter::ProtocolBufferConverter<rst::geometry::Pose>());

        rsb::converter::converterRepository<std::string>()->registerConverter(converter2);

        boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::math::Vec3DDouble> >
                converter3(new rsb::converter::ProtocolBufferConverter<rst::math::Vec3DDouble>());
        rsb::converter::converterRepository<std::string>()->registerConverter(converter3);

        boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::generic::Dictionary> >
                converter4(new rsb::converter::ProtocolBufferConverter<rst::generic::Dictionary>());
        rsb::converter::converterRepository<std::string>()->registerConverter(converter4);

        boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::Shape3DFloat> >
                converter5(new rsb::converter::ProtocolBufferConverter<rst::geometry::Shape3DFloat>());
        rsb::converter::converterRepository<std::string>()->registerConverter(converter5);

        boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::PointCloud3DFloat> >
                converter6(new rsb::converter::ProtocolBufferConverter<rst::geometry::PointCloud3DFloat>());
        rsb::converter::converterRepository<std::string>()->registerConverter(converter6);

        boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::BoundingBox3DFloat> >
                converter7(new rsb::converter::ProtocolBufferConverter<rst::geometry::BoundingBox3DFloat>());
        rsb::converter::converterRepository<std::string>()->registerConverter(converter7);

        //create remoteServer(client)
        Factory& factory = getFactory();
        remoteServer = factory.createRemoteServer("/arm/picknplace/server");
        callServerMethod(argc, argv);

    } catch (rsb::Exception& e) {
        cerr << "failed to connect to Server: " << e.what();
    }

	// Wait here so incoming method calls can be processed.
	return 0;
}


