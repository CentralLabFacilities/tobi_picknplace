/*
 * GraspGenerator.cpp
 *
 *  Created on: Jul 2, 2015
 *      Author: lziegler
 */

#include "GraspGenerator.h"

#include "../util/ParamReader.h"

#include <ros/ros.h>

using namespace std;

GraspGenerator::GraspGenerator() {

}

GraspGenerator::~GraspGenerator() {
}

/**
 * x, y, z: center of grasp point (the point that should be between the finger tips of the gripper)
 * MUST BE IN ARM COORDINATES
 */
std::vector<moveit_msgs::Grasp> GraspGenerator::generate_grasps_angle_trans(double x, double y,
		double z, double height) {

	ParamReader& params = ParamReader::getParamReader();

	ROS_INFO_NAMED("GraspGenerator", "Generate grasps at %.3f,%.3f,%.3f (frame: %s) - h:%.3f", x, y, z, params.frameArm.c_str(), height);

	std::vector<moveit_msgs::Grasp> grasps;

	double xStart = x;
	double xMax = 0.0;

	if (height > params.heightSkipTop + params.heightSkipBottom) {
		double xTop = x + height / 2.0 - params.heightSkipTop;
		double xBottom = x - height / 2.0 + params.heightSkipBottom;
		xStart = (xTop + xBottom) / 2.0;
		xMax = (xTop - xBottom) / 2.0;
	}

	for (double yAbs = 0; yAbs <= params.transMax; yAbs += params.transInc) {
		for (double ySign = yAbs; ySign >= -yAbs; ySign -= 2 * yAbs) {
			for (double zAbs = 0; zAbs <= params.transMax; zAbs += params.transInc) {
				for (double zSign = zAbs; zSign >= -zAbs; zSign -= 2 * zAbs) {
					for (double xAbs = 0; xAbs <= xMax; xAbs += params.heightInc) {
						for (double xSign = xAbs; xSign >= -xAbs; xSign -= 2 * xAbs) {
							std::vector<moveit_msgs::Grasp> graspsTmp;
							graspsTmp = generate_grasps_angle_only(xStart + xSign, y + ySign, z + zSign);
							grasps.insert(grasps.end(), graspsTmp.begin(), graspsTmp.end());
							if (xAbs == 0) break;
						}
					}
					if (zAbs == 0) break;
				}
			}
			if (yAbs == 0) break;
		}
	}
	ROS_INFO_NAMED("GraspGenerator", "Done generating %i grasps.", (int ) grasps.size());

	return grasps;

}

/**
 * x, y, z: center of grasp point (the point that should be between the finger tips of the gripper)
 */
std::vector<moveit_msgs::Grasp> GraspGenerator::generate_grasps_angle_only(double x, double y,
		double z) {

	ParamReader& params = ParamReader::getParamReader();
	vector<double> rot = params.graspRot;

	std::vector<moveit_msgs::Grasp> grasps;

	tf::Transform towardsObjectRotation(tf::createQuaternionFromRPY(rot.at(0), rot.at(1), rot.at(2)));
	tf::Transform graspThroughTransform;
	graspThroughTransform.setOrigin(tf::Vector3(params.graspThroughDistance,0.0, 0.0));
	graspThroughTransform.setRotation(tf::createQuaternionFromYaw(0));

	// create grasps
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(x, y, z));

	// try different pitches
	for (double yRot0 = 0.0; yRot0 <= params.anglePitchMax; yRot0 += params.anglePitchInc) {
		for (double yRot1 = yRot0; yRot1 >= -yRot0; yRot1 -= 2 * yRot0) {
			for (double zRot = 0.0; zRot <= M_PI; zRot += M_PI) {
				for (double xRot = 0.0; xRot <= params.angleYawMax; xRot += params.angleYawInc) {
					// + atan2 to center the grasps around the vector from arm to object
					transform.setRotation(tf::createQuaternionFromRPY(xRot, yRot1, zRot));
					grasps.push_back(build_grasp(transform * towardsObjectRotation * graspThroughTransform));

					if (xRot != 0.0) {
						transform.setRotation(tf::createQuaternionFromRPY(-xRot, yRot1, zRot));
						ROS_DEBUG("Try grasp (ypr): %.3f %.3f %.3f", -xRot, yRot1, zRot);
						grasps.push_back(build_grasp(transform * towardsObjectRotation * graspThroughTransform));
					}
				}
			}
			if (yRot0 == 0.0)
				break;
		}
	}

	return grasps;
}

std::vector<moveit_msgs::PlaceLocation> GraspGenerator::generate_placeloc_angle_trans(double x, double y,
		double z) {

	ParamReader& params = ParamReader::getParamReader();

	ROS_INFO_NAMED("GraspGenerator", "Generate place locations at %.3f,%.3f,%.3f (frame: %s)", x, y, z, params.frameArm.c_str());

	std::vector<moveit_msgs::PlaceLocation> placelocs;

	for (double yAbs = 0; yAbs <= params.placeAtMax; yAbs += params.placeAtInc) {
		for (double ySign = yAbs; ySign >= -yAbs; ySign -= 2 * yAbs) {
			for (double zAbs = 0; zAbs <= params.placeAtMax; zAbs += params.placeAtInc) {
				for (double zSign = zAbs; zSign >= -zAbs; zSign -= 2 * zAbs) {
					for (double xAbs = 0; xAbs <= params.placeAtMax; xAbs += params.placeAtInc) {
						for (double xSign = xAbs; xSign >= -xAbs; xSign -= 2 * xAbs) {
							std::vector<moveit_msgs::PlaceLocation> placelocsTmp;
							placelocsTmp = generate_placeloc_angle_only(x + xSign, y + ySign, z + zSign);
							placelocs.insert(placelocs.end(), placelocsTmp.begin(), placelocsTmp.end());
							if (xAbs == 0) break;
						}
					}
					if (zAbs == 0) break;
				}
			}
			if (yAbs == 0) break;
		}
	}
	ROS_INFO_NAMED("GraspGenerator", "Done generating %i place locations.", (int ) placelocs.size());

	return placelocs;

}
std::vector<moveit_msgs::PlaceLocation> GraspGenerator::generate_placeloc_angle_only(double x, double y,
		double z) {

	ParamReader& params = ParamReader::getParamReader();
	vector<double> rot = params.graspRot;

	std::vector<moveit_msgs::PlaceLocation> placelocs;

	tf::Transform towardsObjectRotation(tf::createQuaternionFromRPY(rot.at(0), rot.at(1), rot.at(2)));

	// create grasps
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(x, y, z));

	// try different pitches
	for (double yRot0 = 0.0; yRot0 <= params.placeAtAnglePitchMax; yRot0 += params.placeAtAnglePitchInc) {
		for (double yRot1 = yRot0; yRot1 >= -yRot0; yRot1 -= 2 * yRot0) {
			for (double zRot = 0.0; zRot <= M_PI; zRot += M_PI) {
				for (double xRot = 0.0; xRot <= params.placeAtAngleYawMax; xRot += params.placeAtAngleYawInc) {
					// + atan2 to center the grasps around the vector from arm to object
					transform.setRotation(tf::createQuaternionFromRPY(xRot, yRot1, zRot));
					placelocs.push_back(build_place_location(transform * towardsObjectRotation));

					if (xRot != 0.0) {
						transform.setRotation(tf::createQuaternionFromRPY(-xRot, yRot1, zRot));
						ROS_DEBUG("Try place location (ypr): %.3f %.3f %.3f", -xRot, yRot1, zRot);
						placelocs.push_back(build_place_location(transform * towardsObjectRotation));
					}
				}
			}
			if (yRot0 == 0.0)
				break;
		}
	}

	return placelocs;
}
std::vector<moveit_msgs::PlaceLocation> GraspGenerator::generate_place_locations(double x, double y,
		double z, tf::Quaternion targetOrientation) {
	ParamReader& params = ParamReader::getParamReader();

	ROS_INFO_NAMED("GraspGenerator", "Generate place locations at %.3f,%.3f,%.3f (frame: %s) orientation: %.3f,%.3f,%.3f,%.3f,", x, y, z,
			params.frameArm.c_str(), targetOrientation.x(), targetOrientation.y(), targetOrientation.z(), targetOrientation.w());

	std::vector<moveit_msgs::PlaceLocation> locations;
	// x: up
	// y: right
	// z: forward

	for (double xi = 0; xi < params.placeXMax; xi += params.placeXInc) {
		for (double yi = 0; yi < y + params.placeYMax; yi += params.placeYZInc) {
			for (double ys = yi; ys >= -yi; ys -= 2 * yi) {
				for (double zi = 0; zi < params.placeZMax; zi += params.placeYZInc) {
					for (double zs = zi; zs >= -zi; zs -= 2 * zi) {
						for (double yaw = 0; yaw < params.angleYawMax; yaw += params.angleYawInc) {
							for (double yawSign = yaw; yawSign >= -yaw; yawSign -= 2 * yaw) {
								tf::Transform transform;
								transform.setOrigin(tf::Vector3(x + xi, y + ys, z + zs));
								transform.setRotation(tf::createQuaternionFromRPY(yawSign, 0,0));
								locations.push_back(build_place_location(transform * tf::Transform(targetOrientation)));
								if (yaw == 0) break;
							}
						}
						if (zi == 0) break;
					}
				}
				if (yi == 0) break;
			}
		}
	}
	ROS_INFO_NAMED("GraspGenerator", "Done generating %i locations.", (int ) locations.size());
	return locations;
}

std::vector<moveit_msgs::PlaceLocation> GraspGenerator::generate_place_locations(double x, double y,
		double z, double w, double h, double d, tf::Quaternion targetOrientation) {
	ParamReader& params = ParamReader::getParamReader();

	ROS_INFO_NAMED("GraspGenerator", "Generate place locations at %.3f,%.3f,%.3f (frame: %s) orientation: %.3f,%.3f,%.3f,%.3f whd: %.3f,%.3f,%.3f", x, y, z,
			params.frameArm.c_str(), targetOrientation.x(), targetOrientation.y(), targetOrientation.z(), targetOrientation.w(), w, h, d);

	std::vector<moveit_msgs::PlaceLocation> locations;
	// x: up
	// y: right
	// z: forward

	double placeYMax = w / 2.0;
	double placeZMax = d / 2.0;

	for (double xi = 0; xi < params.placeXMax; xi += params.placeXInc) {
		for (double yi = 0; yi < y + placeYMax; yi += params.placeYZInc) {
			for (double ys = yi; ys >= -yi; ys -= 2 * yi) {
				for (double zi = 0; zi < placeZMax; zi += params.placeYZInc) {
					for (double zs = zi; zs >= -zi; zs -= 2 * zi) {
						for (double yaw = 0; yaw < params.angleYawMax; yaw += params.angleYawInc) {
							for (double yawSign = yaw; yawSign >= -yaw; yawSign -= 2 * yaw) {
								tf::Transform transform;
								transform.setOrigin(tf::Vector3(x + xi, y + ys, z + zs));
								transform.setRotation(tf::createQuaternionFromRPY(yawSign, 0,0));
								locations.push_back(build_place_location(transform * tf::Transform(targetOrientation)));
								if (yaw == 0) break;
							}
						}
						if (zi == 0) break;
					}
				}
				if (yi == 0) break;
			}
		}
	}
	ROS_INFO_NAMED("GraspGenerator", "Done generating %i locations.", (int ) locations.size());
	return locations;
}

//todo: h2r5
moveit_msgs::PlaceLocation GraspGenerator::build_place_location(tf::Transform t) {
	ParamReader& params = ParamReader::getParamReader();
	static int j = 0;
	ROS_DEBUG_NAMED("GraspGenerator", "Build place location at %.3f,%.3f,%.3f orientation: %.3f,%.3f,%.3f,%.3f", t.getOrigin().x(),t.getOrigin().y(), t.getOrigin().z(), t.getRotation().x(), t.getRotation().y(), t.getRotation().z(), t.getRotation().w());

	geometry_msgs::PoseStamped p;
	p.header.frame_id = params.frameArm;
	p.header.stamp = ros::Time::now();

	// orientation
	tf::quaternionTFToMsg(t.getRotation(), p.pose.orientation);

	p.pose.position.x = t.getOrigin().m_floats[0];
	p.pose.position.y = t.getOrigin().m_floats[1];
	p.pose.position.z = t.getOrigin().m_floats[2];

	moveit_msgs::PlaceLocation g;
	g.place_pose = p;
	g.id = boost::lexical_cast<std::string>(j);
	g.allowed_touch_objects.push_back("target_object");

	if(ParamReader::getParamReader().model == "katana") {

        g.pre_place_approach.direction.vector.x = -1.0;
        g.pre_place_approach.direction.header.frame_id = params.frameArm;
    //	g.pre_place_approach.min_distance = approachMinDistance;
        g.pre_place_approach.min_distance = 0.01;
        g.pre_place_approach.desired_distance = params.approachDesiredDistance;

        g.post_place_retreat.direction.vector.x = -1.0;
        g.post_place_retreat.direction.header.frame_id = params.frameGripper;
        g.post_place_retreat.min_distance = params.retreatMinDistance;
        g.post_place_retreat.desired_distance = params.retreatDesiredDistance;

        g.post_place_posture.joint_names.push_back("katana_l_finger_joint");
        g.post_place_posture.joint_names.push_back("katana_r_finger_joint");
        g.post_place_posture.points.resize(1);
        g.post_place_posture.points[0].positions.push_back(params.eefPosOpen.at(0));
        g.post_place_posture.points[0].positions.push_back(params.eefPosOpen.at(0));
	}

	j++;
	return g;
}

moveit_msgs::Grasp GraspGenerator::build_grasp(tf::Transform t) {
	ParamReader& params = ParamReader::getParamReader();
	static int i = 0;

	moveit_msgs::Grasp grasp;
	geometry_msgs::PoseStamped pose;

	tf::Vector3& origin = t.getOrigin();
	tf::Quaternion rotation = t.getRotation();

	// orientation
	tf::quaternionTFToMsg(rotation, pose.pose.orientation);

	// translation
	pose.pose.position.x = origin.m_floats[0];
	pose.pose.position.y = origin.m_floats[1];
	pose.pose.position.z = origin.m_floats[2];

	pose.header.frame_id = params.frameArm;
	pose.header.stamp = ros::Time::now();

	grasp.grasp_pose = pose;

	grasp.id = boost::lexical_cast<std::string>(i);

	i++;
	return grasp;
}
