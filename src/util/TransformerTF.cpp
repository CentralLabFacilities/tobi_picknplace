/*
 * TransformerTF.cpp
 *
 *  Created on: Jul 2, 2015
 *      Author: lziegler
 */

#include "TransformerTF.h"

#include <ros/ros.h>
#include <kdl/frames.hpp>
#include <moveit_msgs/Grasp.h>
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace moveit_msgs;

TransformerTF::TransformerTF():tfListener(tfBuffer) {

}

TransformerTF::~TransformerTF() {
}

//! Change frame of reference of the input vector coordinates in 'from' frame to output vector coordinates in 'to' frame
//! @param x input coordinate on axis-x
//! @param y input coordinate on axis-y
//! @param z input coordinate on axis-z
//! @param from input frame name
//! @param xOut output coordinate on axis-x
//! @param yOut output coordinate on axis-y
//! @param zOut output coordinate on axis-z
//! @param to desired target frame for output
bool TransformerTF::transform(double x, double y, double z, const std::string &from, double &xOut,double &yOut,double &zOut, const std::string &to) const {

	geometry_msgs::Vector3Stamped vec;
	vec.vector.x = x;
	vec.vector.y = y;
	vec.vector.z = z;
	vec.header.frame_id = from;

	if (transform(vec, vec, to)) {
		xOut = vec.vector.x;
		yOut = vec.vector.y;
		zOut = vec.vector.z;
		return true;
	} else {
		return false;
	}
}

//! Change frame of reference of the input End-Effector pose with given frame to a 'to' frame
//! @param pose input End-Effector pose containing source frame
//! @param poseOut output End-Effector pose
//! @param to desired target frame for output pose
bool TransformerTF::transform(const EefPose &pose, EefPose &poseOut, const string &to) const {

	geometry_msgs::PoseStamped ps;

	ps.pose.position.x = pose.translation.xMeter;
	ps.pose.position.y = pose.translation.yMeter;
	ps.pose.position.z = pose.translation.zMeter;
	ps.pose.orientation.w = pose.rotation.qw;
	ps.pose.orientation.x = pose.rotation.qx;
	ps.pose.orientation.y = pose.rotation.qy;
	ps.pose.orientation.z = pose.rotation.qz;
	ps.header.frame_id = pose.frame;

	if (transform(ps, ps, to)) {
		poseOut.translation.xMeter = ps.pose.position.x;
		poseOut.translation.yMeter = ps.pose.position.y;
		poseOut.translation.zMeter = ps.pose.position.z;
		poseOut.rotation.qw = ps.pose.orientation.w;
		poseOut.rotation.qx = ps.pose.orientation.x;
		poseOut.rotation.qy = ps.pose.orientation.y;
		poseOut.rotation.qz = ps.pose.orientation.z;
		poseOut.frame = to;
		return true;
	} else {
		return false;
	}
}

//! Change only the orientation of a frame of reference of the input grasp pose with given frame to a 'to' frame, keeping position as input.
//! @param grasp input grasp containing pose and source frame
//! @param graspOut output grasp
//! @param to desired target frame for output grasp
bool TransformerTF::localtransform(const moveit_msgs::Grasp &grasp, moveit_msgs::Grasp &graspOut, const string &to) const {
  
  geometry_msgs::PoseStamped ps;
  
	ps.pose.position.x = 0.0;
	ps.pose.position.y = 0.0;
	ps.pose.position.z = 0.0;
	ps.pose.orientation.w = grasp.grasp_pose.pose.orientation.w;
	ps.pose.orientation.x = grasp.grasp_pose.pose.orientation.x;
	ps.pose.orientation.y = grasp.grasp_pose.pose.orientation.y;
	ps.pose.orientation.z = grasp.grasp_pose.pose.orientation.z;
	ps.header.frame_id = grasp.grasp_pose.header.frame_id;

  if(transform(ps, ps, to)) {
		graspOut.grasp_pose.pose.position.x = grasp.grasp_pose.pose.position.x;
		graspOut.grasp_pose.pose.position.y = grasp.grasp_pose.pose.position.y;
		graspOut.grasp_pose.pose.position.z = grasp.grasp_pose.pose.position.z;
		graspOut.grasp_pose.pose.orientation.w = ps.pose.orientation.w;
		graspOut.grasp_pose.pose.orientation.x = ps.pose.orientation.x;
		graspOut.grasp_pose.pose.orientation.y = ps.pose.orientation.y;
		graspOut.grasp_pose.pose.orientation.z = ps.pose.orientation.z;
		graspOut.grasp_pose.header.frame_id = to;
  }
}

//! Change frame of reference of the input grasp pose with given frame to a 'to' frame
//! @param grasp input grasp containing pose and source frame
//! @param graspOut output grasp
//! @param to desired target frame for output grasp
bool TransformerTF::transform(const moveit_msgs::Grasp &grasp, moveit_msgs::Grasp &graspOut, const string &to) const {

	geometry_msgs::PoseStamped ps;

	ps.pose.position.x = grasp.grasp_pose.pose.position.x;
	ps.pose.position.y = grasp.grasp_pose.pose.position.y;
	ps.pose.position.z = grasp.grasp_pose.pose.position.z;
	ps.pose.orientation.w = grasp.grasp_pose.pose.orientation.w;
	ps.pose.orientation.x = grasp.grasp_pose.pose.orientation.x;
	ps.pose.orientation.y = grasp.grasp_pose.pose.orientation.y;
	ps.pose.orientation.z = grasp.grasp_pose.pose.orientation.z;
	ps.header.frame_id = grasp.grasp_pose.header.frame_id;

	if (transform(ps, ps, to)) {
		graspOut.grasp_pose.pose.position.x = ps.pose.position.x;
		graspOut.grasp_pose.pose.position.y = ps.pose.position.y;
		graspOut.grasp_pose.pose.position.z = ps.pose.position.z;
		graspOut.grasp_pose.pose.orientation.w = ps.pose.orientation.w;
		graspOut.grasp_pose.pose.orientation.x = ps.pose.orientation.x;
		graspOut.grasp_pose.pose.orientation.y = ps.pose.orientation.y;
		graspOut.grasp_pose.pose.orientation.z = ps.pose.orientation.z;
		graspOut.grasp_pose.header.frame_id = to;
		return true;
	} else {
		return false;
	}
}

//! Change frame of reference of the input ObjectShape with given frame to a 'to' frame 
//! @param object input ObjectShape containing pose and source frame
//! @param objectOut output ObjectShape
//! @param to desired target frame for output ObjectShape
bool TransformerTF::transform(const ObjectShape &object, ObjectShape &objectOut, const string &to) const {

	geometry_msgs::Vector3Stamped vec;
	vec.vector.x = object.center.xMeter;
	vec.vector.y = object.center.yMeter;
	vec.vector.z = object.center.zMeter;
	vec.header.frame_id = object.center.frame;

	if (transform(vec, vec, to)) {
		objectOut.center.xMeter = vec.vector.x;
		objectOut.center.yMeter = vec.vector.y;
		objectOut.center.zMeter = vec.vector.z;
		objectOut.center.frame = to;
		objectOut.depthMeter = object.depthMeter;
		objectOut.heightMeter = object.heightMeter;
		objectOut.widthMeter = object.widthMeter;
		return true;
	} else {
		return false;
	}
}

//! Change frame of reference of the input CollisionObject with given frame to a 'to' frame 
//! @param obj input CollisionObject containing pose and source frame
//! @param objOut output CollisionObject
//! @param to desired target frame for output CollisionObject
bool TransformerTF::transform(const CollisionObject &obj, CollisionObject &objOut, const std::string &to) const {
    try{
        CollisionObject myObj = obj;
        string from = myObj.header.frame_id;
        boost::algorithm::replace_all(from, "/", "");
        myObj.header.frame_id = from;
        //ROS_INFO_STREAM("transform " << from << " to " << to);
        if(from != ""){
            tfBuffer.transform(myObj, objOut, to);
        }
        return true;
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return false;
    }
}

//! Change frame of reference of the input PoseStamped with given frame to a 'to' frame 
//! @param pose input PoseStamped containing pose and source frame
//! @param poseOut output PoseStamped
//! @param to desired target frame for output PoseStamped
bool TransformerTF::transform(const geometry_msgs::PoseStamped &pose,geometry_msgs::PoseStamped &poseOut,  const std::string &to) const {
	try{
		geometry_msgs::PoseStamped myPose = pose;
		string from = myPose.header.frame_id;
		boost::algorithm::replace_all(from, "/", "");
		myPose.header.frame_id = from;
		//ROS_INFO_STREAM("transform " << from << " to " << to);
		tfBuffer.transform(myPose, poseOut, to);
		return true;
	} catch (tf2::TransformException &ex) {
		ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
		return false;
	}
}

//! Change frame of reference of the input Vector3Stamped with given frame to a 'to' frame 
//! @param vec input Vector3Stamped containing position and source frame
//! @param vecOut output Vector3Stamped
//! @param to desired target frame for output Vector3Stamped
bool TransformerTF::transform(const geometry_msgs::Vector3Stamped &vec, geometry_msgs::Vector3Stamped &vecOut, const std::string &to) const {
	try{
		geometry_msgs::Vector3Stamped myVec = vec;
		string from = myVec.header.frame_id;
		boost::algorithm::replace_all(from, "/", "");
		myVec.header.frame_id = from;
		//ROS_DEBUG_STREAM("transform " << from << " to " << to);
		tfBuffer.transform(myVec, vecOut, to);
		return true;
	} catch (tf2::TransformException &ex) {
		ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
		return false;
	}
}

namespace tf2 {

KDL::Frame gmTransformToKDL(const geometry_msgs::TransformStamped& t) {
	return KDL::Frame(
			KDL::Rotation::Quaternion(t.transform.rotation.x, t.transform.rotation.y,
					t.transform.rotation.z, t.transform.rotation.w),
			KDL::Vector(t.transform.translation.x, t.transform.translation.y,
					t.transform.translation.z));
}

// ##########################
// geometry_msgs::PoseStamped
// ##########################

// method to extract timestamp from object
template<>
const ros::Time& getTimestamp(const geometry_msgs::PoseStamped& t) {
	return t.header.stamp;
}

// method to extract frame id from object
template<>
const std::string& getFrameId(const geometry_msgs::PoseStamped& t) {
	return t.header.frame_id;
}

// this method needs to be implemented by client library developers
template<>
void doTransform(const geometry_msgs::PoseStamped& t_in, geometry_msgs::PoseStamped& t_out,
		const geometry_msgs::TransformStamped& transform) {
	KDL::Vector v(t_in.pose.position.x, t_in.pose.position.y, t_in.pose.position.z);
	KDL::Rotation r = KDL::Rotation::Quaternion(t_in.pose.orientation.x, t_in.pose.orientation.y,
			t_in.pose.orientation.z, t_in.pose.orientation.w);

	tf2::Stamped<KDL::Frame> v_out = tf2::Stamped<KDL::Frame>(
			gmTransformToKDL(transform) * KDL::Frame(r, v), transform.header.stamp,
			transform.header.frame_id);
	t_out.pose.position.x = v_out.p[0];
	t_out.pose.position.y = v_out.p[1];
	t_out.pose.position.z = v_out.p[2];
	v_out.M.GetQuaternion(t_out.pose.orientation.x, t_out.pose.orientation.y,
			t_out.pose.orientation.z, t_out.pose.orientation.w);
	t_out.header.stamp = v_out.stamp_;
	t_out.header.frame_id = v_out.frame_id_;
}
geometry_msgs::PoseStamped toMsg(const geometry_msgs::PoseStamped& in) {
	return in;
}
void fromMsg(const geometry_msgs::PoseStamped& msg, geometry_msgs::PoseStamped& out) {
	out = msg;
}

// #############################
// geometry_msgs::Vector3Stamped
// #############################

// method to extract timestamp from object
template<>
const ros::Time& getTimestamp(const geometry_msgs::Vector3Stamped& t) {
	return t.header.stamp;
}

// method to extract frame id from object
template<>
const std::string& getFrameId(const geometry_msgs::Vector3Stamped& t) {
	return t.header.frame_id;
}

template<>
void doTransform(const geometry_msgs::Vector3Stamped& t_in, geometry_msgs::Vector3Stamped& t_out,
		const geometry_msgs::TransformStamped& transform) {
	KDL::Vector v(t_in.vector.x, t_in.vector.y, t_in.vector.z);

	tf2::Stamped<KDL::Vector> v_out = tf2::Stamped<KDL::Vector>(
			gmTransformToKDL(transform) * v, transform.header.stamp,
			transform.header.frame_id);
	t_out.vector.x = v_out.data[0];
	t_out.vector.y = v_out.data[1];
	t_out.vector.z = v_out.data[2];
	t_out.header.stamp = v_out.stamp_;
	t_out.header.frame_id = v_out.frame_id_;
}
geometry_msgs::Vector3Stamped toMsg(const geometry_msgs::Vector3Stamped& in) {
	return in;
}
void fromMsg(const geometry_msgs::Vector3Stamped& msg, geometry_msgs::Vector3Stamped& out) {
	out = msg;
}

// #############################
// moveit_msgs::CollisionObject
// #############################

// method to extract timestamp from object
template<>
const ros::Time& getTimestamp(const CollisionObject& t) {
    return t.header.stamp;
}

// method to extract frame id from object
template<>
const std::string& getFrameId(const CollisionObject& t) {
    return t.header.frame_id;
}

template<>
void doTransform(const CollisionObject& t_in, CollisionObject& t_out,
        const geometry_msgs::TransformStamped& transform) {
    t_out = CollisionObject(t_in);
    for (int i = 0; i < t_in.primitive_poses.size(); i++) {
        KDL::Vector v(t_in.primitive_poses[i].position.x, t_in.primitive_poses[i].position.y, t_in.primitive_poses[i].position.z);
        KDL::Rotation r = KDL::Rotation::Quaternion(t_in.primitive_poses[i].orientation.x, t_in.primitive_poses[i].orientation.y,
                t_in.primitive_poses[i].orientation.z, t_in.primitive_poses[i].orientation.w);

        tf2::Stamped<KDL::Frame> v_out = tf2::Stamped<KDL::Frame>(
                gmTransformToKDL(transform) * KDL::Frame(r, v), transform.header.stamp,
                transform.header.frame_id);
        t_out.primitive_poses[i].position.x = v_out.p[0];
        t_out.primitive_poses[i].position.y = v_out.p[1];
        t_out.primitive_poses[i].position.z = v_out.p[2];
        v_out.M.GetQuaternion(t_out.primitive_poses[i].orientation.x, t_out.primitive_poses[i].orientation.y,
                t_out.primitive_poses[i].orientation.z, t_out.primitive_poses[i].orientation.w);
        t_out.header.stamp = v_out.stamp_;
        t_out.header.frame_id = v_out.frame_id_;
    }
}
CollisionObject toMsg(const CollisionObject& in) {
    return in;
}
void fromMsg(const CollisionObject& msg, CollisionObject& out) {
    out = msg;
}
}
