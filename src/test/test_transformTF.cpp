#include <ros/ros.h>
#include "../util/TransformerTF.h"
#include <geometry_msgs/PoseStamped.h>
#include <string>

/* for this test to work, three frames must be published relative to a reference frame (here world)
 * simulating wrist frame of an arm in rest position
 * rosrun tf static_transform_publisher 0 0 1 0 1.5707 0 /world /wrist_frame 100
 * simulating grasp frame of an arm in rest position
 * rosrun tf static_transform_publisher 0.074 0 0 1.5707 0 3.1415 /wrist_frame /grasp_frame 100
 * generated grasp frame by the grasp gen represents grasp in reference frame world:
 * rosrun tf static_transform_publisher 0.1 0.2 0.3 0.66598 0.48296 0.29995 0.48296 /world /mygrasp 100
 * 
 * result is seen if compute matched expected
*/

int main(int argc, char **argv) {

  ros::init(argc, argv, "test_transformTF");
  TransformerTF tfTransformer;

  geometry_msgs::PoseStamped input_pose, output_pose;
  input_pose.header.frame_id = "base_link";

  input_pose.pose.position.x = 0.1;
  input_pose.pose.position.y = 0.2;
  input_pose.pose.position.z = 0.3;

  // w0.66598, x0.48296, y0.29995, z0.48296 = matrix2quaternion(rotx(pi/4)*roty(pi/3)*rotz(pi/4))
  input_pose.pose.orientation.x = 0.66598;
  input_pose.pose.orientation.y = 0.48296;
  input_pose.pose.orientation.z = 0.29995;
  input_pose.pose.orientation.w = 0.48296;


  std::string input_frame = "grasp_frame";
  std::string output_frame = "wrist_frame";
  
  ROS_INFO("Input pose (in frame: %s) x:%.3f y:%.3f z:%.3f - ori qw:%.3f qx:%.3f qy:%.3f qz:%.3f",
            input_pose.header.frame_id.c_str(), input_pose.pose.position.x, input_pose.pose.position.y, input_pose.pose.position.z,
            input_pose.pose.orientation.w, input_pose.pose.orientation.x, input_pose.pose.orientation.y, input_pose.pose.orientation.z);
  ros::spinOnce();
  tfTransformer.transformLink(input_pose, output_pose, input_frame, output_frame);
  ros::spinOnce();
  ROS_INFO("Calculated output pose (in frame: %s) x:%.3f y:%.3f z:%.3f - ori qw:%.3f qx:%.3f qy:%.3f qz:%.3f",
            output_pose.header.frame_id.c_str(), output_pose.pose.position.x, output_pose.pose.position.y, output_pose.pose.position.z,
            output_pose.pose.orientation.w, output_pose.pose.orientation.x, output_pose.pose.orientation.y, output_pose.pose.orientation.z);
            
  ROS_INFO("Expected output pose   (in frame: base_link) x:0.074 y:0.205 z:0.231 - ori qw:0.812 qx:-0.129 qy:-0.554 qz:-0.129");

  return 0;
}
