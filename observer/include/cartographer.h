#include <iostream>
#include "ros/ros.h"
#include "camera_model.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/UInt32MultiArray.h"
class cartographer
{
private:
	ros::NodeHandle s_;
	ros::Subscriber curr_pose,
					r1, r2, r3, r4, r5, r6, r7, 
					g1, g2, g3, g4, g5, g6, g7;
	ros::Publisher pose_arr_pub;
	geometry_msgs::PoseArray groundbots_world_loc;
	geometry_msgs::PoseStamped uavPose;
	projection_::cameraModel c1;
	projection_::cameraModel c2;
	projection_::cameraModel c3;
	projection_::cameraModel c4;
	projection_::cameraModel c5;
	projection_::cameraModel c6;
	projection_::cameraModel c7;
public:
	cartographer();
	~cartographer();
	void verify_positions_location();
	void sort_positions_time();
	void update_pose(const geometry_msgs::PoseStamped& cur_loc);
	void cam_1_callback(const std_msgs::UInt32MultiArray& msg);
	void cam_2_callback(const std_msgs::UInt32MultiArray& msg);
	void cam_3_callback(const std_msgs::UInt32MultiArray& msg);
	void cam_4_callback(const std_msgs::UInt32MultiArray& msg);
	void cam_5_callback(const std_msgs::UInt32MultiArray& msg);
	void cam_6_callback(const std_msgs::UInt32MultiArray& msg);
	void cam_7_callback(const std_msgs::UInt32MultiArray& msg);
};
