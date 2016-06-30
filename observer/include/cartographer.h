#include <iostream>
#include "ros/ros.h"
#include "camera_model.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/Int32MultiArray.h"
#include <vector>
class cartographer
{
private:
	ros::NodeHandle s_;
	ros::Subscriber curr_pose,
					r1, r2, r3, r4, r5, r6, r7, 
					g1, g2, g3, g4, g5, g6, g7;
	ros::Publisher rpp, gpp; //red plate publisher, green plate publisher
	geometry_msgs::PoseArray green_groundbots_world_loc, red_groundbots_world_loc;
	geometry_msgs::PoseStamped uavPose_;
	projection_::cameraModel c1, c2, c3, c4, c5, c6, c7;

public:
	cartographer();
	~cartographer();
	void merge_positions_location(std::vector<geometry_msgs::Pose> po_v, char color);
	void point_callback(const std_msgs::Int32MultiArray& msg, char camID, char color);
	void update_pose(const geometry_msgs::PoseStamped& cur_loc);
	void cam_1r_callback(const std_msgs::Int32MultiArray& msg);
	void cam_2r_callback(const std_msgs::Int32MultiArray& msg);
	void cam_3r_callback(const std_msgs::Int32MultiArray& msg);
	void cam_4r_callback(const std_msgs::Int32MultiArray& msg);
	void cam_5r_callback(const std_msgs::Int32MultiArray& msg);
	void cam_6r_callback(const std_msgs::Int32MultiArray& msg);
	void cam_7r_callback(const std_msgs::Int32MultiArray& msg);
	void cam_1g_callback(const std_msgs::Int32MultiArray& msg);
	void cam_2g_callback(const std_msgs::Int32MultiArray& msg);
	void cam_3g_callback(const std_msgs::Int32MultiArray& msg);
	void cam_4g_callback(const std_msgs::Int32MultiArray& msg);
	void cam_5g_callback(const std_msgs::Int32MultiArray& msg);
	void cam_6g_callback(const std_msgs::Int32MultiArray& msg);
	void cam_7g_callback(const std_msgs::Int32MultiArray& msg);
};
