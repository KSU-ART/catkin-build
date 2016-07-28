#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32MultiArray.h>
#include <math.h>
#include <iostream>
#include <string>
#include <vector>

#define PI 3.14159265359

class ai_navigator
{
private:
	/// ******** CONSTANTS *********
	double SETPOINT_INTERVAL;
	bool DEBUG;
	double TARGET_ALTITUDE;
	/// the angle +- 180 to the interact with ground robot in radians
	double GOAL_ANGLE;
	
	/// ros hadlers
	ros::NodeHandle n_;
	ros::Subscriber curent_pose_sub, red_plate_poses_sub,
			green_plate_poses_sub, obstacles_sub;
	ros::Publisher retractMsg_pub, modeMsg_pub, setpoint_pub, 
			EMERGENCY_LAND_pub, pid_XY_pub, pid_z_pub;4
	/// State machine
	enum state 
	{
		TakeOff, RandomTraversal, TargetGR,
		InteractWithRobot, AvoidObstacle, HoldPosition, Land
	};
	state cur_state;
	double state_time, start_time; 
	bool new_state;
	
	/// current position and target position
	geometry_msgs::PoseStamped current_pose;
	geometry_msgs::Point setpoint;
	bool at_setpoint;
	
	/// targeting new ground robot
	double setpoint_start_time;
	geometry_msgs::Pose target_gr;
	
	void find_target();
	void crop_angle(double& angle);
	
	/// plate callbacks
	geometry_msgs::Pose min_loc_r;
	geometry_msgs::Pose min_loc_g;
	bool found_red;
	bool found_green;
	
	/// interact ground robot
	
	
public:
	///constructor
	ai_navigator();
	
	///main function loop
	void init();
	
	///determine_state_function
	state determine_state();
	
	///action functions:
	void take_off();
	void random_traversal();
	void target_ground_robot();
	void interact_with_robot();
	void avoid_obstacle();
	void hold_position();
	void land();
	
	///data callback functions:
	void current_pose_cb(const geometry_msgs::PoseStamped& msg);
	void red_plate_poses_cb(const geometry_msgs::PoseArray& msg);
	void green_plate_poses_cb(const geometry_msgs::PoseArray& msg);
	void obstacles_cb(const geometry_msgs::PoseArray& msg);

};
