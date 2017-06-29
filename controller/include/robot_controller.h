/****************************************************************************************************
 * This is the class using the pid controller to 
 * generate output to the pixhawk.
 * 
 * CLASS INVARIANCE:
 * 		X+ => Forward
 * 		Y+ => Left
 * 		Z+ => Up
 *
 * MSG GUIDE:
 * Subscribed Topic:	            Data Type					Usage
 * "/IARC/ai_reset"		            std_msgs::Bool	            resets state machine and pids for when manual override is turned off
 * "/IARC/currentAltitude"	        std_msgs::Float32			true = override, false = disabled
 * "/IARC/setAltitude"		        std_msgs::Float32			true = emergencyland, false = normal
 * "/IARC/YOLO/target/x"	        std_msgs::Int16&        	setpoint (desired location)
 * "/IARC/YOLO/target/y"	        std_msgs::Int16&        	0 = altitude hold, 1 = stabilize, 2 = land;
 * "/IARC/OrientationNet/pos/x"     std_msgs::Int16&        	true = retracts down, false = up;
 * "/IARC/OrientationNet/pos/y"		std_msgs::Int16&        	{p, i, d, min, max}
 * "/IARC/Obstacle/PitchPID"		std_msgs::Float32			{p, i, d, min, max}
 * "/IARC/Obstacle/RollPID"		    std_msgs::Float32			{p, i, d, min, max}
 * 
 * MSG CONSTANCES:
 * MSG.CHAN_RELEASE = 0
 * MSG.CHAN_NOCHANGE = 65535
 * 
 * PRIORITY OF CONTROL:
 * 
 * 
 * Mannual Override Switch: 
 * channel 9 (8 in {0-8}) HIGH_PWM is land
 * 
 **************************************************************************************************/
#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <termios.h>
#include "flight_definitions.h"
#include <iostream>
#include "ros/ros.h"
#include <PIDController.h>
#include <math.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/RCIn.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"

#include "pid_handler.h"
#include "mavros_handler.h"

/// robot_controller class
/// 
class robot_controller 
{
private:
	ros::NodeHandle n;
	
	short throttle, roll, pitch, yaw;

	enum mode_enum{
		DownCam 	= 0,
		Obstacle 	= 1,
		Yolo 		= 2,
		Null		= 3
	};

	mode_enum state_mode = Yolo;
	mode_enum pre_state_mode = state_mode;

	int camera_width, camera_height;
	double current_altitude,
		   current_yolo_yaw,
		   current_yolo_pitch,
		   current_down_cam_pitch,
		   current_down_cam_roll,
		   current_obstacle_pitch,
		   current_obstacle_roll;

	// loads pid calibration file
	pid_handler pids("/home/stoplime/catkin_ws/src/catkin-build/controller/include/pid_calibration.txt");
	// pid_handler pids;

	mavros_handler mav;
	
public:
	//contructor
	robot_controller(){
		debug = true;
		
		//initial values
		/// camera dimentions
		camera_width 			= 640;
		camera_height 			= 480;
		/// altitude is in meters with the point lidar
		current_altitude		= 0;
		/// yolo yaw is the delta x value in pixels from the front camera
		current_yolo_yaw		= 0;
		/// yolo pitch is the delta y value in pixels from the front camera
		current_yolo_pitch		= 0;
		/// down cam pitch is the delta y value of the target to the center of the camera
		current_down_cam_pitch	= 0;
		/// down cam pitch is the delta x value of the target to the center of the camera
		current_down_cam_roll	= 0;
		/// obstacle pitch is the y component of the obstacle avoidence vector, NOT the actual obstacle vector
		current_obstacle_pitch	= 0;
		/// obstacle pitch is the x component of the obstacle avoidence vector, NOT the actual obstacle vector
		current_obstacle_roll	= 0;
		
		//subs:
		n.subscribe("/IARC/ai_reset", 1, &robot_controller::ai_reset_cb, this);
		n.subscribe("/IARC/currentAltitude", 1, &robot_controller::current_altitude_cb, this);
		n.subscribe("/IARC/setAltitude", 1, &robot_controller::target_altitude_cb, this);
		n.subscribe("/IARC/YOLO/target/x", 1, &robot_controller::yolo_x_cb, this);
		n.subscribe("/IARC/YOLO/target/y", 1, &robot_controller::yolo_y_cb, this);
		n.subscribe("/IARC/OrientationNet/pos/x", 1, &robot_controller::delta_down_cam_x_cb, this);
		n.subscribe("/IARC/OrientationNet/pos/y", 1, &robot_controller::delta_down_cam_y_cb, this);
		n.subscribe("/IARC/Obstacle/PitchPID", 1, &robot_controller::obstacle_pitch_cb, this);
		n.subscribe("/IARC/Obstacle/RollPID", 1, &robot_controller::obstacle_roll_cb, this);
	}

	//********************** callbacks *************************
	void ai_reset_cb(const std_msgs::Bool& msg);

	void current_altitude_cb(const std_msgs::Float32& msg);

	void target_altitude_cb(const std_msgs::Float32& msg);

	void yolo_x_cb(const std_msgs::Int16& msg);

	void yolo_y_cb(const std_msgs::Int16& msg);

	void delta_down_cam_x_cb(const std_msgs::Int16& msg);

	void delta_down_cam_y_cb(const std_msgs::Int16& msg);

	void obstacle_pitch_cb(const std_msgs::Float32& msg);

	void obstacle_roll_cb(const std_msgs::Float32& msg);

	//**************************** applying current values to pids *******************************
	double get_throttle_control();

	double get_roll_control();

	double get_pitch_control();

	double get_yaw_control();


	//***************************** custom ros loop ***********************
	void start_nav(bool rosOK);
	
};

#endif
