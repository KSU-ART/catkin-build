/****************************************************************************************************
 * This is the class using the pid controller to 
 * generate output to the pixhawk.
 * 
 * MSG GUIDE:
 * Subscribed Topic:	Data Type					Usage
 * "setpoint"			geometry_msgs::Point		setpoint (desired location)
 * "curent_pose"		geometry_msgs::PoseStamped	current location
 * "manOverrideMsg"		std_msgs::Bool				true = override, false = disabled
 * "EMERGENCY_LAND"		std_msgs::Bool				true = emergencyland, false = normal
 * "modeMsg"			std_msgs::Bool				true = altitude hold, false = stabilize;
 * "retractMsg"			std_msgs::Bool				true = retracts down, false = up;
 * "pid_XY"				std_msgs::Int32MultiArray	{p, i, d, min, max}
 * "pid_z"				std_msgs::Int32MultiArray	{p, i, d, min, max}
 * 
 * PRIORITY OF CONTROL:
 * physical override switch overrides software switch
 * mannual_override overrides emergency_land
 * emergency_land overrides ai_pilot
 * 
 * 
 * Mannual Override Switch: 
 * channel 9 (8 in {0-8}) HIGH_PWM is land
 * 
 **************************************************************************************************/
 
#include "flight_definitions.h"

#include "ros/ros.h"
#include <PIDController.h>
#include <math.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/RCIn.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32MultiArray.h"


