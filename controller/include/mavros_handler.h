/****************************************************************************************************
 * mavros_handler class
 * holds all the rc_msg types such as ai, land, and release(manual)
 * 
 * checks for the space bar key to be pressed durring a flight to set it to land mode
 *
 * subscribes to the rc controller for manual override
 * 
 * 
 **************************************************************************************************/
#ifndef MAVROS_HANDLER_H
#define MAVROS_HANDLER_H
#include <iostream>
#include "ros/ros.h"
#include <termios.h>
#include "flight_definitions.h"
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/RCIn.h>
#include "std_msgs/Bool.h"

class mavros_handler{
private:
	ros::NodeHandle n;
	ros::Publisher rc_pub, ai_reset_pub;

	mavros_msgs::OverrideRCIn RC_MSG;

	enum flight_mode_enum{
		ai,
		land,
		manual
	};

	flight_mode_enum flight_mode = ai;

	char landChar;

	bool LAND_TOGGLE, DEBUG;

public:
	mavros_handler(){
		LAND_TOGGLE = false;
		DEBUG = true;

		// just anything not space
		landChar = 'f';

		rc_pub = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);
		ai_reset_pub = n.advertise<std_msgs::Bool>("/IARC/ai_reset", 1);

		n.subscribe("/mavros/rc/in", 1, &mavros_handler::RCIn_callback, this);
	}

	void release_msg_channels();

	void land_msg_channels();
	
	void ai_msg_channels(int roll, int pitch, int yaw, int throttle);

	int getNonBlocking();

    /// update loop for mavros handler
	void update_loop(int roll, int pitch, int yaw, int throttle);

    /// releases all channels
    void disable_rc_overide();

	void RCIn_callback(const mavros_msgs::RCIn& msg);

};

#endif