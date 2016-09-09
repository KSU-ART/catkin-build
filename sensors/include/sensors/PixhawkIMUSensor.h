#ifndef PIXHAWK_IMU_SENSOR_CLASS_H
#define PIXHAWK_IMU_SENSOR_CLASS_H

#include "ros/ros.h"

class PixhawkIMUSensor
{
private:
	ros::Subscriber _pixhawkIMUSub;
	
public:
	/// Pre: requires a nodehandle object
	/// Post: 
	PixhawkIMUSensor(ros::NodeHandle n);
	
	/// Post: deconstructs any pointers
	~PixhawkIMUSensor();
	
	
};

#endif
