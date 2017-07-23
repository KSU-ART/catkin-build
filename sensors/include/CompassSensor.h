#ifndef COMPASS_CLASS_H
#define COMPASS_CLASS_H

#include "ros/ros.h"
#include <array>
#include <string>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>

class CompassSensor
{
private:
	ros::NodeHandle n;
	ros::Subscriber _compassSub;
	ros::Publisher _anglePub;
	
public:
	/// Pre: requires a nodehandle object
	/// Post: 
	CompassSensor(){
		ros::Subscriber compassSub = n.subscribe("/mavros/global_position/compass_hdg", 1, &CompassSensor::CompassSensorCB, this);
		_anglePub = n.advertise<std_msgs::Float32>("/IARC/currentAngle", 1);
	}
	
	void CompassSensorCB(const std_msgs::Float32::ConstPtr& msg);
};
#endif
