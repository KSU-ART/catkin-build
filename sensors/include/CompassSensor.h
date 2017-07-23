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
	
	void CompassSensorCB(const std_msgs::Float32::ConstPtr& msg){
		float angle = msg->data;
		if(angle > 180){
			angle -= 360;
		}
		else if(angle < -180){
			angle += 360;
		}
		angle = angle * 3.1415926/180;

		std_msgs::Float32 new_msg;
		new_msg.data = angle;
		_anglePub.publish(new_msg);
	}
};
#endif
