#ifndef COMPASS_CLASS_H
#define COMPASS_CLASS_H

#include "ros/ros.h"
#include <array>
#include <string>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

class CompassSensor
{
private:
	ros::NodeHandle n;
	ros::Subscriber compassSub;
	ros::Publisher anglePub;
	
public:
	/// Pre: requires a nodehandle object
	/// Post: 
	CompassSensor(){
		compassSub = n.subscribe("/mavros/global_position/compass_hdg", 1, &CompassSensor::CompassSensorCB, this);
		anglePub = n.advertise<std_msgs::Float32>("/IARC/currentAngle", 1);
	}
	
	void CompassSensorCB(const std_msgs::Float64::ConstPtr& msg){
		std::cout << "got msg" <<std::endl;
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
		anglePub.publish(new_msg);
	}
};
#endif
