#ifndef LIDAR_SENSOR_CLASS_H
#define LIDAR_SENSOR_CLASS_H

#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>

class LidarSensor
{
private:
	ros::Subscriber _LidarSub;
	
public:
	/// Pre: requires a nodehandle object
	/// Post: 
	LidarSensor(ros::NodeHandle n);
	
	/// Post: deconstructs any pointers
	~LidarSensor();
	
	/// Post: Handles the lidar messages
	void LidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
};

#endif
