#ifndef LIDAR_SENSOR_CLASS_H
#define LIDAR_SENSOR_CLASS_H

#include "ros/ros.h"
#include <string>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <vector>

class LidarSensor
{
private:
	ros::Subscriber _LidarSub;
	std::vector<ros::Publisher> _LidarPubs;
public:
	/// Pre: requires a nodehandle object
	/// Post: 
	LidarSensor(ros::NodeHandle, int, std::string);
	
	/// Post: deconstructs any pointers
	~LidarSensor();

	/// Post: initializes publishers
	void obstaclePublishers();
	void altitudePublishers();
	
	/// Post: Handles the lidar messages
	void LidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void LidarCallback(const sensor_msgs::Range::ConstPtr& msg);
};

#endif
