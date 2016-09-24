#ifndef IMU_SENSOR_CLASS_H
#define IMU_SENSOR_CLASS_H

#include "ros/ros.h"
#include <array>
#include <string>
#include <sensor_msgs/Imu.h>

class IMUSensor
{
private:
	ros::Subscriber _IMUSub;
	std::array<double, 4> _quaternionImu;
	
public:
	/// Pre: requires a nodehandle object
	/// Post: 
	IMUSensor(ros::NodeHandle);
	IMUSensor(ros::NodeHandle, std::string);
	
	/// Post: deconstructs any pointers
	~IMUSensor();
	
	/// Post: Handles the imu messages
	void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg);
	
	/// Post: Gets the current quaternion values
	std::array<double, 4> getQuaternionImu ();
	
	double getQuaternionImuX ();
	double getQuaternionImuY ();
	double getQuaternionImuZ ();
	double getQuaternionImuW ();
};
#endif
