#ifndef IMU_SENSOR_CLASS_H
#define IMU_SENSOR_CLASS_H

#include "ros/ros.h"
#include <array>
#include <string>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>

class IMUSensor
{
private:
	ros::NodeHandle nodeHandle;
	ros::Subscriber _IMUSub;
	ros::Publisher _anglePub;
	std::array<double, 4> _quaternionImu;
	double _angle;
	
public:
	/// Pre: requires a nodehandle object
	/// Post: 
	IMUSensor();
	IMUSensor(std::string);
	
	/// Post: deconstructs any pointers
	~IMUSensor();
	
	/// Post: Handles the imu messages
	void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg);
	
	/// Post: Gets the current quaternion values
	std::array<double, 4> getQuaternionImu ();

	double convertQuat(std::array<double, 4> quat);
	void computeAngle();

	void runOnce();

	double getAngle();
	
	double getQuaternionImuX ();
	double getQuaternionImuY ();
	double getQuaternionImuZ ();
	double getQuaternionImuW ();
};
#endif
