#include <IMUSensor.h>

IMUSensor::IMUSensor(ros::NodeHandle nodeHandle, std::string topic="/mavros/imu/data")
{
	_IMUSub = nodeHandle.subscribe(topic, 1, &IMUSensor::ImuCallback, this);
}
IMUSensor::IMUSensor(ros::NodeHandle nodeHandle)
{
	_IMUSub = nodeHandle.subscribe("/mavros/imu/data", 1, &IMUSensor::ImuCallback, this);
}

IMUSensor::~IMUSensor()
{
}

void IMUSensor::ImuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	_quaternionImu[0] = msg->orientation.x;
	_quaternionImu[1] = msg->orientation.y;
	_quaternionImu[2] = msg->orientation.z;
	_quaternionImu[3] = msg->orientation.w;
}

std::array<double, 4> IMUSensor::getQuaternionImu ()
{
	return _quaternionImu;
}

double IMUSensor::getQuaternionImuX ()
{
	return _quaternionImu[0];
}
double IMUSensor::getQuaternionImuY ()
{
	return _quaternionImu[1];
}
double IMUSensor::getQuaternionImuZ ()
{
	return _quaternionImu[2];
}
double IMUSensor::getQuaternionImuW ()
{
	return _quaternionImu[3];
}
