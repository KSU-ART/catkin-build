#include <LidarSensor.h>

LidarSensor::LidarSensor(ros::NodeHandle nodeHandle, std::string topic="/scan2")
{
	_LidarSub = nodeHandle.subscribe(topic, 1, &LidarSensor::LidarCallback, this);
}

LidarSensor::~LidarSensor()
{
}

void LidarSensor::LidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
}
