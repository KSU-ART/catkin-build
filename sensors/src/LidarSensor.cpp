#include <LidarSensor.h>

namespace LidarSensor
{
	LidarSensor(ros::NodeHandle nodeHandle, std::string topic="/scan2")
	{
		_IMUSub = n.subscribe(topic, 1, &sensor_processor::hokuyo_sub, this);
	}

	~LidarSensor()
	{
	}

	void LidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
	{
	}
}
