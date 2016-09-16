#include <IMUSensor.h>

namespace IMUSensor
{
	IMUSensor(ros::NodeHandle n, std::String topic="/mavros/imu/data")
	{
		_IMUSub = n.subscribe(topic, 1, &IMUSensor::ImuCallback, this);
	}

	:~IMUSensor()
	{
		
	}

	void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg)
	{
		_quaternionImu[0] = msg->orientation.x;
		_quaternionImu[1] = msg->orientation.y;
		_quaternionImu[2] = msg->orientation.z;
		_quaternionImu[3] = msg->orientation.w;
	}

	std::array<double, 4> getQuaternionImu ()
	{
		return _quaternionImu;
	}

	double getQuaternionImuX ()
	{
		return _quaternionImu[0];
	}
	double getQuaternionImuY ()
	{
		return _quaternionImu[1];
	}
	double getQuaternionImuZ ()
	{
		return _quaternionImu[2];
	}
	double getQuaternionImuW ()
	{
		return _quaternionImu[3];
	}
}
