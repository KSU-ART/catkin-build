#include <IMUSensor.h>

IMUSensor::IMUSensor(std::string topic="/mavros/imu/data")
{
	_IMUSub = nodeHandle.subscribe(topic, 1, &IMUSensor::ImuCallback, this);
	_anglePub = nodeHandle.advertise<std_msgs::Float32>("/IARC/currentAngle", 1);
}
IMUSensor::IMUSensor()
{
	_IMUSub = nodeHandle.subscribe("/mavros/imu/data", 1, &IMUSensor::ImuCallback, this);
	_anglePub = nodeHandle.advertise<std_msgs::Float32>("/IARC/currentAngle", 1);
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

double IMUSensor::convertQuat(std::array<double, 4> quat)
{
	// made by Kyle
	// returns double representing radians rotated in x-z plane between -PI and PI
	// with the positive direction being counter-clockwise
	// don't ask me how it works. it's magic
	double test = quat[0]*quat[1] + quat[2]*quat[3];
	if(test > 0.499){
		return 2*atan2(quat[0], quat[3]);
	}
	else if(test < -0.499){
		return -2*atan2(quat[0], quat[3]);
	}
	return atan2(2*quat[1]*quat[3] - 2*quat[0]*quat[2], 1 - 2*pow(quat[1],2.0) - 2*pow(quat[2],2.0));
}

void IMUSensor::computeAngle()
{
	_angle = convertQuat(_quaternionImu);
}

void IMUSensor::runOnce()
{
	computeAngle();
	std_msgs::Float32 msg;
	msg.data = _angle;
	_anglePub.publish(msg);
}

double IMUSensor::getAngle()
{
	return _angle;
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
