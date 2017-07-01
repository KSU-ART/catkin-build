#include <LidarSensor.h>

LidarSensor::LidarSensor(int type, std::string topic="/scan")
{
	if (type == 0){
		_LidarSub = nh.subscribe(topic, 1, &LidarSensor::LidarCallback0, this);
	}
	else{
		_LidarSub = nh.subscribe(topic, 1, &LidarSensor::LidarCallback1, this);
	}
	_LidarPubs = std::vector<ros::Publisher>();
	if(type == 0){
		// use Hokuyo
		obstaclePublishers();
	}
	else{
		// use point lidar
		altitudePublishers();
	}
}

LidarSensor::~LidarSensor()
{
}


void LidarSensor::obstaclePublishers()
{
	_LidarPubs.push_back( nh.advertise<std_msgs::Float32>("/IARC/Obstacle/dist", 1) );
	_LidarPubs.push_back( nh.advertise<std_msgs::Float32>("/IARC/Obstacle/angle", 1) );
}

void LidarSensor::altitudePublishers()
{
	_LidarPubs.push_back( nh.advertise<std_msgs::Float32>("/IARC/currentAltitude", 1) );
}

void LidarSensor::LidarCallback0(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	// Hokuyo
	// publish angle and distance of closest object
	// float[] ranges = msg->ranges;
	float minElement = msg->ranges[0];
	int iterCount = 0;
	int minElementCount = 0;
	for(std::vector<float>::const_iterator it = msg->ranges.begin(); it != msg->ranges.end(); ++it){
		if(*it < minElement){
			minElement = *it;
			minElementCount = iterCount;
		}
		iterCount++;
	}
	float startAngle = msg->angle_min;
	float increment = msg->angle_increment;

	std_msgs::Float32 closObs;
	std_msgs::Float32 obsAng;
	closObs.data = minElement;
	obsAng.data = startAngle + increment * minElementCount;
	
	_LidarPubs[0].publish(closObs);
	_LidarPubs[1].publish(obsAng);
}

void LidarSensor::LidarCallback1(const sensor_msgs::Range::ConstPtr& msg)
{
	// Point lidar
	// publish altitude
	float value = msg->range;
	// any calibrations are done to value here
	double offset = 0.17876921059;
	value = value - offset;
	
	std_msgs::Float32 alt;
	alt.data = value;
	_LidarPubs[0].publish(alt);
}
