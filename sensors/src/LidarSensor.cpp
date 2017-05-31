#include <LidarSensor.h>

LidarSensor::LidarSensor(ros::NodeHandle nodeHandle, int type, std::string topic="/scan")
{
	_LidarSub = nodeHandle.subscribe(topic, 1, &LidarSensor::LidarCallback, this);
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

void obstaclePublishers()
{
	_LidarPubs.push_back( nodeHandle.advertise<std_msgs::float32>("closestObstacle", 1) );
	_LidarPubs.push_back( nodeHandle.advertise<std_msgs::float32>("obstacleAngle", 1) );
}

void altitudePublishers()
{
	_LidarPubs.push_back( nodeHandle.advertise<std_msgs::float32>("altitudeLidar", 1) );
}

void LidarSensor::LidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	// Hokuyo
	// publish angle and distance of closest object
	float minElement = msg->ranges[0];
	int iterCount = 0;
	int minElementCount = 0;
	for(std::vector<int>::const_iterator it = msg->ranges.begin(); it != msg->ranges.end(); ++it){
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

void LidarSensor::LidarCallback(const sensor_msgs::Range::ConstPtr& msg)
{
	// Point lidar
	// publish altitude
	float value = msg->range;
	// any calibrations are done to value here

	std_msgs::Float32 alt;
	alt.data = value;
	_LidarPubs[0].publish(alt);
}
