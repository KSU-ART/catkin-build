#include "ros/ros.h"
#include "sensors.h"

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "SensorMain");
	ros::NodeHandle nodeHandle;
	LidarSensor sensorHokuyo(nodeHandle, 0, "/scan");
	LidarSensor sensorAltitudeLidar(nodeHandle, 1, "/terarangerone");

    return 0;
}

