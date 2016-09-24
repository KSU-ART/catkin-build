#include "ros/ros.h"
#include "IMUSensor.h"
#include "LidarSensor.h"

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "SensorMain");
	ros::NodeHandle nodeHandle;
	IMUSensor sensorPixhawkIMU(nodeHandle);
	LidarSensor sensorAltitudeLidar(nodeHandle);

    return 0;
}

