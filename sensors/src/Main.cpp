#include "ros/ros.h"
#include "sensors.h"

using namespace std;

void system_loop()
{
	ros::Rate loop_rate(120);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "SensorMain");
	LidarSensor sensorHokuyo(0, "/scan");
	LidarSensor sensorAltitudeLidar(1, "/terarangerone");
	system_loop();
    return 0;
}

