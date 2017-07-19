#include "ros/ros.h"
#include "sensors.h"

using namespace std;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "SensorMain");
	LidarSensor sensorHokuyo(0, "/scan");
	LidarSensor sensorAltitudeLidar(1, "/terarangerone");
	// imageEncoder forwardCam(1, "/sensor/compressed/forwardCam");
	imageEncoder forwardCam(1, "/sensor/compressed/downCam");

	ros::Rate loop_rate(120);
	while (ros::ok())
	{
		forwardCam.runOnce();

		ros::spinOnce();
		loop_rate.sleep();
	}
    return 0;
}

