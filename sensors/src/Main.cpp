#include "ros/ros.h"
#include "IMUSensor.h"
#include "sensors.h"
#include <cmath>
#include "compressed_image_repeater.h"

using namespace std;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "SensorMain");
	LidarSensor sensorHokuyo(0, "/scan");
	LidarSensor sensorAltitudeLidar(1, "/terarangerone");
	imageEncoder forwardCam(1, "/sensor/compressed/forwardCam");
	ImageRepeater downCam("/usb_cam_down/image_raw");
	IMUSensor imu;

	ros::Rate loop_rate(120);
	while (ros::ok())
	{
		forwardCam.runOnce();

		imu.runOnce();

		ros::spinOnce();
		loop_rate.sleep();
	}
    return 0;
}

