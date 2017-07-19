#include "ros/ros.h"
#include "sensors.h"
#include <cmath>

using namespace std;

float convertQuat(std::array<double, 4> quat){
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

