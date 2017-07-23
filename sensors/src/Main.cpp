#include "ros/ros.h"
#include "CompassSensor.h"
#include "sensors.h"
#include <cmath>
#include "compressed_image_repeater.h"

using namespace std;

ros::Publisher anglePub;

void CompassSensorCB(const std_msgs::Float32::ConstPtr& msg){
	float angle = msg->data;
	if(angle > 180){
		angle -= 360;
	}
	else if(angle < -180){
		angle += 360;
	}
	angle = angle * 3.1415926/180;

	std_msgs::Float32 new_msg;
	new_msg.data = angle;
	anglePub.publish(new_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "SensorMain");
	ros::NodeHandle n;
	ros::Subscriber compassSub = n.subscribe("/mavros/global_position/compass_hdg", 2, CompassSensorCB);
	anglePub = n.advertise<std_msgs::Float32>("/IARC/currentAngle", 1);
	LidarSensor sensorHokuyo(0, "/scan");
	LidarSensor sensorAltitudeLidar(1, "/terarangerone");

	imageEncoder forwardCam(1, "/sensor/compressed/forwardCam");
	ImageRepeater downCam("/usb_cam_down/image_raw");
	// CompassSensor com;

	ros::Rate loop_rate(120);
	while (ros::ok()){
		forwardCam.runOnce();

		ros::spinOnce();
		loop_rate.sleep();
	}
    return 0;
}

