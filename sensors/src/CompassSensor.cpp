#include <CompassSensor.h>

void CompassSensor::CompassSensorCB(const std_msgs::Float32::ConstPtr& msg){
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
	_anglePub.publish(new_msg);
}
