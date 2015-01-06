#include "ros/ros.h"
#include <ros/console.h>
#include "px_comm/OpticalFlow.h"
#include <roscopter/RC.h>
#include <roscopter/VFR_HUD.h>
#include <math.h>
#include <spsuart/PIDloop.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>

using namespace std;
using namespace ros;

sensor_msgs::Imu globalImuValue;
int velCounter=0;
double xAccelAccumulator=0;
double yAccelAccumulator=0;
double zAccelAccumulator=0;
double currentVelX=0;
double currentVelY=0;
double currentVelZ=0;

void hudCallback(const roscopter::VFR_HUD::ConstPtr& msg){
	msg->heading;
}

void phidgetCallback(const sensor_msgs::Imu::ConstPtr& msg){
	globalImuValue.angular_velocity.x= msg->angular_velocity.x;
	globalImuValue.angular_velocity.y= msg->angular_velocity.y;
	globalImuValue.angular_velocity.z= msg->angular_velocity.z;
	globalImuValue.linear_acceleration.x=msg->linear_acceleration.x;
	globalImuValue.linear_acceleration.y=msg->linear_acceleration.y;
	globalImuValue.linear_acceleration.z=msg->linear_acceleration.z;
	
	//f(velCounter<3){
	//	xAccelAccumulator+=msg->linear_acceleration.x;
	//	yAccelAccumulator+=msg->linear_acceleration.y;
	//	zAccelAccumulator+=msg->linear_acceleration.z;
	//	velCounter++;
	//}
	//else{
	//	globalImuValue.linear_acceleration.x=currentVelX + msg->linear_acceleration.x*(1/120);
	//	currentVelX = currentVelX + msg->linear_acceleration.x*(1/120);
	//	globalImuValue.linear_acceleration.y=currentVelY + msg->linear_acceleration.y*(1/120);
	//	currentVelY = currentVelY + msg->linear_acceleration.y*(1/120);
	//	globalImuValue.linear_acceleration.z=currentVelZ + msg->linear_acceleration.x*(1/120);
	//	currentVelZ = currentVelZ + msg->linear_acceleration.z*(1/120);
	//	xAccelAccumulator=0;
	//	yAccelAccumulator=0;
	//	zAccelAccumulator=0;
	//	velCounter=0;
	//}
}

void flowCallback(const px_comm::OpticalFlow::ConstPtr& msg){

}	

int main(int argc, char **argv)
{
 
  init(argc, argv, "sensor_fusion");
  
  NodeHandle n;
  //Ignoring pixhawk and optical flow data for now and est vel will be 
  // estimated solely by the phidget 
  //ros::Subscriber subHud = n.subscribe("/vfr_hud", 1, hudCallback);
  //ros::Subscriber subflow = n.subscribe("/px4flow/opt_flow", 1, flowCallback);
  Subscriber subPhidget = n.subscribe("/imu/data_raw", 1, phidgetCallback);
  Publisher  est_vel = n.advertise<geometry_msgs::Twist>("est_vel", 1);
  Rate r(45);
  geometry_msgs::Twist currentEstTwist;

	while(ros::ok()){
		currentEstTwist.angular.x=globalImuValue.angular_velocity.x;
		currentEstTwist.angular.y=globalImuValue.angular_velocity.y;
		currentEstTwist.angular.z=globalImuValue.angular_velocity.z;
		currentEstTwist.linear.x=globalImuValue.linear_acceleration.x;
		currentEstTwist.linear.y=globalImuValue.linear_acceleration.y;
		currentEstTwist.linear.z=globalImuValue.linear_acceleration.z;
		est_vel.publish(currentEstTwist);
		spinOnce();
		r.sleep();	
	}

  return 0;
}

