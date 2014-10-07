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

void hudCallback(const roscopter::VFR_HUD::ConstPtr& msg){
	msg->heading;
}

void phidgetCallback(const sensor_msgs::Imu::ConstPtr& msg){
	globalImuValue.angular_velocity.x= msg->angular_velocity.x;
	globalImuValue.angular_velocity.y= msg->angular_velocity.y;
	globalImuValue.angular_velocity.z= msg->angular_velocity.z;
}

void flowCallback(const px_comm::OpticalFlow::ConstPtr& msg){

}	

int main(int argc, char **argv)
{
 
  init(argc, argv, "sensor_fusion");
  
  NodeHandle n;
  //Ignoring pixhawk and sendTransformoptical flow data for now and est vel will be 
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
		//currentEstTwist.linear.x=globalImuValue.linear_velocity.x;
		//currentEstTwist.linear.y=globalImuValue.linear_velocity.y;
		//currentEstTwist.linear.z=globalImuValue.linear_velocity.z;
		spinOnce();
		r.sleep();	
	}

  return 0;
}

