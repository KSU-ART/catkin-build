#include "ros/ros.h"
#include <ros/console.h>
#include "px_comm/OpticalFlow.h"
#include <roscopter/RC.h>
#include <roscopter/VFR_HUD.h>
#include <math.h>
#include <spsuart/PIDloop.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include "PIDController.h"
#include "SimpleSerial.cpp"

using namespace std;
using namespace ros;

sensor_msgs::Imu globalImuValue;

PIDController* rateController = new PIDController();
double linearVelocity = 0;
double lastValue = 0;
double lastTime = 0;
double nowTime = 0;

double startTime=0.0;
double stopTime=0.0;

void phidgetCallback(const sensor_msgs::Imu::ConstPtr& msg){
	globalImuValue.angular_velocity.x= msg->angular_velocity.x;
	globalImuValue.angular_velocity.y= msg->angular_velocity.y;
	globalImuValue.angular_velocity.z= msg->angular_velocity.z;
	globalImuValue.linear_acceleration.x=msg->linear_acceleration.x;
	globalImuValue.linear_acceleration.y=msg->linear_acceleration.y;
	globalImuValue.linear_acceleration.z=msg->linear_acceleration.z;

	nowTime = ros::Time::now().toSec();
	linearVelocity = (nowTime - lastTime) * ((globalImuValue.linear_acceleration.x - lastValue)/2); // Trapezoid rule: nInt(f,a,b) = (b - a) * (f(b) - f(a))/2
	cout << "Vel: " << linearVelocity * 1000 << endl;
	lastValue = globalImuValue.linear_acceleration.x;
	lastTime = nowTime;
}

void updatePID(){
	//lastTime = nowTime;
	//nowTime = ros::Time::now().toSec();
	//xLinearVelocity += globalImuValue.linear_acceleration.x * (nowTime - lastTime);
	//yLinearVelocity += globalImuValue.linear_acceleration.y * (nowTime - lastTime);
	//cout << "X Vel: " << xLinearVelocity << endl;
	//cout << "Y Vel: " << yLinearVelocity << endl;
	//xRateController->calc(xLinearVelocity, nowTime);
	//yRateController->calc(yLinearVelocity, nowTime);
}

int main(int argc, char **argv)
{
 
  init(argc, argv, "velocity_test");
  
  NodeHandle n;
  Subscriber subPhidget = n.subscribe("/imu/data_raw", 1, phidgetCallback);
  startTime=ros::Time::now().toSec();
  stopTime=ros::Time::now().toSec() + ros::Duration(120).toSec();
  SimpleSerial serial("/dev/ttyACM2",115200);
  serial.writeString("2000/n");
  
  // Init PID rate controller
  rateController->setGains(0, 0, 0);
  rateController->init();
  rateController->targetSetpoint(0);

  
  Rate r(45);
	

	while(ros::ok()){
		
    	updatePID();
		spinOnce();
		r.sleep();	
	}

  return 0;
}

