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

PIDController* xRateController = new PIDController();
PIDController* yRateController = new PIDController();
double xLinearVelocity = 0;
double yLinearVelocity = 0;
double xLastValue = 0;
double yLastValue = 0;
double lastTime = 0;
double nowTime = 0;

double startTime=0.0;
double stopTime=0.0;
int throttle=0;
int pitch=0;
int roll=0;
int yaw=0;
int cameraRoll=1500;
int cameraTilt=1500;
double highest=0.0;

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

	nowTime = ros::Time::now().toSec();
	xLinearVelocity = (nowTime - lastTime) * ((globalImuValue.linear_acceleration.x - xLastValue)/2); // Trapezoid rule: nInt(f,a,b) = (b - a) * (f(b) - f(a))/2
	yLinearVelocity = (nowTime - lastTime) * ((globalImuValue.linear_acceleration.y - yLastValue)/2);
	cout << "X Vel: " << xLinearVelocity * 1000 << endl;
	cout << "Y Vel: " << yLinearVelocity * 1000 << endl << endl;
	xLastValue = globalImuValue.linear_acceleration.x;
	yLastValue = globalImuValue.linear_acceleration.y;
	lastTime = nowTime;
}

void flowCallback(const px_comm::OpticalFlow::ConstPtr& msg){

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
 
  init(argc, argv, "phidget_based_flight");
  
  NodeHandle n;
  ros::Subscriber subHud = n.subscribe("/vfr_hud", 1, hudCallback);
  ros::Subscriber subflow = n.subscribe("/px4flow/opt_flow", 1, flowCallback);
  Subscriber subPhidget = n.subscribe("/imu/data_raw", 1, phidgetCallback);
  Publisher  send_rc = n.advertise<roscopter::RC>("send_rc", 1);
  startTime=ros::Time::now().toSec();
  stopTime=ros::Time::now().toSec() + ros::Duration(120).toSec();
  roscopter::RC msg; 
  
  // Init X/Y PID rate controllers
  xRateController->setGains(0, 0, 0);
  yRateController->setGains(0, 0, 0);
  xRateController->init();
  yRateController->init();
  xRateController->targetSetpoint(0);
  yRateController->targetSetpoint(0);
  
  Rate r(45);
	

	while(ros::ok()){
		msg.channel[0]=roll;
	    msg.channel[1]=pitch;
        msg.channel[2]=throttle;
        msg.channel[3]=yaw;
    	msg.channel[4]=0;
    	msg.channel[5]=0;
	    msg.channel[6]=cameraTilt;
    	msg.channel[7]=cameraRoll;
    	updatePID();
		spinOnce();
		r.sleep();	
	}

  return 0;
}

