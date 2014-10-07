#include "ros/ros.h"
#include <ros/console.h>
//#include "px_comm/OpticalFlow.h"
#include <roscopter/RC.h>
#include <roscopter/VFR_HUD.h>
#include <ros_opencv/Diffmessage.h>
#include <ros_opencv/TrackingPoint.h>
#include <math.h>
#include <spsuart/PIDloop.h>

using namespace std;
int throttle;
int pitch;
int roll;
int yaw=0;
int pan=1500;
int tilt=1500;
int compassValue;
bool startUpPhase=true;
roscopter::RC rcValues;
ros::Publisher rc_pub;
double targetAlt=2.5;
int retracts=1000;

//PID Stuff
PIDloop yLoop = PIDloop(.75, 0, 0, 240); //You should play around with the .1 values and find what works, these are just place holders.  0 is totally acceptable, especially for I
PIDloop xLoop = PIDloop(.1, 0, 0, 320);

void deployLandingGear(){
retracts=1000;
}
void retractLandingGear(){
retracts=2000;
}


void flowHud(const roscopter::VFR_HUD::ConstPtr& msg){
compassValue=msg->heading;
if(msg->alt<targetAlt-.1)
throttle=1700;
else if(msg->alt>targetAlt+.1)
throttle=1300;
else
throttle=1500;

if(msg->alt<.6)
deployLandingGear();
else if(msg->alt>1.5)
retractLandingGear();
}

void callback(const ros_opencv::TrackingPoint::ConstPtr& msgs){
double x=msgs->pointX;
double y=msgs->pointY;


if(x<0){
roll=rcValues.channel[0];
}
else{

if(x>325)
{
double diffValue=50.0*((x-320.0)/320.0);
diffValue= round(diffValue);
roll=1500-diffValue;
if((pan+diffValue)<=2000)
pan+=diffValue;

}
else if(x<315)
{
double diffValue=50.0*((320.0-x)/320.0);
diffValue= round(diffValue);
roll=1500+diffValue;
if((pan-diffValue)>=1000)
pan-=diffValue;

}
//else{
//if(pan<1450){
//double diffValue=75.0*((450.0-(1000-pan))/450.0);
//diffValue= round(diffValue);
//roll=1500-diffValue;
//}
//else if(pan>1550){
//double diffValue=75.0*((pan-1550)/450.0);
//diffValue= round(diffValue);
//roll=1500+diffValue;
//}
//else{
//roll=1500;	
//}
//}
}



if(y<0){
pitch=rcValues.channel[1];
}
else{

if(y<230){
double diffValue=50.0*((240.0-y)/240.0);
diffValue= round(diffValue);
pitch=1500+diffValue;
if((tilt+diffValue)<=2000)
tilt+=diffValue;

}
else if(y>250){
double diffValue=50.0*((y-240.0)/240.0);
diffValue= round(diffValue);
pitch=1500-diffValue;
if((tilt-diffValue)>=1350)
tilt-=diffValue;

}
//else{
//if(tilt<1450){
//double diffValue=75.0*((450.0-(1000-tilt))/450.0);
//diffValue= round(diffValue);
//pitch=1500-diffValue;
//}
//else if(tilt>1550){
//double diffValue=75.0*((tilt-1550)/450.0);
//diffValue= round(diffValue);
//pitch=1500+diffValue;
//}
//else{
//pitch=1500;	
//}
//}

}

if(startUpPhase){
pitch=1425;
roll=1500;
}

}

void initrcneutral(){
	rcValues.channel[0]=1500;
	rcValues.channel[1]=1500;
	rcValues.channel[2]=1500;
	rcValues.channel[3]=1500;
 	rcValues.channel[4]=1500; 	
	rcValues.channel[5]=0;
	rcValues.channel[6]=0;
	rcValues.channel[7]=0;
}

void takeOff(){
    roscopter::RC msg;
    msg.channel[0]=0;
    msg.channel[1]=0;
    msg.channel[2]=1700;
    msg.channel[3]=0;
    msg.channel[4]=1100;
    msg.channel[5]=0;
    msg.channel[6]=0;
    msg.channel[7]=0;
    rc_pub.publish(msg);
}

void land(){
	roscopter::RC msg;
	cout<<"Initiating Land"<<endl;
    msg.channel[0]=0;
    msg.channel[1]=0;
    msg.channel[2]=1500;
    msg.channel[3]=0;
    msg.channel[4]=1900;
    msg.channel[5]=0;
    msg.channel[6]=0;
    msg.channel[7]=0;
    rc_pub.publish(msg);
    ros::Duration(5).sleep();
}

void pubZeros(){
	roscopter::RC msg;
	msg.channel[0]=0;
    msg.channel[1]=0;
    msg.channel[2]=0;
    msg.channel[3]=0;
    msg.channel[4]=0;
    msg.channel[5]=0;
    msg.channel[6]=0;
    msg.channel[7]=0;
    rc_pub.publish(msg);
    cout<<"Closing"<<endl;
}



int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "follow_irobot");
  
  ros::NodeHandle n;
 
  ros::Subscriber subHud = n.subscribe("/vfr_hud", 1, flowHud);
  ros::Subscriber subbot = n.subscribe("irobotinfo", 1, callback);
  rc_pub = n.advertise<roscopter::RC>("send_rc", 1);
	
  initrcneutral();	
  takeOff();

    ros::Rate r(45);
    double startTime=ros::Time::now().toSec();
    double stopTime=ros::Time::now().toSec() + ros::Duration(180).toSec();
    int i=0;
    roscopter::RC msg; 
	while(ros::ok()){
		if(i>100)
		startUpPhase=false;
		if(ros::Time::now().toSec()>stopTime)
			ros::shutdown();
        msg.channel[0]=roll;
	    msg.channel[1]=pitch;
        msg.channel[2]=throttle;
        msg.channel[3]=yaw;
    	msg.channel[4]=0;
    	msg.channel[5]=0;
	    msg.channel[6]=tilt;
    	msg.channel[7]=retracts;
    	
        rc_pub.publish(msg);
		ros::spinOnce();
		i++;
		r.sleep();	
	}

	land();
	pubZeros();

  return 0;
}

