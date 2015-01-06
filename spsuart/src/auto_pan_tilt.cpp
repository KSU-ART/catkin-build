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
roscopter::RC rcValues;
ros::Publisher rc_pub;



void flowHud(const roscopter::VFR_HUD::ConstPtr& msg){
//compassValue=msg->heading;

}

void callback(const ros_opencv::TrackingPoint::ConstPtr& msgs){
double x=msgs->pointX;
double y=msgs->pointY;


if(x<0){

}
else{

if(x>325)
{
double diffValue=75.0*((x-320.0)/320.0);
diffValue= round(diffValue);
if((pan+diffValue)<=1650)
	pan+=diffValue;

}
else if(x<315)
{
double diffValue=75.0*((320.0-x)/320.0);
diffValue= round(diffValue);
if((pan-diffValue)>=1225)
	pan-=diffValue;

}
}

if(y<0){

}
else{

if(y<230){
double diffValue=75.0*((240.0-y)/240.0);
diffValue= round(diffValue);
if((tilt+diffValue)<=1850)
	tilt+=diffValue;

}
else if(y>250){
double diffValue=75.0*((y-240.0)/240.0);
diffValue= round(diffValue);
if((tilt-diffValue)>=1250)
	tilt-=diffValue;
}

}

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
  ros::Subscriber subbot = n.subscribe("tracking_point", 1, callback);
  rc_pub = n.advertise<roscopter::RC>("send_rc", 1);

    ros::Rate r(200);

    roscopter::RC msg; 
	while(ros::ok()){
        msg.channel[0]=1100;
	    msg.channel[1]=1100;
        msg.channel[2]=1100;
        msg.channel[3]=1100;
    	msg.channel[4]=1100;
    	msg.channel[5]=1100;
	    msg.channel[6]=tilt;
    	msg.channel[7]=pan;
    	
        rc_pub.publish(msg);
		ros::spinOnce();
		r.sleep();	
	}
	pubZeros();

  return 0;
}

