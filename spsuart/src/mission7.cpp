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
int startUpTimer=0;
int compassValue;
int yawGoal=-1;
double desiredAlt=2.0;
bool startUpPhase=true;
bool scanForIrobots=true;
bool ascending=true;
double startTime;
double stopTime;
int retracts;
roscopter::RC rcValues;
ros::Publisher rc_pub;

void deployLandingGear(){
retracts=1000;
}
void retractLandingGear(){
retracts=2000;
}

void flowHud(const roscopter::VFR_HUD::ConstPtr& msg){
	compassValue=msg->heading;
	if(msg->alt<desiredAlt-.1)
		throttle=1700;
	else if(msg->alt>desiredAlt+.1)
		throttle=1300;
	else
		throttle=1500;
		
	if(msg->alt<.5)
		deployLandingGear();
	else if(msg->alt>1)
		retractLandingGear();
}

void callbackDownwardCam(const ros_opencv::TrackingPoint::ConstPtr& msgs){
double x=msgs->pointX;
double y=msgs->pointY;

//If No TrackingPoint on downard cam Send neutral values
	if(x<0 || y<0){
		roll =1500;
		pitch =1500;
		return;
	}
	
	roll= 1440;
	if(x>600){
		if(compassValue<=270)
			yawGoal=compassValue+90;
		else
			yawGoal=(compassValue+90)-360;
			
			if(yawGoal==360)
			yawGoal=359;
			if(yawGoal==0)
			yawGoal=1;
	}
//Y Pixel
		if(y<220){
			double diffValue=50.0*((240.0-y)/240.0);
			diffValue= round(diffValue);
			pitch=1500-diffValue;
		}
		else if(y>260){
			double diffValue=50.0*((y-240.0)/240.0);
			diffValue= round(diffValue);
			pitch=1500+diffValue;
		}
}

void callbackXtion(const ros_opencv::TrackingPoint::ConstPtr& msgs){
double x=msgs->pointX;
double y=msgs->pointY;
if(scanForIrobots){
	
		if(x>0 && y>0){
		scanForIrobots=false;
		return;
	}
	
	if(ascending && (tilt+25)<=2000)
		tilt+=25;
	else if(tilt>=2000)
		ascending=false;
		
	if(!ascending && (tilt-25)<=1000)
		tilt-=25;
	else if(tilt<=1000)
		ascending=true;
}
else{
//If No TrackingPoint on Xtion scan field
	if(x<0 || y<0){
		scanForIrobots=true;
		return;
	}

	scanForIrobots=false;
//X Pixel
		if(x>330){
			double diffValue=100.0*((x-320.0)/320.0);
			diffValue= round(diffValue);
				if((pan+diffValue)<=2000)
					pan+=diffValue;
		}
		else if(x<310){
			double diffValue=100.0*((320.0-x)/320.0);
			diffValue= round(diffValue);
			if((pan-diffValue)>=1000)
				pan-=diffValue;
		}
//Y Pixel
		if(y<230){
			double diffValue=100.0*((240.0-y)/240.0);
			diffValue= round(diffValue);
				if((tilt+diffValue)<=2000)
					tilt+=diffValue;
		}
		else if(y>250){
			double diffValue=100.0*((y-240.0)/240.0);
			diffValue= round(diffValue);
				if((tilt-diffValue)>=1000)
					tilt-=diffValue;
		}
	}
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

void interruptChecks(){
		if(startUpTimer<60){
			pitch=1400;
			roll=1510;
		}
		if(yawGoal==-1 || (compassValue<=yawGoal+1 && compassValue>=yawGoal-1)){
			yaw=0;
			yawGoal=-1;
		}	
	    if(yawGoal!=-1 && compassValue!=yawGoal)
		{	
			roll=1500;
			pitch=1515;
			yaw=1600;
		}

	    if(ros::Time::now().toSec() > stopTime)
			ros::shutdown();
}


int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "mission7");
  
  ros::NodeHandle n;
 
  ros::Subscriber subHud = n.subscribe("/vfr_hud", 1, flowHud);
  ros::Subscriber subbot = n.subscribe("irobotinfo", 1, callbackXtion);
  ros::Subscriber subbound = n.subscribe("boundaryinfo", 1, callbackDownwardCam);
  rc_pub = n.advertise<roscopter::RC>("send_rc", 1);
	
  takeOff();

    ros::Rate r(45);
    startTime=ros::Time::now().toSec();
    stopTime=ros::Time::now().toSec() + ros::Duration(120).toSec();
    roscopter::RC msg; 
	while(ros::ok()){
		interruptChecks();
        msg.channel[0]=roll;
	    msg.channel[1]=pitch;
        msg.channel[2]=throttle;
        msg.channel[3]=yaw;
    	msg.channel[4]=0;
    	msg.channel[5]=0;
	    msg.channel[6]=tilt;
    	msg.channel[7]=0;
        rc_pub.publish(msg);
		ros::spinOnce();
		startUpTimer++;
		r.sleep();	
	}

	land();
	pubZeros();

  return 0;
}

