#include "ros/ros.h"
#include "px_comm/OpticalFlow.h"
#include <roscopter/RC.h>
#include <ros_opencv/Diffmessage.h>

using namespace std;
ros::Publisher rc_pub;
double targetAltitude;
int throttlePWM;
int pitch;
int roll;

void flowCallback(const px_comm::OpticalFlow::ConstPtr& msg)
{
if(msg->ground_distance<1.4)
throttlePWM=1650;
else if(msg->ground_distance>1.6)
throttlePWM=1350;
else
throttlePWM=1500;
}

void callbackX(const ros_opencv::Diffmessage::ConstPtr& msgs){
int diff=msgs->value;
if(diff>75){
    roll=1520;
    cout<<"Rolling Right"<<endl;
}
else if((diff>-75 && diff <75) || diff==-321){
    roll=0;
    cout<<"Roll Neutral"<<endl;
}
else if(diff<-75){
    roll=1480;
    cout<<"Rolling Left"<<endl;
}
}

void callbackY(const ros_opencv::Diffmessage::ConstPtr& msgs){
int diff=msgs->value;
if(diff>75){
    pitch=1500;
    cout<<"Pitching Back"<<endl;
}
else if((diff>-75 && diff <75) || diff==-241){
    pitch=0;
    cout<<"Pitch Neutral"<<endl;
}
else if(diff<-75){
    pitch=1460;
    cout<<"Pitching Forward"<<endl;
}
}

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "alt_hold");
  
  ros::NodeHandle n;
 
  ros::Subscriber subflow = n.subscribe("/px4flow/opt_flow", 1, flowCallback);
  ros::Subscriber subx = n.subscribe("diff", 1, callbackX);
  ros::Subscriber suby = n.subscribe("diffy", 1, callbackY);
  rc_pub = n.advertise<roscopter::RC>("send_rc", 1);

    roscopter::RC msg;
    msg.channel[0]=0;
    msg.channel[1]=0;
    msg.channel[2]=1700;
    msg.channel[3]=0;
    msg.channel[4]=0;
    msg.channel[5]=1000;
    msg.channel[6]=0;
    msg.channel[7]=0;
    rc_pub.publish(msg);
    ros::Rate r(4);
    double startTime=ros::Time::now().toSec();
    double stopTime=ros::Time::now().toSec() + ros::Duration(30).toSec();
    bool run=true; 
	while(ros::ok() && run){
		if(ros::Time::now().toSec() >= stopTime){
 		  run=true;
		}
		cout<<"Throttle PWM: "<<throttlePWM<<endl;
                msg.channel[0]=roll;
		msg.channel[1]=pitch;
                msg.channel[2]=throttlePWM;
    		msg.channel[3]=0;
    		msg.channel[4]=0;
    		msg.channel[5]=0;
   		msg.channel[6]=0;
    		msg.channel[7]=0;;
                rc_pub.publish(msg);
		ros::spinOnce();
		r.sleep();	
	}
	
    cout<<"Initiating Land"<<endl;
    msg.channel[0]=0;
    msg.channel[1]=0;
    msg.channel[2]=0;
    msg.channel[3]=0;
    msg.channel[4]=0;
    msg.channel[5]=2000;
    msg.channel[6]=0;
    msg.channel[7]=0;
    rc_pub.publish(msg);
    ros::Duration(5).sleep();

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
    r.sleep();
  return 0;
}

//bool changeAlt(spsuart::ChangeAlt::Request &req, spsuart::ChangeAlt::Response &res)
//{
//targetAltitude=req.alt;
//cout<<"Changing altitude to "<< req.alt<<endl;
//res.value=req.alt;
//"Rekt." -Brandon Hopewell 
//}
//ros::ServiceServer service n.advertiseService("alt",changeAlt);
