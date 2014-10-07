#include "ros/ros.h"
#include "px_comm/OpticalFlow.h"
#include <roscopter/RC.h>
#include <ros_opencv/Diffmessage.h>
#include <ros_opencv/Boundary.h>

using namespace std;
ros::Publisher rc_pub;
double targetAltitude;
int throttlePWM;
int pitch;
int roll;
int yaw=0;
roscopter::RC rcValues;

void flowCallback(const px_comm::OpticalFlow::ConstPtr& msg)
{
if(msg->ground_distance<1.9)
throttlePWM=1650;
else if(msg->ground_distance>2.1)
throttlePWM=1350;
else
throttlePWM=1500;
}

void callback(const ros_opencv::Boundary::ConstPtr& msgs){
bool turn=msgs->turn;
bool onbound=msgs->onbound;
int x=msgs->pointX;
int y=msgs->pointY;


}

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "follow_line");
  
  ros::NodeHandle n;
 
  ros::Subscriber subflow = n.subscribe("/px4flow/opt_flow", 1, flowCallback);
  ros::Subscriber subbound = n.subscribe("boundaryinfo", 1, callback);
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
    ros::Rate r(10);
    double startTime=ros::Time::now().toSec();
    double stopTime=ros::Time::now().toSec() + ros::Duration(30).toSec();
    bool run=true; 
	while(ros::ok()){
            msg.channel[0]=roll;
		    msg.channel[1]=pitch;
            msg.channel[2]=throttlePWM;
    		msg.channel[3]=yaw;
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

