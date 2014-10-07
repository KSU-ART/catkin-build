#include <ros/ros.h>
#include <ros_opencv/Diffmessage.h>
#include <ros_opencv/Depthmessage.h>
#include <roscopter/RC.h>

using namespace std;

ros::Publisher rc_pub;
roscopter::RC currentRCout;


void callback(const ros_opencv::Diffmessage::ConstPtr& msgs){
	
	roscopter::RC msg;
int yaw;
int currentYaw=1500;
int diff=msgs->value;
if(diff>75){
    yaw=currentYaw+100;
	cout<<"Yawing Left"<<endl;
}
else if((diff>-75 && diff <75) || diff==-321){
    yaw=0;
}
else if(diff<-100){
    yaw=currentYaw-100;
	cout<<"Yawing Right"<<endl;
}

//currentYaw=yaw;
//pitch code here
    msg.channel[0]=0;
    msg.channel[1]=0;
    msg.channel[2]=0;
    msg.channel[3]=yaw;
    msg.channel[4]=0;
    msg.channel[5]=0;
    msg.channel[6]=0;
    msg.channel[7]=0;
    rc_pub.publish(msg);

   if(ros::isShuttingDown()){
    msg.channel[0]=0;
    msg.channel[1]=0;
    msg.channel[2]=0;
    msg.channel[3]=0;
    msg.channel[4]=0;
    msg.channel[5]=0;
    msg.channel[6]=0;
    msg.channel[7]=0;
    rc_pub.publish(msg);
	cout<<"Node just died publishing zeros"<<endl;
}
}

void callbackDepth(const ros_opencv::Depthmessage::ConstPtr& msgs){
	
roscopter::RC msg;
int pitch;
int currentPitch=1500;
int depthValue=msgs->value;
if(depthValue>8){
    pitch=currentPitch-120;
	cout<<"Pitching Forward"<<endl;
}
else if((depthValue>4 && depthValue<9) || depthValue==-1){
    pitch=0;
	
}
else if(depthValue<5){	
    pitch=currentPitch+120;
	cout<<"Pitching Back"<<endl;
}

//currentPitch=pitch;
//pitch code here
    msg.channel[0]=0;
    msg.channel[1]=pitch;
    msg.channel[2]=0;
    msg.channel[3]=0;
    msg.channel[4]=0;
    msg.channel[5]=0;
    msg.channel[6]=0;
    msg.channel[7]=0;
    rc_pub.publish(msg);


   if(ros::isShuttingDown()){
    msg.channel[0]=0;
    msg.channel[1]=0;
    msg.channel[2]=0;
    msg.channel[3]=0;
    msg.channel[4]=0;
    msg.channel[5]=0;
    msg.channel[6]=0;
    msg.channel[7]=0;
    rc_pub.publish(msg);
	cout<<"Node just died publishing zeros"<<endl;
}
}

void rccallback(const roscopter::RC::ConstPtr& msg)
{
	cout<<"send rc:["<<msg->channel[0]<<","<<msg->channel[1]<<","<<msg->channel[2]<<","<<msg->channel[3]<<","<<msg->channel[4]<<","<<msg->channel[5]<<","<<msg->channel[6]<<","<<msg->channel[7]<<"]"<<endl; 
   if(ros::isShuttingDown()){
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
	cout<<"Node just died publishing zeros"<<endl;
}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle n;
    rc_pub = n.advertise<roscopter::RC>("send_rc", 1);     
    ros::Subscriber sub = n.subscribe("diff", 1, callback);
    ros::Subscriber subdepth = n.subscribe("depthValFromMask", 1, callbackDepth);
    ros::Subscriber subrc = n.subscribe("/rc", 1, rccallback);
    ros::spin();

return 0;
}
