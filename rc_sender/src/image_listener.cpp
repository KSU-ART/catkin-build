#include <ros/ros.h>
#include <ros_opencv/Diffmessage.h>
#include <ros_opencv/Depthmessage.h>
#include <roscopter/RC.h>

using namespace std;

ros::Publisher rc_pub;
//roscopter::RC currentRCout;
int pitch;
int roll;

void callback(const ros_opencv::Diffmessage::ConstPtr& msgs){
int diff=msgs->value;
if(diff>75){
    roll=1575;
    cout<<"Rolling Right"<<endl;
}
else if((diff>-75 && diff <75) || diff==-321){
    roll=0;
}
else if(diff<-75){
    roll=1425;
    cout<<"Rolling Left"<<endl;
}
}

void callbackY(const ros_opencv::Diffmessage::ConstPtr& msgs){
int diff=msgs->value;
if(diff>75){
    pitch=1575;
    cout<<"Pitching Back"<<endl;
}
else if((diff>-75 && diff <75) || diff==-241){
    pitch=0;
}
else if(diff<-75){
    pitch=1425;
    cout<<"Pitching Forward"<<endl;
}
}

//void rccallback(const roscopter::RC::ConstPtr& msg)
//{
//currentRCout.channel[0]=msg->channel[0];
//currentRCout.channel[1]=msg->channel[1];
//currentRCout.channel[2]=msg->channel[2];
//currentRCout.channel[3]=msg->channel[3];
//currentRCout.channel[4]=msg->channel[4];
//currentRCout.channel[5]=msg->channel[5];
//currentRCout.channel[6]=msg->channel[6];
//currentRCout.channel[7]=msg->channel[7];
//  cout<<"["<<msg->channel[0]<<","<<msg->channel[1]<<","<<msg->channel[2]<<","<<msg->channel//[3]<<","<<msg->channel[4]<<","<<msg->channel[5]<<","<<msg->channel[6]<<","<<msg->channel[7]<<"]"<<endl;
//}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle n;
    rc_pub = n.advertise<roscopter::RC>("send_rc", 1);     
    ros::Subscriber sub = n.subscribe("diff", 1, callback);
    ros::Subscriber suby = n.subscribe("diffy", 1, callbackY);
    //ros::Subscriber subrc = n.subscribe("/rc", 1, rccallback);
    ros::Rate r(5);
    roscopter::RC msg;
    msg.channel[0]=0;
    msg.channel[1]=0;
    msg.channel[2]=0;
    msg.channel[3]=0;
    msg.channel[4]=0;
    msg.channel[5]=0;
    msg.channel[6]=0;
    msg.channel[7]=0; 
 	while(ros::ok()){
		msg.channel[0]=roll;
                msg.channel[1]=pitch;
                rc_pub.publish(msg);
		ros::spinOnce();
		r.sleep();
	}
    rc_pub.publish(msg);
	cout<<"Node just died publishing zeros"<<endl;
    r.sleep();
return 0;
}
