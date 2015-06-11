#include "ros/ros.h"
#include <mavros/OverrideRCIn.h>
#include <mavros/Mavlink.h>
#include "mavlink/v1.0/ardupilotmega/mavlink.h"
#include "px_comm/OpticalFlow.h"
#include "ros_opencv/TrackingPoint.h"
#include "sensor_msgs/NavSatFix.h"
#include "iostream"
#include "spsuart/pid-controller/PIDController.h"
#include "spsuart/DspFilters/Dsp.h"

using namespace std;

PIDController* altPosCtrl = new PIDController(500,0,0,-300,300);
//Lidar hack references
mavlink_message_t* msgt = NULL;
__mavlink_rangefinder_t* x = NULL;
double pos_x;
double pos_y;
double prevTime;

//Lidar hack : The product of pair programming
void apmMavlinkmsgCallback(const mavros::Mavlink::ConstPtr& msg){
        if(msg->msgid==173){
                if(msgt == NULL)
                        msgt = new mavlink_message_t();
                if(x == NULL)
                        x = new __mavlink_rangefinder_t();
                msgt->seq = msg->seq;
                msgt->len = msg->len;
                msgt->sysid = msg->sysid;
                msgt->compid = msg->compid;
                msgt->msgid = msg->msgid;
                msgt->payload64[0] = msg->payload64[0];
                mavlink_msg_rangefinder_decode(msgt, x); 
                
                // increase (or decrease?) altitude based on value from pid
                cout<<"Altitude :"<<x->distance<<endl;                
        double out = altPosCtrl->calc(double(x->distance));

        delete msgt;
                delete x;
                msgt = NULL;
                x = NULL;
        }
}

void imagePointCallback(const ros_opencv::TrackingPoint::ConstPtr& msg)
{
	
}

void optFlowCallback(const px_comm::OpticalFlow::ConstPtr& msg) 
{
   double currentTimeNano = ros::Time::now().toNSec();
   double deltaTime = currentTimeNano - prevTime;
   
   pos_x += msg->velocity_x * deltaTime;
   pos_y += msg->velocity_y * deltaTime;
   
   prevTime = ros::Time::now().toNSec();
}

int main(int argc, char **argv)
{
    //ROS node init and NodeHandle init
    ros::init(argc, argv, "gps_interface_node");
    ros::NodeHandle n;
    
    //Init prev time for velocity calc
    prevTime = ros::Time::now().toNSec();
    
    //Init positions at (0,0)
    pos_x = 0;
    pos_y = 0;
    
    //Image and mavlink message subscriber
    ros::Subscriber subpoint = n.subscribe("image_point",1,imagePointCallback);
    ros::Subscriber subflow = n.subscribe("px4flow/OpticalFlow",1,optFlowCallback);
    
    //GPS publisher
    ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>("/gps/fix", 1);
   
    //GPA msg container that will be sent to the FC
    sensor_msgs::NavSatFix gps;
    ros::Rate fcuCommRate(10); // emulating speed of dx9 controller
    
    //While node is alive send RC values to the FC @ fcuCommRate hz
    while(ros::ok())
    {
        cout<<"Pos X estimate: "<<pos_x<<endl;
        cout<<"Pos Y estimate: "<<pos_y<<endl;
        ros::spinOnce();
        fcuCommRate.sleep();
    }
    return 0;
}
