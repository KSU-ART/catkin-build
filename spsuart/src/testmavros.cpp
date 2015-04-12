#include "ros/ros.h"
#include "px_comm/OpticalFlow.h"
#include <mavros/OverrideRCIn.h>
#include <mavros/Mavlink.h>
#include <roscopter/VFR_HUD.h>
#include <ros_opencv/Diffmessage.h>
#include <ros_opencv/TrackingPoint.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv/cv.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
//#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include "PIDController.h"
#include <sstream>

#include "mavlink/v1.0/ardupilotmega/mavlink.h"

using namespace std;
using namespace cv;
using namespace ros;  

namespace enc = sensor_msgs::image_encodings;	
PIDController* rateController = new PIDController();
Publisher rc_pub;
mavlink_message_t* msgt = NULL;
__mavlink_rangefinder_t* x = NULL;

double yLinearVelocity = 0;
int roll=1500;
int pitch=1500;

void flowCallback(const px_comm::OpticalFlow::ConstPtr& msg)
{
	float distance = msg->ground_distance;
	//float velx = msg->velocity_x;
	//float vely = msg->velocity_y;
	yLinearVelocity = -msg->velocity_y;
	double nowTime = ros::Time::now().toSec();
	//int dutyCycle = (rateController->calc(yLinearVelocity, nowTime) * 5) + 1500;
	cout<<"Flow altitude: "<< distance<<endl;
	//cout<<"Velocity x: "<<velx<<endl;
	cout<<"Velocity y: "<<yLinearVelocity<<endl;
}

void apmMavlinkmsgCallback(const mavros::Mavlink::ConstPtr& msg){
	//178
	if(msg->msgid==173){

		if(msgt == NULL)
		{
			msgt = new mavlink_message_t();
		}

		if(x == NULL)
		{
			x = new __mavlink_rangefinder_t();
		}

		msgt->seq = msg->seq;
		msgt->len = msg->len;
		msgt->sysid = msg->sysid;
		msgt->compid = msg->compid;
		msgt->msgid = msg->msgid;
		msgt->payload64[0] = msg->payload64[0];

		mavlink_msg_rangefinder_decode(msgt, x); 

		cout << "distance: " << x->distance << endl;
		cout << "Voltage: " << x->voltage << endl;

		//cout<<(float)((char *)((msg)->payload64[0])) <<endl;
		cout<<"Value 1: "<<msg->payload64[0]<<endl;
		//cout<<"Value 2: "<<msg->payload64[1]<<endl;
		//cout<<"Value 3: "<<msg->payload64[2]<<endl;
		//cout<<"Value 4: "<<msg->payload64[3]<<endl;
		cout<<"--------------------------------------------------------------------------"<<endl;

		delete msgt;
		delete x;

		msgt = NULL;
		x = NULL;
	}
}

void updatePID(){
	//lastTime = nowTime;
	
	//xLinearVelocity += globalImuValue.linear_acceleration.x * (nowTime - lastTime);
	//yLinearVelocity += globalImuValue.linear_acceleration.y * (nowTime - lastTime);
	//cout << "X Vel: " << xLinearVelocity << endl;
	//cout << "Y Vel: " << yLinearVelocity << endl;
	//xRateController->calc(xLinearVelocity, nowTime);
	//yRateController->calc(yLinearVelocity, nowTime);
}

void flowImageCallback(const sensor_msgs::ImageConstPtr& msg){
   cv_bridge::CvImagePtr cv_ptr;
   try
   {
     cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
     Size size(300,300);
     Mat dest;
     resize(cv_ptr->image, dest, size);	  
     cv::imshow("FlowImage", dest);
     cv::waitKey(3);
   }
   catch (cv_bridge::Exception& e) 
   {
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
   }
}

void callbackImagePoint(const ros_opencv::TrackingPoint::ConstPtr& msgs){
double x=msgs->pointX;
double y=msgs->pointY;

//If No TrackingPoint on cam keep values neutral
	if(x<0 || y<0){
		roll=1500;
		pitch=1500;
		//return;
	}

	if(x>330){
		roll=1500 + 75*((x-320)/320);
	}
	
	
	if(x<310){
		roll=1500 + 75*((320-x)/320);
	}

}

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "testmavros");
  
  ros::NodeHandle n;
  //ros::Subscriber subflow = n.subscribe("/px4flow/opt_flow", 1, flowCallback);
  ros::Subscriber submav = n.subscribe("mavlink/from", 1, apmMavlinkmsgCallback);
  //image_transport::ImageTransport it_(n);
  //image_transport::Subscriber flow_image_sub_;
  //flow_image_sub_ = it_.subscribe("/px4flow/camera_image", 1, flowImageCallback);  
  rc_pub = n.advertise<mavros::OverrideRCIn>("/mavros/rc/override", 1);	
  mavros::OverrideRCIn msg;
  ros::Rate r(45); 


  while(ros::ok()){
	msg.channels[0]=msg.CHAN_RELEASE;
	msg.channels[1]=msg.CHAN_RELEASE;
    msg.channels[2]=msg.CHAN_RELEASE;
    msg.channels[3]=msg.CHAN_RELEASE;
    msg.channels[4]=msg.CHAN_RELEASE;
    msg.channels[5]=msg.CHAN_RELEASE;
	msg.channels[6]=msg.CHAN_RELEASE;
    msg.channels[7]=msg.CHAN_RELEASE;
    rc_pub.publish(msg);  
	updatePID();  
 	ros::spinOnce();
 	r.sleep();	
	}  

  return 0;
}

