#include "ros/ros.h"
#include "px_comm/OpticalFlow.h"
#include <mavros/OverrideRCIn.h>
#include <roscopter/VFR_HUD.h>
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
#include <SerialStream.h>
#include "PIDController.h"
#include <sstream>

using namespace LibSerial;
using namespace std;
using namespace cv;
using namespace ros;  

namespace enc = sensor_msgs::image_encodings;	
Publisher rc_pub;
int throttle=1100;
int yaw=1500;

void callbackImagePoint(const ros_opencv::TrackingPoint::ConstPtr& msgs){
double x=msgs->pointX;
double y=msgs->pointY;

//If No TrackingPoint on cam keep values neutral
	if(x<0 || y<0){
		yaw=1500;
		return;
	}
	
	if(x>340){
		yaw= 1500 - 75*((x-320)/320);
	}
	if(x<300){
		yaw= 1500 + 75*((320-x)/320);
	}

}


void flowCallback(const px_comm::OpticalFlow::ConstPtr& msg)
{
	float distance = msg->ground_distance;
	float velx = msg->velocity_x;
	float vely = msg->velocity_y;
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


int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "mavros_yaw_test");
  
  ros::NodeHandle n;
  ros::Subscriber subflow = n.subscribe("/px4flow/opt_flow", 1, flowCallback);
  ros::Subscriber subpoint = n.subscribe("test/image_point", 1, callbackImagePoint);
  image_transport::ImageTransport it_(n);
  image_transport::Subscriber flow_image_sub_;
  flow_image_sub_ = it_.subscribe("/px4flow/camera_image", 1, flowImageCallback);  
  rc_pub = n.advertise<mavros::OverrideRCIn>("/mavros/rc/override", 1);	
  mavros::OverrideRCIn msg;
  ros::Rate r(45);
  int startTime=ros::Time::now().toSec();
  int stopTime=ros::Time::now().toSec() + ros::Duration(120).toSec();

  while(ros::ok()){
	    msg.channels[0]=msg.CHAN_NOCHANGE;
	    msg.channels[1]=msg.CHAN_NOCHANGE;
        msg.channels[2]=throttle;
        msg.channels[3]=0;
    	msg.channels[4]=msg.CHAN_NOCHANGE;
    	msg.channels[5]=msg.CHAN_NOCHANGE;
	    msg.channels[6]=msg.CHAN_NOCHANGE;
    	msg.channels[7]=msg.CHAN_NOCHANGE;
        rc_pub.publish(msg);  
 	ros::spinOnce();
 	r.sleep();	
	}
	
		msg.channels[0]=msg.CHAN_NOCHANGE;
	    msg.channels[1]=msg.CHAN_NOCHANGE;
        msg.channels[2]=1000;
        msg.channels[3]=msg.CHAN_NOCHANGE;
    	msg.channels[4]=msg.CHAN_NOCHANGE;
    	msg.channels[5]=msg.CHAN_NOCHANGE;
	    msg.channels[6]=msg.CHAN_NOCHANGE;
    	msg.channels[7]=msg.CHAN_NOCHANGE;
    	rc_pub.publish(msg);    

  return 0;
}

