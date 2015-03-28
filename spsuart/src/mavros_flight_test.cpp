#include "ros/ros.h"
#include "px_comm/OpticalFlow.h"
#include <mavros/OverrideRCIn.h>
#include <mavros/VFR_HUD.h>
#include <ros_opencv/TrackingPoint.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv/cv.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include "PIDController.h"
#include <sstream>

using namespace std;
using namespace cv;
using namespace ros;  

namespace enc = sensor_msgs::image_encodings;	
Publisher rc_pub;
int throttle=1000;


void callbackVFRHUD(const mavros::VFR_HUD::ConstPtr& msgs){
	double alt =msgs->altitude;
	cout<<"Altitude: "<<alt<<endl;
	
	if(alt<1.4)
		throttle=1600;
	else if(alt>1.6)
		throttle=1200;
	else 
		throttle=1500;
	
}


void callbackImagePoint(const ros_opencv::TrackingPoint::ConstPtr& msgs){
//double x=msgs->pointX;
//double y=msgs->pointY;

//If No TrackingPoint on cam keep values neutral
	//if(x<0 || y<0){
		//yaw=1500;
		//return;
	//}
	
	//if(x>340){
		//yaw= 1500 - 75*((x-320)/320);
	//}
	//if(x<300){
		//yaw= 1500 + 75*((320-x)/320);
	//}

}


int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "mavros_flight_test");
  
  ros::NodeHandle n;
  ros::Subscriber subpoint = n.subscribe("test/image_point", 1, callbackImagePoint);
  ros::Subscriber subVfr = n.subscribe("mavros/vfr_hud", 1, callbackVFRHUD);
  image_transport::ImageTransport it_(n);
  image_transport::Subscriber flow_image_sub_;
  rc_pub = n.advertise<mavros::OverrideRCIn>("/mavros/rc/override", 1);	
  mavros::OverrideRCIn msg;
  ros::Rate r(45);

  while(ros::ok()){
	    msg.channels[0]=msg.CHAN_NOCHANGE;
	    msg.channels[1]=msg.CHAN_NOCHANGE;
        msg.channels[2]=throttle;
        msg.channels[3]=msg.CHAN_NOCHANGE;
    	msg.channels[4]=2000;
    	msg.channels[5]=msg.CHAN_NOCHANGE;
	    msg.channels[6]=msg.CHAN_NOCHANGE;
    	msg.channels[7]=msg.CHAN_NOCHANGE;
        rc_pub.publish(msg);  
 	ros::spinOnce();
 	r.sleep();	
	}

  return 0;
}

