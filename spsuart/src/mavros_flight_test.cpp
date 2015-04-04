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
int roll=1500;
int pitch=1500;
//PIDController* xPosCtrl = PIDController();
//PIDController* yPosCtrl = PIDController();
PIDController* altPosCtrl = new PIDController();

double nowTimeVFRHUD = 0;
double nowTimeImagePoint = 0;

void callbackVFRHUD(const mavros::VFR_HUD::ConstPtr& msgs){
	double alt =msgs->altitude;
	nowTimeVFRHUD = ros::Time::now().toSec();
	double calc = altPosCtrl->calc(alt, nowTimeVFRHUD);
	throttle = 1500 + calc;
	cout<<"Setpoint: "<<altPosCtrl->getSetpoint()<<", Altitude: "<<alt<<", PID Calc: " <<calc<<endl;
}


void callbackImagePoint(const ros_opencv::TrackingPoint::ConstPtr& msgs){
/*
double x=msgs->pointX;
double y=msgs->pointY;

nowTimeImagePoint = = ros::Time::now().toSec();

if(x<0 || y<0){
		roll=1500;
		pitch=1500;
		xPosCtrl->calc(xPosCtrl->getSetpoint(), nowTimeImagePoint);
		yPosCtrl->calc(yPosCtrl->getSetpoint(), nowTimeImagePoint);
		return;
	}

	else {
		roll = 1500 - xPosCtrl->calc(x, nowTimeImagePoint);
		pitch = 1500 - xPosCtrl->calc(y, nowTimeImagePoint);
	}
*/	
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

  //xPosCtrl.setGains(1, 0, 0);
  //yPosCtrl.setGains(1, 0, 0);
  altPosCtrl->setGains(500, 0.0, 0);

  //xPosCtrl.setConstraints(-100, 100);
  //yPosCtrl.setConstraints(-100, 100);
  altPosCtrl->setConstraints(-300, 300);
  
  //xPosCtrl.init();
  //yPosCtrl.init();
  altPosCtrl->init();
  
  //xPosCtrl.targetSetpoint(320);
  //yPosCtrl.targetSetpoint(240);
  altPosCtrl->targetSetpoint(2.0);

 while(ros::ok()){
		msg.channels[0]=msg.CHAN_NOCHANGE;//roll;
		msg.channels[1]=msg.CHAN_NOCHANGE;//pitch;
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

