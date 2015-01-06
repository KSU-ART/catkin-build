#include "ros/ros.h"
#include "px_comm/OpticalFlow.h"
#include <roscopter/RC.h>
#include <roscopter/VFR_HUD.h>
#include <ros_opencv/Diffmessage.h>
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
#include <SerialStream.h>
#include "PIDController.h"
#include <sstream>

using namespace LibSerial;
using namespace std;
using namespace cv;
using namespace ros;  

namespace enc = sensor_msgs::image_encodings;	
SerialStream serial;
PIDController* rateController = new PIDController();
Publisher rc_pub;

double yLinearVelocity = 0;

void serialWrite (std::string str);

string IntToString(int number) {
	ostringstream oss;
	oss << number;
	return oss.str();
}

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


int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "sensor_test");
  
  ros::NodeHandle n;
  ros::Subscriber subflow = n.subscribe("/px4flow/opt_flow", 1, flowCallback);
  image_transport::ImageTransport it_(n);
  image_transport::Subscriber flow_image_sub_;
  flow_image_sub_ = it_.subscribe("/px4flow/camera_image", 1, flowImageCallback);  
  rc_pub = n.advertise<roscopter::RC>("send_rc", 1);	
  roscopter::RC msg;
  ros::Rate r(45); 
  rateController->setGains(1, 0, 0);
  rateController->init();
  rateController->targetSetpoint(0);

  while(ros::ok()){
	    msg.channel[0]=0;
	    msg.channel[1]=0;
        msg.channel[2]=0;
        msg.channel[3]=0;
    	msg.channel[4]=0;
    	msg.channel[5]=0;
	    msg.channel[6]=1480;
    	msg.channel[7]=0;
        rc_pub.publish(msg);  
	updatePID();  
 	ros::spinOnce();
 	r.sleep();	
	}  

  return 0;
}

