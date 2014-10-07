#include "ros/ros.h"
//#include "px_comm/OpticalFlow.h"
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

using namespace std;
using namespace cv;

namespace enc = sensor_msgs::image_encodings;


Mat getBoundary(Mat src)
{
Mat fullRedThresh;
Mat fullGreenThresh;
Mat fullFinalThresh;

Mat imgGreenHighThresh; inRange(src, Scalar(75,20,235), Scalar(105,125,255),imgGreenHighThresh);
    Mat imgGreenLowThresh; inRange(src, Scalar(60,120,210), Scalar(85,185,255),imgGreenLowThresh);
Mat imgRedHighThresh; inRange(src, Scalar(50,60,215), Scalar(195,125,255),imgRedHighThresh);
Mat imgRedLowThresh; inRange(src, Scalar(0,50,220), Scalar(10,125,255),imgRedLowThresh);
Mat imgWhiteThresh; inRange(src, Scalar(0,0,230), Scalar(5,5,255),imgWhiteThresh);

    bitwise_or(imgGreenHighThresh,imgGreenLowThresh,fullGreenThresh);
bitwise_or(imgRedHighThresh,imgRedLowThresh,fullRedThresh);
    bitwise_or(fullGreenThresh,fullRedThresh,fullFinalThresh);
    bitwise_or(fullFinalThresh,imgWhiteThresh,fullFinalThresh);

erode(fullFinalThresh, fullFinalThresh, Mat(), Point(-1,-1),2,1,1);

return fullFinalThresh;
}

//void flowCallback(const px_comm::OpticalFlow::ConstPtr& msg)
//{
//cout<<"Flow altitude: "<<msg->ground_distance<<endl;
//}

void callbackX(const ros_opencv::Diffmessage::ConstPtr& msgs){
int diff=msgs->value;
if(diff>75){
    cout<<"Rolling Right"<<endl;
}
else if((diff>-75 && diff <75) || diff==-321){
    cout<<"Roll Neutral"<<endl;
}
else if(diff<-75){
    cout<<"Rolling Left"<<endl;
}
}

void callbackY(const ros_opencv::Diffmessage::ConstPtr& msgs){
int diff=msgs->value;
if(diff>75){
    cout<<"Pitching Back"<<endl;
}
else if((diff>-75 && diff <75) || diff==-241){
    cout<<"Pitch Neutral"<<endl;
}
else if(diff<-75){
    cout<<"Pitching Forward"<<endl;
}
}

void callbackRc(const roscopter::RC::ConstPtr& msgs){
 cout<<"RC value:["<<msgs->channel[0]<<","<<msgs->channel[1]<<","<<msgs->channel[2]<<","<<msgs->channel[3]<<","<<msgs->channel[4]<<","<<msgs->channel[5]<<","<<msgs->channel[6]<<","<<msgs->channel[7]<<"]"<<endl; 
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
   cv_bridge::CvImagePtr cv_ptr;
   try
   {
     cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
     cv::imshow("XtionRgbImage", cv_ptr->image);
     cv::waitKey(3);
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
   }
}

void imageCallbackDown(const sensor_msgs::ImageConstPtr& msg){
   cv_bridge::CvImagePtr cv_ptr;
   try
   {
     cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
     cv::imshow("Downward Facing Camera", cv_ptr->image);
     Mat frameSmall=cv_ptr->image;
	 Mat hsv;
     cvtColor(frameSmall, hsv, CV_BGR2HSV);
     Mat finalImage = getBoundary(hsv);
     cv::imshow("Downward Facing Camera Post Processed", finalImage);
     cv::waitKey(3);
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
   }
}

void depthImageCallback(const sensor_msgs::ImageConstPtr& msg){
   cv_bridge::CvImagePtr cv_ptr;
   try
   {
     cv_ptr = cv_bridge::toCvCopy(msg, enc::TYPE_16UC1);	  
     cv::imshow("XtionDepthImage", cv_ptr->image);
     cv::waitKey(3);
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
   }
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

void callbackhud(const roscopter::VFR_HUD::ConstPtr& msgs){
cout<<"Barometer altitude: "<<msgs->alt<<endl;
cout<<"Compass: "<<msgs->heading<<endl;
} 

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "log_watcher");
  
  ros::NodeHandle n;
 
  //ros::Subscriber subflow = n.subscribe("/px4flow/opt_flow", 1, flowCallback);
  ros::Subscriber subx = n.subscribe("diff", 1, callbackX);
  ros::Subscriber suby = n.subscribe("diffy", 1, callbackY);
  ros::Subscriber rcsub= n.subscribe("rc", 1, callbackRc);
  ros::Subscriber subhud= n.subscribe("vfr_hud", 1, callbackhud);
  image_transport::ImageTransport it_(n);
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber image_sub2_;
  image_transport::Subscriber depth_image_sub_;
  image_transport::Subscriber flow_image_sub_;
  image_sub_ = it_.subscribe("/rgb/image_raw", 1, imageCallback);
  image_sub2_ = it_.subscribe("/usb_cam/image_raw", 1, imageCallbackDown);
  depth_image_sub_ = it_.subscribe("/depth/image_raw", 1, depthImageCallback);
  flow_image_sub_ = it_.subscribe("/px4flow/camera_image", 1, flowImageCallback);  

  while(ros::ok()){
 	ros::spin();
	}  


  return 0;
}

