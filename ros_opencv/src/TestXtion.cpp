#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros_opencv/Diffmessage.h>
#include "opencv/cv.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

using namespace  cv;
using namespace std;

namespace enc = sensor_msgs::image_encodings;


class LightDetector
{
 ros::NodeHandle nh_;
 image_transport::ImageTransport it_;
 image_transport::Subscriber image_sub_;
 image_transport::Subscriber depth_image_sub;
 ros::Publisher result_pub_;
 ros::Publisher result_pub_y;
 ros::Publisher depth_image_pub;
 ros_opencv::Diffmessage light;
 cv::Point p;
 cv::Vec3f selectedColor;
 bool loadNewStock;
 vector<cv::Point> lightPoints; 
public:

 LightDetector()
   : it_(nh_)
 {

   result_pub_ = nh_.advertise<ros_opencv::Diffmessage>("diff" , 1);
   result_pub_y= nh_.advertise<ros_opencv::Diffmessage>("diffy" , 1);
   image_sub_ = it_.subscribe("/camera/depth_registered/hw_registered/image_rect_raw", 1, &LightDetector::imageCb, this);
   
 }

 ~LightDetector()
 {

 }


Mat GetThresholdedImage(Mat img, _InputArray lowBound, _InputArray upBound)
{

Mat imgHSV(img.rows,img.cols, CV_8UC3, Scalar(0,0,0));

cvtColor(img, imgHSV, CV_BGR2HSV);

//imshow("hsv", imgHSV);

Mat imgThreshed(img.rows,img.cols, CV_8UC3, Scalar(0,0,0));

// Values 110,100,100 to 130,255,255 working perfect for Blue light
inRange((_InputArray)imgHSV, lowBound, upBound, imgThreshed);

imgHSV.release();

return imgThreshed;
}


 void imageCb(const sensor_msgs::ImageConstPtr& msg)
 {
   cv_bridge::CvImagePtr cv_ptr;
   try
   {
     cv_ptr = cv_bridge::toCvCopy(msg, enc::TYPE_16UC1);
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
   }
Point p=Point(-1,-1);
Vec3f selectedColor;
bool loadNewStock=false;

Mat frame=cv_ptr->image;
cout<<frame.rows << " " <<frame.cols <<endl;
}   
};

int main(int argc, char** argv)
{
 ros::init(argc, argv, "light_talker");
 LightDetector ld;
 ros::spin();
 return 0;
}

