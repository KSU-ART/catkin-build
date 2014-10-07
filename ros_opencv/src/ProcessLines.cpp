#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros_opencv/Diffmessage.h>
#include <ros_opencv/TrackingPoint.h>
#include "opencv/cv.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/photo/photo.hpp"

using namespace  cv;
using namespace std;

namespace enc = sensor_msgs::image_encodings;

class ColorDetector
{
 ros::NodeHandle nh_;
 image_transport::ImageTransport it_;
 image_transport::Subscriber image_sub_;
 image_transport::Subscriber depth_image_sub;
 ros::Publisher result_pub;
 ros::Publisher result_pub_y;
 ros::Publisher depth_image_pub;
 ros_opencv::Diffmessage color;
 ros_opencv::TrackingPoint boundmsg;
 cv::Point p;
 cv::Vec3f selectedColor;
 bool loadNewStock;

public:

 ColorDetector()
   : it_(nh_)
 {
   result_pub= nh_.advertise<ros_opencv::TrackingPoint>("boundaryinfo" , 1);
   image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &ColorDetector::imageCb, this);
 }

 ~ColorDetector()
 {

 }

Mat GetThresholdedImage(Mat img, _InputArray lowBound, _InputArray upBound)
{

Mat imgHSV(img.rows,img.cols, CV_8UC3, Scalar(0,0,0));

cvtColor(img, imgHSV, CV_BGR2HSV);

Mat imgThreshed(img.rows,img.cols, CV_8UC3, Scalar(0,0,0));

inRange((_InputArray)imgHSV, lowBound, upBound, imgThreshed);

imgHSV.release();

return imgThreshed;
}

//Night Led webcam
//~ Mat getBoundary(Mat src)
//~ {
	//~ Mat fullRedThresh;
	//~ Mat fullGreenThresh;
	//~ Mat fullWhiteThresh;
	//~ Mat fullFinalThresh;
//~ 
	//~ Mat imgGreenHighThresh; inRange(src, Scalar(40,20,235), Scalar(80,125,255),imgGreenHighThresh);
    //~ Mat imgGreenLowThresh; inRange(src, Scalar(50,55,210), Scalar(85,110,255),imgGreenLowThresh);
	//~ Mat imgRedHighThresh; inRange(src, Scalar(50,60,215), Scalar(195,125,255),imgRedHighThresh);
	//~ Mat imgRedLowThresh; inRange(src, Scalar(0,50,220), Scalar(10,125,255),imgRedLowThresh);
    //~ Mat imgWhiteLowThresh; inRange(src, Scalar(0,0,200), Scalar(5,5,255),imgWhiteLowThresh);
    //~ Mat imgWhiteMidThresh; inRange(src, Scalar(60,0,200), Scalar(120,35,255),imgWhiteMidThresh);
    //~ Mat imgWhiteHighThresh; inRange(src, Scalar(200,5,200), Scalar(255,15,255),imgWhiteHighThresh);
//~ 
    //~ bitwise_or(imgGreenHighThresh,imgGreenLowThresh,fullGreenThresh);
	//~ bitwise_or(imgRedHighThresh,imgRedLowThresh,fullRedThresh);
	//~ bitwise_or(imgWhiteLowThresh,imgWhiteHighThresh,fullWhiteThresh);
    //~ bitwise_or(fullGreenThresh,fullRedThresh,fullFinalThresh);
    //~ bitwise_or(fullFinalThresh,fullWhiteThresh,fullFinalThresh);
    //~ bitwise_or(fullFinalThresh,imgWhiteMidThresh,fullFinalThresh);
//~ 
	//~ erode(fullFinalThresh, fullFinalThresh, Mat(), Point(-1,-1),2,1,1);
//~ 
	//~ return fullFinalThresh;
//~ }

//HD webcam
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


 void imageCb(const sensor_msgs::ImageConstPtr& msg)
 {
   cv_bridge::CvImagePtr cv_ptr;
   try
   {
     cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
   }

Mat frame=cv_ptr->image;
Mat frameSmall;
resize(frame, frameSmall, Size(640,480));
Mat hsv;
cvtColor(frameSmall, hsv, CV_BGR2HSV);
Mat finalImage = getBoundary(hsv);



IplImage iplimg =finalImage;

// Calculate the moments to estimate the position of the color
CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
cvMoments(&iplimg, moments, 1);

// The actual moment values
double moment10 = cvGetSpatialMoment(moments, 1, 0);
double moment01 = cvGetSpatialMoment(moments, 0, 1);
double area = cvGetCentralMoment(moments, 0, 0);

int posX;
int posY;

if(area>50){
 posX = moment10/area;
 posY = moment01/area;
}
else{
posX=-1;
posY=-1;
}

//circle(frameSmall, Point(posX,posY), 30, cvScalar(255,0,0), 5, 8, 0);

//imshow("ColorTrackerRGB", frameSmall);
//imshow("ColorTrackerThresh", finalImage);
//waitKey(30);

boundmsg.pointX=posX;
boundmsg.pointY=posY;

result_pub.publish(boundmsg);

}   
};

int main(int argc, char** argv)
{
 ros::init(argc, argv, "process_lines");
 ColorDetector ld;
 ros::spin();
 return 0;
}
