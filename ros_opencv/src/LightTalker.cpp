#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv/cv.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "ros_opencv/TrackingPoint.h"

using namespace  cv;
using namespace std;

namespace enc = sensor_msgs::image_encodings;


class LightDetector
{
 ros::NodeHandle nh_;
 image_transport::ImageTransport it_;
 image_transport::Subscriber image_sub_;
 ros::Publisher result_pub;
 ros_opencv::TrackingPoint light;
 cv::Point p;
public:

 LightDetector()
   : it_(nh_)
 {
	result_pub= nh_.advertise<ros_opencv::TrackingPoint>(
	"tracking_point" , 1); image_sub_ = it_.subscribe(
	"image_raw", 1, &LightDetector::imageCb, this);
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
     cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
   }
Point p=Point(-1,-1);


Mat frame=cv_ptr->image;
Mat frameSmall;
Mat threshSmall;

resize(frame,frameSmall,Size(640,480));


// If we couldn't grab a frame... quit
   if(!frame.data)
       cout << "error grabbing frame";

  //Range for Blue light Detection (_InputArray)cvScalar(110,100,100), (_InputArray)cvScalar(130,255,255)
Mat imgColorThresh = GetThresholdedImage(frameSmall, (_InputArray)cvScalar(76,55,125), (_InputArray)cvScalar(84,105,225));

resize(imgColorThresh,threshSmall,Size(640,480));

IplImage iplimg =imgColorThresh;

// Calculate the moments to estimate the position of the light
CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
cvMoments(&iplimg, moments, 1);

// The actual moment values
double moment10 = cvGetSpatialMoment(moments, 1, 0);
double moment01 = cvGetSpatialMoment(moments, 0, 1);
double area = cvGetCentralMoment(moments, 0, 0);

int posX;
int posY;

if(area>0){
 posX = moment10/area;
 posY = moment01/area;
cout<<"Area: " <<area<<endl; } else{ posX=-1; posY=-1; }

cout << "Area Center: (" << posX << "," << posY << ")" << endl;

circle(frameSmall, Point(posX,posY), 30, cvScalar(255,0,0), 5, 8, 0);

cvNamedWindow("ColorTrackerRGB");
imshow("ColorTrackerRGB", frameSmall);
imshow("ColorTrackerThresh", threshSmall);
cv::waitKey(3);

threshSmall.release();
frameSmall.release();

light.pointX=posX;
light.pointY=posY;
result_pub.publish(light);
}   
};

int main(int argc, char** argv)
{
 ros::init(argc, argv, "light_talker");
 LightDetector ld;
 ros::spin();
 return 0;
}
