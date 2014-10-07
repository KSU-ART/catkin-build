#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros_opencv/CameraCVmessage.h>
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
 ros::Publisher result_pub_;
 ros_opencv::CameraCVmessage light;

 
public:

 LightDetector()
   : it_(nh_)
 {

   result_pub_ = nh_.advertise<ros_opencv::CameraCVmessage>("blueLight" , 1);
   image_sub_ = it_.subscribe("/image_raw", 1, &LightDetector::imageCb, this);


 }

 ~LightDetector()
 {

 }


IplImage* GetThresholdedImage(IplImage* img)
{
// Convert the image into an HSV image
IplImage* imgHSV = cvCreateImage(cvGetSize(img), 8, 3);
cvCvtColor(img, imgHSV, CV_BGR2HSV);

IplImage* imgThreshed = cvCreateImage(cvGetSize(img), 8, 1);

// Values 20,100,100 to 30,255,255 working perfect for Blue at around 6pm
cvInRangeS(imgHSV, cvScalar(110,100,100), cvScalar(130,255,255), imgThreshed);

cvReleaseImage(&imgHSV);

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


bool lightResult =false;

Mat img_scene=cv_ptr->image;
// Couldn't get a device? Throw an error and quit
if(!img_scene.data)
   {
       cout << "Could not read image...\n";
return;
   }
IplImage img_scene_ipl_copy= img_scene;
IplImage* img_scene_ipl =&img_scene_ipl_copy;
// Holds the Blue thresholded image (Blue = white, rest = black)
IplImage* imgBlueThresh = GetThresholdedImage(img_scene_ipl);

// Calculate the moments to estimate the position of the light
CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
cvMoments(imgBlueThresh, moments, 1);

// The actual moment values
double moment10 = cvGetSpatialMoment(moments, 1, 0);
double moment01 = cvGetSpatialMoment(moments, 0, 1);
double area = cvGetCentralMoment(moments, 0, 0);

//cout << "Moment 10:" << moment10 << " \n Moment01:" << moment01 << "Area: ";
if(moment10>20000)
lightResult=true;


Mat imgMat(imgBlueThresh);  //Construct an Mat image "img" out of an IplImage
Mat mimg = imgBlueThresh;



cvReleaseImage(&imgBlueThresh);

delete moments;

cout<<moment10<<endl;
light.lighton=lightResult;

result_pub_.publish(light);

   return;

}
};

int main(int argc, char** argv)
{
 ros::init(argc, argv, "light_detector");
 LightDetector ld;
 ros::spin();
 return 0;
}
