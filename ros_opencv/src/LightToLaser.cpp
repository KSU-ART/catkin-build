#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros_opencv/LightToLasermessage.h>
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
 ros::Publisher depth_image_pub;
 ros_opencv::LightToLasermessage light;
 cv::Point p;
 cv::Vec3f selectedColor;
 bool loadNewStock;
 vector<cv::Point> lightPoints; 
public:

 LightDetector()
   : it_(nh_)
 {

   result_pub_ = nh_.advertise<ros_opencv::LightToLasermessage>("LightToLaser" , 1);
   image_sub_ = it_.subscribe("/image_raw", 1, &LightDetector::imageCb, this);
   

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
Vec3f selectedColor;
bool loadNewStock=false;

Mat frame=cv_ptr->image;
Vec3f defaultColor;
defaultColor.val[0]=defaultColor.val[1]=defaultColor.val[2]=0;
Mat frameSmall;
Mat threshSmall;

resize(frame,frameSmall,Size(640,480));


// If we couldn't grab a frame... quit
   if(!frame.data)
       cout << "error grabbing frame";


    if(p.x!=-1||p.y!=-1){
        selectedColor[0]=frameSmall.data[frameSmall.channels()*(frameSmall.cols*p.y + p.x) + 0];
        selectedColor[1]=frameSmall.data[frameSmall.channels()*(frameSmall.cols*p.y + p.x) + 1];
        selectedColor[2]=frameSmall.data[frameSmall.channels()*(frameSmall.cols*p.y + p.x) + 2];
    }
    else
    selectedColor= defaultColor;


    Mat rgbPix= Mat(Size(50,50),CV_8UC3,Scalar(selectedColor.val[0],selectedColor.val[1],selectedColor.val[2]));
    Mat hsvPix;

    cvtColor((_InputArray)rgbPix,(_OutputArray)hsvPix,CV_RGB2HSV);

    //imshow("rgbPix", rgbPix);
    //imshow("hsvPix", hsvPix);

    int hsvH=(double)hsvPix.data[hsvPix.channels()*(hsvPix.cols*1 + 1) + 0];
       int hsvS=(double)hsvPix.data[hsvPix.channels()*(hsvPix.cols*1 + 1) + 1];
       int hsvV=(double)hsvPix.data[hsvPix.channels()*(hsvPix.cols*1 + 1) + 2];
    //cout<<"RGB: "<<(double)selectedColor.val[0]<<","<<selectedColor.val[1]<<","<<selectedColor.val[2]<<endl;
    cout<<"RGB: "<<selectedColor.val[0]<<","<<selectedColor.val[1]<<","<<selectedColor.val[2]<<endl;
    cout<<"HSV: "<<(double)hsvPix.data[hsvPix.channels()*(hsvPix.cols*1 + 1) + 0]<<","<<(double)hsvPix.data[hsvPix.channels()*(hsvPix.cols*1 + 1) + 1]<<","<<(double)hsvPix.data[hsvPix.channels()*(hsvPix.cols*1 + 1) + 2]<<endl;
// Holds the Color thresholded image (Color = white, rest = black)
    //(_InputArray)cvScalar(hsvH-10,hsvS-10,hsvV-10), (_InputArray)cvScalar(hsvH+10,hsvS+10,hsvV+10)
   //Range for Blue light Detection (_InputArray)cvScalar(110,100,100), (_InputArray)cvScalar(130,255,255)
Mat imgColorThresh = GetThresholdedImage(frameSmall, (_InputArray)cvScalar(110,100,100), (_InputArray)cvScalar(130,255,255) );

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
//cout<<"Area: " <<area<<endl;	
}
else{
posX=-1;
posY=-1;
}

cout << "Area Center: (" << posX << "," << posY << ")" << endl;

circle(frameSmall, Point(posX,posY), 30, cvScalar(255,0,0), 5, 8, 0);
int index;
if(posX!=-1){
double normal= (double)posX/(double)frameSmall.cols;

}
else{
index=-1;
}
cout<<"Image X: "<<posX;

cvNamedWindow("ColorTrackerRGB");

//cvSetMouseCallback("ColorTrackerRGB", mouseEvent, 0);

imshow("ColorTrackerRGB", frameSmall);
imshow("ColorTrackerThresh", threshSmall);

threshSmall.release();
frameSmall.release();
light.value=posX;
result_pub_.publish(light);
}   
};

int main(int argc, char** argv)
{
 ros::init(argc, argv, "light_to_laser");
 LightDetector ld;
 ros::spin();
 return 0;
}

