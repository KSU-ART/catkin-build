#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros_opencv/Diffmessage.h>
#include <ros_opencv/Depthmessage.h>
#include <ros_opencv/TrackingPoint.h>
#include "opencv/cv.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;



#define USE_IMAGE_TRANSPORT_SUBSCRIBER_FILTER 1

#if USE_IMAGE_TRANSPORT_SUBSCRIBER_FILTER
#  include <image_transport/subscriber_filter.h>
#else
#  include <sensor_msgs/Image.h>
#  include <message_filters/subscriber.h>
#endif

class MyClass {

ros::NodeHandle nh_;
image_transport::ImageTransport it_;
ros::Publisher result_pub;
ros::Publisher result_pub_y;
ros::Publisher depth_pub;
ros_opencv::TrackingPoint color;
ros_opencv::Depthmessage depthPoint;
cv::Point p;
cv::Vec3f selectedColor;
bool loadNewStock;
vector<cv::Point> colorPoints;

public:
MyClass() :
it_(nh_),
#if USE_IMAGE_TRANSPORT_SUBSCRIBER_FILTER
rgb_image_sub_( it_, "camera/rgb/image_raw", 1 ),
depth_image_sub_( it_, "camera/depth/image_raw", 1 ),
#else
orig_image_sub_( nh_, "camera/rgb/image_raw", 1 ),
warp_image_sub_( nh_, "camera/depth/image_raw", 1 ),
#endif
sync( MySyncPolicy(2), rgb_image_sub_, depth_image_sub_ )
{
sync.registerCallback( boost::bind( &MyClass::callback, this, _1, _2 ) );
}


Mat GetThresholdedImage(Mat img, _InputArray lowBound, _InputArray upBound)
{


Mat imgHSV(img.rows,img.cols, CV_8UC3, Scalar(0,0,0));

cvtColor(img, imgHSV, CV_BGR2HSV);

Mat imgThreshed(img.rows,img.cols, CV_8UC3, Scalar(0,0,0));

// Values 110,100,100 to 130,255,255 working perfect for Blue light
inRange((_InputArray)imgHSV, lowBound, upBound, imgThreshed);

imgHSV.release();

return imgThreshed;
}

void FilledCircle( Mat img, Point center )
{
int thickness = -1;
int lineType = 8;

circle( img,
center,
3,
Scalar( 0, 0, 255 ),
thickness,
lineType );
}


double Average(vector<double> v)
{      int sum=0;
for(int i=0;i<v.size();i++)
sum+=v[i];
return sum/v.size();
}

double Deviation(vector<double> v, double ave)
{
double E=0;
double inverse = 1.0 / static_cast<double>(v.size());
for(unsigned int i=0;i<v.size();i++)
{
E += pow(static_cast<double>(v[i]) - ave, 2);
}
return sqrt(inverse * E);
}


void callback(
const sensor_msgs::ImageConstPtr& rgb_msg,
const sensor_msgs::ImageConstPtr& depth_msg
){

result_pub= nh_.advertise<ros_opencv::TrackingPoint>("tracking_point" , 1);
depth_pub= nh_.advertise<ros_opencv::Depthmessage>("depthValFromMask" , 1);

cv_bridge::CvImagePtr cv_ptr;
cv_bridge::CvImagePtr cv_ptr_depth;
Mat blur_img;
try
{
cv_ptr = cv_bridge::toCvCopy(rgb_msg, enc::BGR8);
cv_ptr_depth = cv_bridge::toCvCopy(depth_msg, enc::TYPE_32FC1);	
    double minVal, maxVal;
    minMaxLoc(cv_ptr_depth->image, &minVal, &maxVal); //find minimum and maximum intensities
    //Mat draw;
    cv_ptr_depth->image.convertTo(blur_img, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));  
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
Mat depth=blur_img;
Mat frameSmall;
Mat threshSmall;
Mat depthSmall;

resize(frame,frameSmall,Size(640,480));


Mat imgColorThresh = GetThresholdedImage(frameSmall, (_InputArray)cvScalar(40,0,0), (_InputArray)cvScalar(58,255,255));

resize(imgColorThresh,threshSmall,Size(640,480));
resize(depth,depthSmall,Size(640,480));

IplImage iplimg =imgColorThresh;

// Calculate the moments to estimate the position of the color
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
}
else{
posX=-1;
posY=-1;
}

color.pointX=posX;
color.pointY=posY;
result_pub.publish(color);

circle( frameSmall, Point(posX, posY), 3, Scalar( 0, 0, 255), 2, 8);
cv::imshow("rgb image", frameSmall);
cv::imshow("threshed image", threshSmall);
cv::imshow("depth image", depthSmall);
cv::waitKey(3);
//Release images from memory
threshSmall.release();
frameSmall.release();
depthSmall.release();

}

private:

#if USE_IMAGE_TRANSPORT_SUBSCRIBER_FILTER
typedef image_transport::SubscriberFilter ImageSubscriber;
#else
typedef message_filters::Subscriber< sensor_msgs::Image > ImageSubscriber;
#endif

ImageSubscriber rgb_image_sub_;
ImageSubscriber depth_image_sub_;

typedef message_filters::sync_policies::ApproximateTime<
sensor_msgs::Image, sensor_msgs::Image
> MySyncPolicy;

message_filters::Synchronizer< MySyncPolicy > sync;
};

int main(int argc, char** argv) {

ros::init( argc, argv, "XtionTrackColor" );
MyClass mc;

while( ros::ok() ){
ros::spin();

}

return EXIT_SUCCESS;
}
