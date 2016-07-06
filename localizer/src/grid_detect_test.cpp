#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <string>

using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;

/// Global variables
Mat src;
int thresh = 200;
int max_thresh = 255;

string source_window = "Source image";
string corners_window = "Corners detected";

/// Function header
void cornerHarris_demo( int, void* );

void chatterCallback(const sensor_msgs::ImageConstPtr& msg)
{
	/// Load source image and convert it to gray
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
  
  src = cv_ptr->image; 
  
  imshow( source_window, src );
  cvtColor( src, src, CV_BGR2GRAY );
  cornerHarris_demo( 0, 0 );
  
  waitKey(10);
}

/** @function main */
int main( int argc, char** argv )
{
  
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/usb_cam/image_raw", 1, chatterCallback);
  

  /// Create a window and a trackbar
  namedWindow( source_window );
  namedWindow( corners_window );
  createTrackbar( "Threshold: ", source_window, &thresh, max_thresh, cornerHarris_demo );
 
  ros::spin();

  return(0);
}

/** @function cornerHarris_demo */
void cornerHarris_demo( int, void* )
{

  Mat dst, dst_norm, dst_norm_scaled;
  dst = Mat::zeros( src.size(), CV_32FC1 );

  /// Detector parameters
  int blockSize = 3;
  int apertureSize = 3;
  double k = 0.04;

  /// Detecting corners
  cornerHarris( src, dst, blockSize, apertureSize, k, BORDER_DEFAULT );

  /// Normalizing
  normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
  convertScaleAbs( dst_norm, dst_norm_scaled );

  /// Drawing a circle around corners
  for( int j = 0; j < dst_norm.rows ; j++ )
     { for( int i = 0; i < dst_norm.cols; i++ )
          {
            if( (int) dst_norm.at<float>(j,i) > thresh )
              {
               circle( dst_norm_scaled, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );
              }
          }
     }
  /// Showing the result
  
  imshow( corners_window, dst_norm_scaled );
  waitKey(10);
}
