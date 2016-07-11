#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <vector>

using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;

/// Global variables
Mat src;
int thresh = 140;
int max_thresh = 255;
int radius = 30;
int max_radius = 100;
int lowThreshold = 255;
int const max_lowThreshold = 255;
int ratio = 3;
int kernel_size = 3;

int k = 4;

vector<Point> corners;
vector<Point> actual_corners;
vector<Vec3f> circles;

string source_window = "Source image";
string corners_window = "Corners detected";
string canny_window = "Canny detected";

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
	namedWindow( canny_window );
	createTrackbar( "Threshold: ", source_window, &thresh, max_thresh, cornerHarris_demo );
	createTrackbar( "Radius: ", source_window, &radius, max_radius, cornerHarris_demo );
	createTrackbar( "Min Threshold:", source_window, &lowThreshold, max_lowThreshold, cornerHarris_demo );
	createTrackbar( "K:", source_window, &k, 10, cornerHarris_demo );

	ros::MultiThreadedSpinner spinner(6);
	spinner.spin();

	return(0);
}

/** @function cornerHarris_demo */
void cornerHarris_demo( int, void* )
{
	
	Mat dst, dst_norm, dst_norm_scaled;
	dst = Mat::zeros( src.size(), CV_32FC1 );

	/// Detector parameters
	int blockSize = 5;
	int apertureSize = 5;

	/// Detecting corners
	Canny( src, dst, lowThreshold, lowThreshold*ratio, kernel_size );
	imshow( canny_window, dst );
	cornerHarris( dst, dst, blockSize, apertureSize, (double)k/100, BORDER_DEFAULT );

	/// Normalizing
	normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
	convertScaleAbs( dst_norm, dst_norm_scaled );
	
	/// Drawing a circle around corners
	corners.clear();
	actual_corners.clear();
	int count_detected(0);
	
	//inrage thresholding
	inRange(dst_norm, Scalar(thresh, thresh, thresh), Scalar(255, 255, 255), dst_norm_scaled);
	
	/*
	for( int j = 0; j < dst_norm.rows ; j++ )
	{ 
		for( int i = 0; i < dst_norm.cols; i++ )
		{
			if( (int) dst_norm.at<float>(j,i) > thresh )
			{
				count_detected++;
				if (count_detected <= 500)
				{
					//circle( dst_norm_scaled, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );
					Point c(i,j);
					corners.push_back(c);
				}
				else
				{
					break;
				}
				
			}
		}
	}
	for (int i = corners.size(); i >= 0; i--)
	{
		int count_corners(0);
		for (int j = corners.size(); j >= 0; j--)
		{
			if (j != i)
			{
				if (corners[j].x >= corners[i].x-radius && corners[j].x <= corners[i].x+radius)
				{
					if (corners[j].y >= corners[i].y-radius && corners[j].y <= corners[i].y+radius)
					{
						count_corners++;
					}
				}
			}
		}
		if (count_corners == 3)
		{
			actual_corners.push_back(corners[i]);
		}
	}
	
	for (int i = 0; i < corners.size(); i++)
	{
		circle( dst_norm_scaled, corners[i], radius,  Scalar(0), 2, 8, 0 );
	}
	*/
	/// Showing the result

	imshow( corners_window, dst_norm_scaled );
	waitKey(10);
}









