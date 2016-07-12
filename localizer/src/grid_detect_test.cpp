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
int thresh = 2;
int max_thresh = 10;

string source_window = "Source image";
string corners_window = "Corners detected";

/// Function header
void cornerHarris_demo( int, void* );

void drawLine(Vec2f line, Mat &img, Scalar rgb = CV_RGB(0,0,255))
{
    if(line[1]!=0)
    {
        float m = -1/tan(line[1]);

        float c = line[0]/sin(line[1]);

        cv::line(img, Point(0, c), Point(img.size().width, m*img.size().width+c), rgb);
    }
    else
    {
        cv::line(img, Point(line[0], 0), Point(line[0], img.size().height), rgb);
    }

}

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

	ros::MultiThreadedSpinner spinner(6);
	spinner.spin();

	return(0);
}

/** @function cornerHarris_demo */
void cornerHarris_demo( int, void* )
{
	/// cornerHarris_demo
	Mat dst, dst2, dst3;
	dst = Mat::zeros( src.size(), CV_32FC1 );

	/// Detector parameters
	int blockSize = thresh*2+3;
	int apertureSize = 5;

	/// Detecting corners
	GaussianBlur(src, dst, Size(11,11), 0);
	adaptiveThreshold(dst, dst, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, blockSize, 2);
	bitwise_not(dst, dst);
	Mat kernel = (Mat_<uchar>(3,3) << 0,1,0,1,1,1,0,1,0);
    dilate(dst, dst2, kernel);
    
    int count=0;
    int max=-1;

    Point maxPt;

    for(int y=0;y<dst2.size().height;y++)
    {
        uchar *row = dst2.ptr(y);
        for(int x=0;x<dst2.size().width;x++)
        {
            if(row[x]>=128)
            {

                 int area = floodFill(dst2, Point(x,y), CV_RGB(16,16,64));

                 if(area>max)
                 {
                     maxPt = Point(x,y);
                     max = area;
                 }
            }
        }
    }
    
    floodFill(dst2, maxPt, CV_RGB(255,255,255));
    
    /*
    for(int y=0;y<dst2.size().height;y++)
    {
        uchar *row = dst2.ptr(y);
        for(int x=0;x<dst2.size().width;x++)
        {
            if(row[x]==64 && x!=maxPt.x && y!=maxPt.y)
            {
                int area = floodFill(dst2, Point(x,y), CV_RGB(0,0,0));
            }
        }
	}
    
    vector<Vec2f> lines;
    HoughLines(dst2, lines, 1, CV_PI/180, 200);
    
    for(int i=0;i<lines.size();i++)
    {
        drawLine(lines[i], dst2, CV_RGB(0,0,128));
    }
    //*/
	//cornerHarris( dst, dst, blockSize, apertureSize, (double)k/100, BORDER_DEFAULT );

	/// Normalizing
	//normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
	//convertScaleAbs( dst_norm, dst_norm_scaled );
	
	///inrage thresholding
	//inRange(dst_norm, Scalar(thresh, thresh, thresh), Scalar(255, 255, 255), dst_norm_scaled);
	
	/// Showing the result

	imshow( corners_window, dst2 );
	waitKey(10);
}









