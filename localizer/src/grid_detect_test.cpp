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
int thresh = 64;
int max_thresh = 255;

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

void mergeRelatedLines(vector<Vec2f> *lines, Mat &img)
{
	vector<Vec2f>::iterator current;
    for(current=lines->begin();current!=lines->end();current++)
    {
		if((*current)[0]==0 && (*current)[1]==-100) 
			continue;
		float p1 = (*current)[0];
        float theta1 = (*current)[1];
        
        Point pt1current, pt2current;
        if(theta1>CV_PI*45/180 && theta1<CV_PI*135/180)
        {
            pt1current.x=0;

            pt1current.y = p1/sin(theta1);

            pt2current.x=img.size().width;
            pt2current.y=-pt2current.x/tan(theta1) + p1/sin(theta1);
        }
        else
        {
            pt1current.y=0;

            pt1current.x=p1/cos(theta1);

            pt2current.y=img.size().height;
            pt2current.x=-pt2current.y/tan(theta1) + p1/cos(theta1);

        }
        
        vector<Vec2f>::iterator pos;
        for(pos=lines->begin();pos!=lines->end();pos++)
        {
            if(*current==*pos) 
				continue;
			if(fabs((*pos)[0]-(*current)[0])<20 && fabs((*pos)[1]-(*current)[1])<CV_PI*10/180)
            {
                float p = (*pos)[0];
                float theta = (*pos)[1];
                
                Point pt1, pt2;
                if((*pos)[1]>CV_PI*45/180 && (*pos)[1]<CV_PI*135/180)
                {
                    pt1.x=0;
                    pt1.y = p/sin(theta);
                    pt2.x=img.size().width;
                    pt2.y=-pt2.x/tan(theta) + p/sin(theta);
                }
                else
                {
                    pt1.y=0;
                    pt1.x=p/cos(theta);
                    pt2.y=img.size().height;
                    pt2.x=-pt2.y/tan(theta) + p/cos(theta);
                }
                if( ((double)(pt1.x-pt1current.x)*(pt1.x-pt1current.x) + (pt1.y-pt1current.y)*(pt1.y-pt1current.y)<thresh*thresh) &&
					((double)(pt2.x-pt2current.x)*(pt2.x-pt2current.x) + (pt2.y-pt2current.y)*(pt2.y-pt2current.y)<thresh*thresh) )
                {
                    // Merge the two
                    (*current)[0] = ((*current)[0]+(*pos)[0])/2;

                    (*current)[1] = ((*current)[1]+(*pos)[1])/2;

                    (*pos)[0]=0;
                    (*pos)[1]=-100;
                }
			}
		}
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
	
	Rect croping(18, 36, src.size().width- 58, src.size().height- 85);
	src = src(croping);
	
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

	ros::Subscriber sub = n.subscribe("/usb_cam/image_rect_color", 1, chatterCallback);


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
	int blockSize = 21;
	int apertureSize = 5;

	/// Detecting corners
	GaussianBlur(src, dst, Size(11,11), 0);
	bitwise_not(dst, dst);
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
	mergeRelatedLines(&lines, dst2);
	
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









