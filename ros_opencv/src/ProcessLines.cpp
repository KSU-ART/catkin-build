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
   image_sub_ = it_.subscribe("image_raw", 1, &ColorDetector::imageCb, this);
 }

 ~ColorDetector()
 {

 }
 
 


// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
double angle( Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}
 
void findSquares( Mat& image, vector<vector<Point> >& squares )
{
	int thresh = 50, N = 11;
const char* wndname = "Square Detection Demo";

    squares.clear();

    Mat pyr, timg, gray0(image.size(), CV_8U), gray;

    // down-scale and upscale the image to filter out the noise
    pyrDown(image, pyr, Size(image.cols/2, image.rows/2));
    pyrUp(pyr, timg, image.size());
    vector<vector<Point> > contours;

    // find squares in every color plane of the image
    for( int c = 0; c < 1; c++ )
    {
        int ch[] = {c, 0};
        mixChannels(&timg, 1, &gray0, 1, ch, 1);
        


        // try several threshold levels
        for( int l = 0; l < N; l++ )
        {
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            if( l == 0 )
            {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                Canny(gray0, gray, 0, thresh, 5);
                // dilate canny output to remove potential
                // holes between edge segments
                dilate(gray, gray, Mat(), Point(-1,-1));
            }
            else
            {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                gray = gray0 >= (l+1)*255/N;
            }

            // find contours and store them all as a list
            findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

            vector<Point> approx;
            
            // test each contour
            for( size_t i = 0; i < contours.size(); i++ )
            {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if( approx.size() == 4 &&
                    fabs(contourArea(Mat(approx))) > 1000 &&
                    isContourConvex(Mat(approx)) && fabs(contourArea(Mat(approx))) < 100000)
                {
                    double maxCosine = 0;

                    for( int j = 2; j < 5; j++ )
                    {
                        // find the maximum cosine of the angle between joint edges
                        double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }

                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                    if( maxCosine < 0.3 ){
                        squares.push_back(approx);						
						}
					}
                }

            }
        }
}


// the function draws all the squares in the image
void drawSquares( Mat& image, const vector<vector<Point> >& squares )
{
	int thresh = 50, N = 11;
const char* wndname = "Square Detection Demo";
	
	int avgPosX=0;
	int avgPosY=0;
    
    for( size_t i = 0; i < squares.size(); i++ )
    {
        const Point* p = &squares[i][0];
        int n = (int)squares[i].size();
        polylines(image, &p, &n, 1, true, Scalar(0,255,0), 3, 8, 0);
        Moments m = moments(squares[i]);
		avgPosX += m.m10/m.m00;
		avgPosY += m.m01/m.m00;
    }
    
    if(squares.size() != 0)
	{
		avgPosX /= squares.size();
		avgPosY /= squares.size();
		circle(image, Point(avgPosX,avgPosY), 30, cvScalar(255,0,0), 5, 8, 0);
	}
	else{
		avgPosX = -1;
		avgPosY = -1;
	}
	
	ros_opencv::TrackingPoint trackingPoint;
	trackingPoint.pointX=avgPosX;
	trackingPoint.pointY=avgPosY;
	result_pub.publish(trackingPoint);

    imshow("Grid Squares", image);
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

		vector<vector<Point> > squares;
        findSquares(frameSmall, squares);
        drawSquares(frameSmall, squares);

//imshow("ColorTrackerRGB", frameSmall);
waitKey(1);

}
};

int main(int argc, char** argv)
{
 ros::init(argc, argv, "process_lines");
 ColorDetector ld;
 ros::spin();
 return 0;
}
