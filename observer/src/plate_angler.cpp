/********************************************************
 * This program is to get shape location and orientation
 * (specifically of IARC ground-bot plates) using opencv.
 * Note currently for this program to function, the camera 
 *  must be nearly centered over the plate. This restriction 
 *  could be removed through implementation of an algorithm 
 * 	using more complicated geometrical characteristics of the plate.
 * Created by SPSU/KSU in 2016
 * ******************************************************/
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/Float32.h"

//this function returns the smallest angle between 2 vectors, measured in cosines.
static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
	//cout for debugging on jetson:
	std::cout << "Point 1: " << pt1.x<< " ' " <<pt1.y << "\nPoint 2: "
	 << pt2.x <<" , " << pt2.y << "\nPoint 0: " <<pt0.x << " , " << pt0.y << "\n\n";
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return(dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 +dy2*dy2) + 1e-10);
}

//this function returns the angle made by 2 cv::Point's in degrees.
double angle(cv::Point pt1, cv::Point pt2)
{
	double dy = (pt1.y-pt2.y);//pt1.y-pt2.y fixes y component being inverted
	double dx = (pt2.x-pt1.x + 1e-10);
	return (atan2(dy,dx)*180/3.14159265);
}

//this function returns the length between two cv::Point's in pixels.
double length(cv::Point pt1, cv::Point pt2)
{
	double dy = (pt2.y-pt1.y);
	double dx = (pt2.x-pt1.x + 1e-10);
	return (sqrt(dx*dx+dy*dy));
}

//the following returns the midpoint of 2 cv::Point's as a cv::Point.
cv::Point findMidpoint(cv::Point pt1, cv::Point pt2)
{
	cv::Point mid;
	mid.x = (pt1.x+pt2.x)/2;
	mid.y = (pt1.y+pt2.y)/2;
	return(mid);
	
}

//to sort vertices if this is the problem:
/*std::vector<cv::Point> sortVertices(std::vector<cv::Point> original)
{
	std::vector<cv::point> sorted;
	
}*/

namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;


class findAngle
{
  ros::NodeHandle n;
  ros::Publisher angle_pub;
  ros::Publisher angle_pub_2;
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber image_sub_2_;
  
public:
  findAngle()
    : it_(nh_)
  {
	angle_pub = n.advertise<std_msgs::Float32>("green_plate_angle", 100);
	angle_pub_2 = n.advertise<std_msgs::Float32>("red_plate_angle", 100);
    image_sub_ = it_.subscribe("green_binary", 1, &findAngle::greenCb, this);
    image_sub_2_ = it_.subscribe("red_binary", 1, &findAngle::redCb, this);
  }

  ~findAngle()
  {
	
  }

  void redCb(const sensor_msgs::ImageConstPtr& msg)
  {
	std_msgs::Float32 pubVar;
	pubVar.data = imageCb(msg);
	angle_pub_2.publish(pubVar);
  }
  
  
  void greenCb(const sensor_msgs::ImageConstPtr& msg)
  {
	std_msgs::Float32 pubVar;
	pubVar.data = imageCb(msg);
	angle_pub.publish(pubVar);
  }
  
  float imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
	bool cont = true;
	cv_bridge::CvImagePtr cv_ptr;
	cv::Mat binary_image;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      cont = false;
    }
    if (cont)
    {
		binary_image = cv_ptr->image;

		float orientation;
		
		//find contours
		std::vector<std::vector<cv::Point> > contours_vec;
		cv::findContours(
			binary_image.clone(), 
			contours_vec, 
			CV_RETR_EXTERNAL,//retrieves only extreme outer contours 
			CV_CHAIN_APPROX_SIMPLE //compresses horizontal, vertical, and diagonal segments to leave only their endpoints
		);
		

		//get approximate polygonal curbs from all contours:
		//array containing the approximation endpoints:
		std::vector<cv::Point> approx;
		
		
		//loop through all the contours:
		for (int i = 0; i < contours_vec.size(); i++)
		{
			//approximate edge points with accuracy proportional
			//	to the contour perimeter
			cv::approxPolyDP(
				cv::Mat (contours_vec[i]),
				approx,
				cv::arcLength(cv::Mat(contours_vec[i]), true) * 0.02,
				true
			);
			
			//skip small and convex objects:
			
			if (std::fabs( cv::contourArea(contours_vec[i]) ) < 500 || cv::isContourConvex(approx))
				continue;
			
			
			//std::cout << "the number of vertices is " <<approx.size() << std::endl;	
			
			//where number of verticies is 8 (like in the plate we want to detect):
			if (approx.size() == 8)
			{
				//number of vertices:
				int vertices = approx.size();
				
				//get angle (in cosines) of all corners:
				std::vector<double> cos;
				for (int j = 2; j < vertices + 2; j++)
					cos.push_back( angle(approx[j%vertices], approx[j-2], approx[(j-1)%vertices]) );
					
				//output angles to terminal (for debugging):
				///*
				std::cout << "\n\n" << cos.size() << "\n\n";
				for (int a = 0; a < cos.size(); a++)
					std::cout <<"angle " <<a+1 << ": " << cos[a] << "\n";//*/
					
				//sort ascending the corner degree values:
				std::sort( cos.begin(), cos.end() );
				
				//get lowest and highest angle:
				double mincos = cos.front();
				double maxcos = cos.back();
				
				//use angles above and the number of vertices
				//	to make sure all corners are about 90 (72-107) degrees and 
				//	consequently that we are about centered on the plate:
				if (mincos > -0.3 && maxcos < 0.3)
				{
					//Make sure rectangular boundaries
					cv::Rect r = cv::boundingRect(contours_vec[i]);
					double ratio = std::abs(1 - (double)r.width / r.height);
					if (ratio >= 0.02)
					{
						//find the longest side:
						int longest[] = {0,0};
						for (int j = 1; j < vertices + 1; j++)
						{
							double length1 = length(approx[j-1], approx[j%vertices]);
							if (length1 > longest[1]) 
							{
								longest[1] = length1;
								longest[2] = j-1;	
							}
						}
						
						//find center of longest side:
						cv::Point longmid = findMidpoint(approx[longest[2]], approx[(longest[2]+1)%vertices]);
						
						//find center of side 4 sides from the longest side (the front side):
						cv::Point frontmid = findMidpoint(approx[ (longest[2]+4)%vertices ], approx[(longest[2]+5)%vertices]);
						
						//calc angle of the vector formed from longest side center to front center
						//	 this is the orientation:
						orientation = angle(longmid, frontmid);
						
						//cout for debugging:
						std::cout << "the angle of the plate is "  << orientation << " degrees."<< std::endl;					
					}
					else
						orientation = 0.0;
				}
				else 
					orientation = 0.0;
			}
			return (orientation);
		}
		
		cv::waitKey(3);
		
	}
 }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "find_angle");
  findAngle ic;
  ros::spin();
  return 0;
}
