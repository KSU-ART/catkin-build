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



class angleFinder
{
  ros::NodeHandle n;
  ros::Publisher angle_pub;
  
public:
	angleFinder();
	
	~angleFinder();
	
	
	double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);
	
	double angle(cv::Point pt1, cv::Point pt2);
	
	double length(cv::Point pt1, cv::Point pt2);
	
	cv::Point findMidpoint(cv::Point pt1, cv::Point pt2);
	
	float getAngle(cv::Mat binary_image);
	
};

