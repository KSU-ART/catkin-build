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
	ros::Publisher g_ang;
	ros::Publisher r_ang;
	image_transport::Subscriber g_sub;
	image_transport::Subscriber r_sub;
	image_transport::ImageTransport it_;
  
public:
	angleFinder();
	
	~angleFinder();
	
	float getPlateAngle(const sensor_msgs::ImageConstPtr& msg);

	void greenCb(const sensor_msgs::ImageConstPtr& msg);

	void redCb(const sensor_msgs::ImageConstPtr& msg);

	double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);
	
	double angle(cv::Point pt1, cv::Point pt2);
	
	double length(cv::Point pt1, cv::Point pt2);
	
	cv::Point findMidpoint(cv::Point pt1, cv::Point pt2);
	
	
};

