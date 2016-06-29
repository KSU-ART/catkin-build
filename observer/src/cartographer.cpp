///cartographer
#include <iostream>
#include "ros/ros.h"
#include "camera_model.h"

using namespace std;
using namespace cv;

class cartographer
{
	ros::NodeHandle s;
	vector <cv::Point> pixels;
	geometry_msgs::Pose uavPose;
public:
	cartographer()
	{
		
	}
};	
		
