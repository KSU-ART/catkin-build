#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "opencv2/opencv.hpp"
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <string>

using namespace std;
using namespace cv;

namespace enc = sensor_msgs::image_encodings;

bool DEBUG = true;

image_transport::Publisher image_pub2;
image_transport::Publisher image_pub3;
//image_transport::Publisher image_pub3;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "multi_vid_cap_down_front");
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	
	image_pub2 = it.advertise("/usb_cam_2/image_raw", 1);
	image_pub3 = it.advertise("/usb_cam_3/image_raw", 1);
	//image_pub3 = it.advertise("/usb_cam_3/image_raw", 1);
	
	Mat frame_left;
	Mat frame_right;
//	Mat frame_other0;

	int seq = 0;
        
	VideoCapture cap_left(0);
	VideoCapture cap_right(1);
//	VideoCapture cap_other0(2);

	// publish here
	cv_bridge::CvImage out_msg;
	out_msg.header.frame_id = "peripheral_cams";
	out_msg.encoding = enc::BGR8;

	ros::Rate rate(30);

	while (ros::ok())
	{
		cap_left >> frame_left;
		cap_right >> frame_right;
//		cap_other0 >> frame_other0;
			
		if (frame_left.empty())
			return 1;
			
		if (frame_right.empty())
			return 1;
			
//		if (frame_other0.empty())
//			return 1;
			
		if (DEBUG)
		{
			imshow("left cam", frame_left);
			imshow("right cam", frame_right);
//			imshow("other0 cam", frame_other0);
		}
			
		out_msg.header.stamp = ros::Time::now();
		out_msg.header.seq = ++seq;

		out_msg.image = frame_left;
		image_pub2.publish(out_msg.toImageMsg());

		out_msg.image = frame_right;
		image_pub3.publish(out_msg.toImageMsg());
			
//		out_msg.image = frame_other0;
//		image_pub3.publish(out_msg.toImageMsg());
			
		if(DEBUG)
		{
			waitKey(33);
		}

		ros::spinOnce();
		rate.sleep();
	} 
    
    return 0;
}
