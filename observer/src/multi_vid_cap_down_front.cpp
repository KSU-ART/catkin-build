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

image_transport::Publisher image_pub0;
//image_transport::Publisher image_pub1;
//image_transport::Publisher image_pub3;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "multi_vid_cap_down_front");
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	
	image_pub0 = it.advertise("/usb_cam_0/image_raw", 1);
	//image_pub1 = it.advertise("/usb_cam_1/image_raw", 1);
	//image_pub3 = it.advertise("/usb_cam_3/image_raw", 1);
	
	Mat frame_down;
	//Mat frame_front;
//	Mat frame_other0;

	int seq = 0;
        
	VideoCapture cap_down(0);
	//VideoCapture cap_front(1);
//	VideoCapture cap_other0(2);

	// publish here
	cv_bridge::CvImage out_msg;
	out_msg.header.frame_id = "peripheral_cams";
	out_msg.encoding = enc::BGR8;

	ros::Rate rate(30);
	int counter = 0;
	double secs =ros::Time::now().toSec();
	while (ros::ok() && ros::Time::now().toSec() < secs+5)
	{
		cap_down >> frame_down;
		//cap_front >> frame_front;
//		cap_other0 >> frame_other0;
			
		if (frame_down.empty())
			return 1;
			
		//if (frame_front.empty())
		//	return 1;
			
//		if (frame_other0.empty())
//			return 1;

		counter++;
		
			
		if (DEBUG)
		{
//			imshow("down cam", frame_down);
			//imshow("front cam", frame_front);
//			imshow("other0 cam", frame_other0);
		}
			
		out_msg.header.stamp = ros::Time::now();
		out_msg.header.seq = ++seq;

//		out_msg.image = frame_down;
//		image_pub0.publish(out_msg.toImageMsg());

		//out_msg.image = frame_front;
		//image_pub1.publish(out_msg.toImageMsg());
			
//		out_msg.image = frame_other0;
//		image_pub3.publish(out_msg.toImageMsg());
			
		if(DEBUG)
		{
			waitKey(1);
		}

		ros::spinOnce();
		//rate.sleep();
	} 
    
	cout << "rate: " << counter / 5 << endl;
    return 0;
}
