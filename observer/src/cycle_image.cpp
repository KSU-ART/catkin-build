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

bool DEBUG = false;

image_transport::Publisher image_pub1;
image_transport::Publisher image_pub2;
image_transport::Publisher image_pub3;
image_transport::Publisher image_pub4;
image_transport::Publisher image_pub5;
image_transport::Publisher image_pub6;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cam_cycle");
	
	ros::NodeHandle n;
	
	image_transport::ImageTransport it(n);
	
	image_pub1 = it.advertise("/uvc_cam_1/image_raw", 1);
	image_pub2 = it.advertise("/uvc_cam_2/image_raw", 1);
	image_pub3 = it.advertise("/uvc_cam_3/image_raw", 1);
	image_pub4 = it.advertise("/uvc_cam_4/image_raw", 1);
	image_pub5 = it.advertise("/uvc_cam_5/image_raw", 1);
	image_pub6 = it.advertise("/uvc_cam_8/image_raw", 1);
	
	string camera[6] = 
	{
		"/dev/video1",
		"/dev/video2",	
		"/dev/video3",
		"/dev/video4",
		"/dev/video5",
		"/dev/video8"
	};
	int cam_id = 0;
	
	Mat frame;
	Mat frame2;
	int seq = 0;
	int counter = 1;
        
	VideoCapture cap;

	do
	{
		cap.open(camera[cam_id]);

		//cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
		cout << "CV_CAP_PROP_FOURCC::  " << cap.get(CV_CAP_PROP_FOURCC) << endl;
		cout << "CV_CAP_PROP_WIDTH::  " << cap.get(CV_CAP_PROP_FRAME_WIDTH) << endl;
		cout << "CV_CAP_PROP_HEIGHT::  " << cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;

		cout << "counter: " << counter << endl;
		counter++;

		while(true)
		{
			cap >> frame;
			
			if (frame.empty())
			{
				break;
			}
			
			if (DEBUG)
			{
				if (frame.empty())
				{
					break;
				}
				imshow("cycle images", frame);
			}
			
			// publish here
			cv_bridge::CvImage out_msg;
			out_msg.header.seq = ++seq;
			out_msg.header.stamp = ros::Time::now();
			out_msg.header.frame_id = "peripheral_cams";
			out_msg.encoding = enc::BGR8;
			switch (cam_id)
			{
				case 0:
					out_msg.image = frame;
					image_pub1.publish(out_msg.toImageMsg());
					break;
				case 1:
					out_msg.image = frame;
					image_pub2.publish(out_msg.toImageMsg());
					break;
				case 2:
					out_msg.image = frame;
					image_pub3.publish(out_msg.toImageMsg());
					break;
				case 3:
					out_msg.image = frame;
					image_pub4.publish(out_msg.toImageMsg());
					break;
				case 4:
					out_msg.image = frame;
					image_pub5.publish(out_msg.toImageMsg());
					break;
				case 5:
					out_msg.image = frame;
					image_pub6.publish(out_msg.toImageMsg());
					break;
			}
			
			if(cap.isOpened())
				cap.release();
			 
			if(cam_id == 5)
			{
				camera = 0;
			}
			else
			{
				camera++;
			}
			continue;
			
			//ros::spinOnce();
			
			//waitKey(10);
			waitKey(33);
		}
	} while (ros::ok());
    
    return 0;
}
