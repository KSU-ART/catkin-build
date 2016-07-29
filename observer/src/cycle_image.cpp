#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "opencv2/opencv.hpp"
#include <sensor_msgs/image_encodings.h>
#include <iostream>

using namespace std;
using namespace cv;

namespace enc = sensor_msgs::image_encodings;

bool DEBUG = true;


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
	image_pub6 = it.advertise("/uvc_cam_6/image_raw", 1);
	
	int camera = 1;
	Mat frame;
	int seq = 0;
	
	do
	{
		cout << "grab new cam: " << endl;
		VideoCapture cap(camera);
		while(1)
		{
			cap >> frame;
			
			cout << "CV_CAP_PROP_FORMAT: " << cap.get(CV_CAP_PROP_FORMAT) << endl;
			//~ cout << "CV_CAP_PROP_FOURCC: " << cap.get(CV_CAP_PROP_FOURCC) << endl;
			//~ cout << "CV_CAP_PROP_FRAME_WIDTH: " << cap.get(CV_CAP_PROP_FRAME_WIDTH) << endl;
			//~ cout << "CV_CAP_PROP_FRAME_HEIGHT: " << cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;
			//~ cout << "CV_CAP_PROP_FPS: " << cap.get(CV_CAP_PROP_FPS) << endl;
			//~ cout << "CV_CAP_PROP_MODE: " << cap.get(CV_CAP_PROP_MODE) << endl;
			
			// timeout sequence for waiting for images
			//~ while (!cap.isOpened())
			//~ {
				//~ waitKey(10);
			//~ }
			
			if (DEBUG)
			{
				if (!(frame.size().width >0 && frame.size().height >0))
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
			switch (camera)
			{
				case 1:
					out_msg.image = frame;
					image_pub1.publish(out_msg.toImageMsg());
					break;
				case 2:
					out_msg.image = frame;
					image_pub2.publish(out_msg.toImageMsg());
					break;
				case 3:
					out_msg.image = frame;
					image_pub3.publish(out_msg.toImageMsg());
					break;
				case 4:
					out_msg.image = frame;
					image_pub4.publish(out_msg.toImageMsg());
					break;
				case 5:
					out_msg.image = frame;
					image_pub5.publish(out_msg.toImageMsg());
					break;
				case 6:
					out_msg.image = frame;
					image_pub6.publish(out_msg.toImageMsg());
					break;
			}
			
			
			if (waitKey(10) >= 27)
			{
				cap.release();
				if(camera > 5)
					camera = 1;
				else
					camera++;
				
				continue;
			}
			
			waitKey(10);
		}
	} while (ros::ok());
    // the camera will be closed automatically upon exit
    // cap.close();
    return 0;
}
