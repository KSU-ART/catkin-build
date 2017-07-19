#ifndef IMAGE_DECODER_CLASS_H
#define IMAGE_DECODER_CLASS_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

// using namespace cv;
class imageDecoder{
private:
	cv::Mat img;
	ros::NodeHandle n;

	ros::Subscriber sub;

public:
	void downCamCB(const std_msgs::UInt8MultiArray::ConstPtr& msg){
		/// Load source image and convert it to gray
		ROS_INFO("GOT: %lu", msg->data.size());
		img = cv::imdecode(msg->data, 1);
		// cv::imshow("Smile", img);
		// cv::waitKey(33);
	}

	imageDecoder(std::string topic){
		sub = n.subscribe(topic, 1, &imageDecoder::downCamCB, this);
	}

	cv::Mat get_image(){
		return img;
	}

};

#endif
