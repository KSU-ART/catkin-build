#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

// using namespace cv;
class encImgListener{
private:
	cv::Mat img;
	ros::NodeHandle n;

	ros::Subscriber sub;

public:
	void downCamCB(const std_msgs::UInt8MultiArray::ConstPtr& msg){
		/// Load source image and convert it to gray
		ROS_INFO("GOT: %d", msg->data.size());
		img = cv::imdecode(msg->data, 1);
		cv::imshow("Smile", img);
		cv::waitKey(33);
	}

	encImgListener(std::string topic){
		sub = n.subscribe(topic, 1, downCamCB, this);
	}

	cv::Mat get_image(){
		return img;
	}

};

int main(int argc, char **argv){
	img = 0;
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	//namedWindow("Smile");
	
	ros::Subscriber sub = n.subscribe("/usb_cam_0/image_jpeg", 1000, downCamCB);
	
	ros::spin();
	
	return 0;
}
