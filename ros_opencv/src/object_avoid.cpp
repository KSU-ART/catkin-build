#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros_opencv/Diffmessage.h>
#include <ros_opencv/ObstacleDetected.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/photo/photo.hpp>

using namespace  cv;
using namespace std;

class ObstacleDetector
{
	ros::NodeHandle nh;
	
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    
    ros::Publisher result_pub;
    ros_opencv::ObstacleDetected obstaclemsg;
	
	public:

        ObstacleDetector()
            : it(nh)
        {
            result_pub=nh.advertise<ros_opencv::ObstacleDetected>("/spsuart/obstacle_detected" , 1);
            image_sub = it.subscribe("/camera/depth/image", 1, &ObstacleDetector::find_obstacle, this);
        }

        ~ObstacleDetector()
        {

        }
	
	Mat GetThresholdedImage (Mat img, _InputArray lowBound, _InputArray upBound)
	{
		Mat processed_bool;
		
		inRange(img,1,1,processed_bool);
		
		GaussianBlur(processed_bool,processed_bool,Size(9,9),1.5);
		
		return processed_bool;
	}
	
	void find_obstacle (const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_8UC1);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		
		_InputArray lowerBound = (_InputArray)cvScalar(0,0,0);
		_InputArray upperBound = (_InputArray)cvScalar(255,255,255);
		
		Mat original_image = cv_ptr->image;

		//Mat invert;
		//bitwise_not(original_image, invert);
		Mat processed_image = GetThresholdedImage(original_image,lowerBound,upperBound);
		
		vector<Point> threshVector;
			for(int j=0; j<processed_image.rows; j++) {
                for (int i=0; i<processed_image.cols; i++) {
                    if(processed_image.at<uchar>(j,i)==255) {
                        threshVector.push_back(Point(j,i));
                    }
                }
            }
		
		//Disabling showing for testing
	    //imshow("grayscale", original_image);
	    //imshow("processed",processed_image);
		//cv::waitKey(3);	
		
		if (threshVector.size() > 15000)
		{
			obstaclemsg.obstacleDetected = true;
		}else
		{
			obstaclemsg.obstacleDetected = false;
		}
		result_pub.publish(obstaclemsg);
	}
	
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_avoid");
    ObstacleDetector od;
    ros::spin();
    return 0;
}


