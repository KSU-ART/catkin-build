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

class ObstacleDetector
{
	_InputArray lowerBound = (_InputArray)cvScalar(0,0,0);
	_InputArray upperBound = (_InputArray)cvScalar(255,255,255);
	
	
	ros::NodeHandle nh;
	
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    
    ros::Publisher result_pub;
    ros_opencv::ObstacleDetected obstaclemsg;
	
	public:

        ObstacleDetector()
            : it(nh)
        {
            result_pub= nh.advertise<ros_opencv::TrackingPoint>("detect obstacle" , 1);
            image_sub = it.subscribe("/camera/depth/image_raw", 1, &ObstacleDetector::find_obstacle, this);
        }

        ~ObstacleDetector()
        {

        }
	
	Mat GetThresholdedImage (Mat img, _InputArray lowBound, _InputArray upBound)
	{
		Mat processed_bool;
		
		inRange(img,lowBound,upBound,processed_bool);
		
		GaussianBlur(processed_bool,processed_bool,Size(9,9),1.5);
		
		return processed_bool;
	}
	
	void find_obstacle (const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		
		Mat original_image = cv_ptr->image;
		
		//tracks num of objects found
		std::vector<Vec3f> objects;
		
		Mat processed_image = GetThresholdedImage(original_image,lowerBound,upperBound);
		
		HoughCisrcles(processed_image,objects,CV_HOUGH_GRADIENT,2,matProcessed.rows /4,100,50,20,400);
		
		if (objects.size() > 0)
		{
			result_pub.publish(true);
		}else
		{
			result_pub.publish(false);
		}
	}
	
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "find obstacle");
    ObstacleDetector od;
    ros::spin();
    return 0;
}


