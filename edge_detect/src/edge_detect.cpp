#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <vector>
#include <math.h>

#define PI 3.14159265

class edgeDetector{
private:
	ros::NodeHandle n;

	ros::Subscriber sub;
    
    cv::Mat src;
    std::string source_window = "Source image";
    std::string corners_window = "Corners detected";

public:
    // constructor
    edgeDetector(){
        sub = n.subscribe("/usb_cam_1/image_rect_color", 1, chatterCallback);

    }

    // debug functions
    void drawLine(Vec2f line, Mat &img, Scalar rgb = CV_RGB(0,0,255), int thickness = 1)
}