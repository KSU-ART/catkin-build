#ifndef EDGE_DETECT_H
#define EDGE_DETECT_H

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <vector>
#include <math.h>
#include "imageDecoder.h"
#include "color.h"

#define PI 3.14159265

class edgeDetector{
private:
	ros::NodeHandle n;

    ros::Publisher pubx;
	ros::Publisher puby;
	ros::Publisher detectPub;

    ros::Subscriber sub;
    
    cv::Mat src, lab;
    cv::Mat dst, dst2, dst3;

    // imageDecoder im;

    std::string source_window;
    std::string corners_window;

    std::string path;
    int lineThresh;
    int maxOverhangThresh;
    int minBufferArea;
    float lineOffset;

    /// Detector parameters
	int blockSize;

    bool DEBUG;

    Color red;
    Color green;
    //cv::Vec3f vec_red;
    //cv::Vec3f vec_green;

    cv::Mat kernel;

public:
    // constructor
    edgeDetector()
    // :im("/usb_cam_down/image_raw/compressed")
    {
        path = "/home/stoplime/catkin_ws/src/catkin-build/plate_detect/include";
        lineThresh = 210;
        // lineThresh = 105;

        pubx = n.advertise<std_msgs::Float32>("/IARC/edgeDetect/arenaVector/x", 1);
        puby = n.advertise<std_msgs::Float32>("/IARC/edgeDetect/arenaVector/y", 1);
        detectPub = n.advertise<std_msgs::Bool>("/IARC/edgeDetect/detected", 1);
        sub = n.subscribe("/usb_cam_down/image_rect_color", 2, &edgeDetector::image_callback, this);

        source_window = "Source image";
        corners_window = "Corners detected";
        blockSize = 21;
        // maxOverhangThresh = 75;
        // minBufferArea = 8000;
        // lineOffset = 10;
        maxOverhangThresh = 150;
        minBufferArea = 32000;
        lineOffset = 20;

        DEBUG = true;
        //red = Color('r', path);
        //green = Color ('g', path);
        // std::cout << "*****************************" << red.getLMax() << std::endl;
        //vec_red = cv::Vec3f((red.getLMax()+red.getLMin())/2,(red.getAMax()+red.getAMin())/2,(red.getBMax()+red.getBMin())/2);
        //vec_green = cv::Vec3f((green.getLMax()+green.getLMin())/2,(green.getAMax()+green.getAMin())/2,(green.getBMax()+green.getBMin())/2);

        kernel = (cv::Mat_<uchar>(3,3) << 0,1,0,1,1,1,0,1,0);
    }

    //added by Kyle
    //std::vector<cv::Vec2f>* whittleLines(std::vector<cv::Vec2f> *lines, float angleThresh);

    void image_callback(const sensor_msgs::Image::ConstPtr& msg);

    // debug functions
    void drawLine(cv::Vec2f line, cv::Mat &img, cv::Scalar rgb = CV_RGB(0, 0, 255), int thickness = 1);

    /// counts the number of grid pixels on either side of each line
    /// if num of pixels is less thean maxOverhangThresh, then it is considered an edge line
    /// These lines will also take a buffer from the edge of the image by calculating the area of either side of the line
    /// if the area is larger than the minBufferArea, then it will be eligable for an edge line
    /// lineOffset is the offset on either side the area and counting of grid pixels will start
    void findEdges(std::vector<cv::Vec2f> *lines, cv::Mat &img, std::vector<cv::Vec2f> *edges, int maxOverhangThresh, int minBufferArea, float lineOffset);

    /// pre: lines of theta ond rho
    /// post: merges the parallel lines together
    void mergeRelatedLines(std::vector<cv::Vec2f> *lines, cv::Mat &img);

    void mergeRelatedLines(std::vector<cv::Vec4i> *lines);

    cv::Vec2f averageEdge(std::vector<cv::Vec2f> *edges);

    // void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    void largestBlob(cv::Mat src, cv::Mat& dst);

    void runGridProcOnce();

    cv::Mat getSrc(){return src;}
    
};

#endif
