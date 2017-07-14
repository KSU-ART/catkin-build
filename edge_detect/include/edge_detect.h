#ifndef EDGE_DETECT_H
#define EDGE_DETECT_H

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
#include "imageDecoder.h"

#define PI 3.14159265

class edgeDetector{
private:
	ros::NodeHandle n;
    
    cv::Mat src;
    cv::Mat dst, dst2, dst3;

    imageDecoder im;

    std::string source_window;
    std::string corners_window;

    /// Detector parameters
	int blockSize;

    bool DEBUG;

public:
    // constructor
    edgeDetector()
    :im("/sensor/forwardCam")
    {
        source_window = "Source image";
        corners_window = "Corners detected";
        blockSize = 21;
        DEBUG = true;
    }

    // debug functions
    void drawLine(cv::Vec2f line, cv::Mat &img, cv::Scalar rgb = CV_RGB(0,0,255), int thickness = 1);

    /// counts the number of grid pixels on either side of each line
    /// if num of pixels is less thean maxOverhangThresh, then it is considered an edge line
    /// These lines will also take a buffer from the edge of the image by calculating the area of either side of the line
    /// if the area is larger than the minBufferArea, then it will be eligable for an edge line
    /// lineOffset is the offset on either side the area and counting of grid pixels will start
    void findEdges(std::vector<cv::Vec2f> *lines, cv::Mat &img, std::vector<cv::Vec3f> *edges, int maxOverhangThresh, int minBufferArea, float lineOffset);

    /// pre: lines of theta ond rho
    /// post: merges the parallel lines together
    void mergeRelatedLines(std::vector<cv::Vec2f> *lines, cv::Mat &img);

    // void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    void runGridProcOnce();

};

#endif