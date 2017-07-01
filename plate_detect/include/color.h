#ifndef BLOB_DETECT_COLOR_H
#define BLOB_DETECT_COLOR_H

#include "opencv2/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <string>
#include <fstream>

class Color{
private:
	char color_type;

	int lMin;
	int aMin;
	int bMin;
	int lMax;
	int aMax;
	int bMax;

	std::string calibration_file;

	cv::Point points[2];
	int clicks;

public:
	Color(){
		color_type = ' ';
		set_calibration_file("");
		lMin = 0;
		aMin = 0;
		bMin = 0;
		lMax = 180;
		aMax = 255;
		bMax = 255;
	}
	Color(char inColor){
		set_calibration_file("");
		set_color(inColor);
	}
	Color(char inColor, std::string cal_file){
		set_calibration_file(cal_file);
		set_color(inColor);
	}

	static void onMouse(int event, int x, int y, int flags, void* userdata);
	void onMouse(int event, int x, int y);

	cv::Scalar get_min_scalar();
	cv::Scalar get_max_scalar();

	void set_color(char inColor);

	void set_calibration_file(std::string file);

	void calibrate_colors(float deviation_percentage);

};


#endif