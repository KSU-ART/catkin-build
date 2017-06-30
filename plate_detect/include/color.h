#ifndef BLOB_DETECT_COLOR_H
#define BLOB_DETECT_COLOR_H

#include "opencv2/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <string>
#include <fstream>


Point points[2] = {Point(0,0),Point(1,1)};
int clicks = 0;

void calibration_click_cb(int event, int x, int y, int flags, void* userdata){
	if  ( event == EVENT_LBUTTONDOWN ){
		clicks++;
		points[clicks]=(Point(x,y));
	}
}

class Color{
private:
	int lMin;
	int aMin;
	int bMin;
	int lMax;
	int aMax;
	int bMax;

	std::string calibration_file;

	float percentage = 0.2;
	string calibration_file;

public:
	Color(){
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

	void set_color(char inColor);

	void set_calibration_file(std::string file);

	void calibrate_colors();

};


#endif