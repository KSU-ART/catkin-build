#ifndef BLOB_DETECT_COLOR_H
#define BLOB_DETECT_COLOR_H

#include <math.h>
#include <iostream>
#include <string>
#include <fstream>

class Color{
private:
	int lMin;
	int aMin;
	int bMin;
	int lMax;
	int aMax;
	int bMax;

	std::string calibration_file;

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
};

#endif