#include "color.h"

void Color::set_color(char inColor){
	switch(inColor){
	case 'g':
		ifstream green;
		green.open(calibration_file + "/green.txt");
		if (green.is_open()){
			green >> lMin;
			green >> aMin;
			green >> bMin;
			green >> lMax;
			green >> aMax;
			green >> bMax;
		}
		green.close();
		break;
		
	case 'r':
		ifstream red;
		red.open(calibration_file + "/red.txt");
		if (red.is_open()){
			red >> lMin;
			red >> aMin;
			red >> bMin;
			red >> lMax;
			red >> aMax;
			red >> bMax;
		}
		red.close();
		break;
		
	default:
		lMin = 0;
		aMin = 0;
		bMin = 0;
		lMax = 180;
		aMax = 255;
		bMax = 255;
	}
}

void Color::set_calibration_file(std::string file){
	if (file.compare("") == 0){
		calibration_file = "/home/stoplime/catkin_ws/src/catkin-build/plate_detect/include/plate_detect";
	}
	else{
		calibration_file = file;
	}
}

