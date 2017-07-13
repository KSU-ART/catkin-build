#include "color.h"

int main(int argc, char** argv) {
	std::string path = "/home/stoplime/catkin_ws/src/catkin-build/plate_detect/include";
	Color red('r', path);
	Color green('g', path);

	float percentage = 0.2;

	std::cout << "Calibrate Red: " << std::endl;
	red.calibrate_colors(1, percentage);
	std::cout << "Calibrate Green: " << std::endl;
	green.calibrate_colors(1, percentage);
	
	return 0;
}
