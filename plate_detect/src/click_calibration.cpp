#include "color.h"

int main(int argc, char** argv) {
	std::string path = "/home/marvin/catkin_ws/src/catkin-build/plate_detect/include";
	Color red('r', path);
	Color green('g', path);

	float percentage = 0.2;

	std::cout << "Calibrate Red: " << std::endl;
	red.calibrate_colors(2, percentage);
	std::cout << "Calibrate Green: " << std::endl;
	green.calibrate_colors(2, percentage);
	
	return 0;
}
