#include "color.h"

int main(int argc, char** argv) {
	Color red('r');
	Color green('g');

	float percentage = 0.2;

	red.calibrate_colors(percentage);
	green.calibrate_colors(percentage);
	
	return 0;
}
