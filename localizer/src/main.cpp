#include "loc_object.h"
#include "grid_tracker.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "localizer_node");
	sensor_processor s1;
	grid_tracker gt_1;
	return 0;
}
