#include "loc_object.h"
#include "grid_tracker.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "localizer_node");
	sensor_processor s1;
	grid_tracker gt_1;
	
	s1.system_loop();
	return 0;
}
