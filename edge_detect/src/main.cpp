#include "edge_detect.h"

int main( int argc, char** argv )
{
	ros::init(argc, argv, "listener");
    edgeDetector ed;

	ros::spin();
}