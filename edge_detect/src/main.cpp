#include "edge_detect.h"

int main( int argc, char** argv )
{
	ros::init(argc, argv, "listener");
    edgeDetector ed;
    ros::Rate loopRate(30);

    while(ros::ok()){
        ed.runGridProcOnce();
        loopRate.sleep();
        ros::spinOnce();
    }
}