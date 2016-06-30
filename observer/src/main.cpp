#include "plate_angler.h"
#include "color_tracker.h"
#include "plate_localizer.h"
int main(int argc, char** argv)
{	
	waitKey(1000);
   ros::init(argc, argv, "find_objectblob");
   trackobjects top_cam("1"), top_right_cam("2"), bottom_right_cam("3"), 
				bottom_cam("4"), bottom_left_cam("5"), top_left_cam("6"),
				down_cam();
   angleFinder angler;
   plate_localizer c1;
   ros::MultiThreadedSpinner spinner(8); //use 8 threads
   spinner.spin(); //will not return until node is shut down
   return 0;
}
