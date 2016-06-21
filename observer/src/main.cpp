#include "plate_angler.h"
#include "color_tracker.h"
#include "camera_model.h"
int main(int argc, char** argv)
{	
	waitKey(1000);
   ros::init(argc, argv, "find_objectblob");
   trackobjects top_cam("1"), top_right_cam("2"), bottom_right_cam("3"), 
				bottom_cam("4"), bottom_left_cam("5"), top_left_cam("6"),
				down_cam();
   angleFinder angler;
   ros::spin();
   return 0;
}
