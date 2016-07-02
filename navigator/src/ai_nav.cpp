#include "ai_nav.h"
ai_navigator::ai_navigator()
{
	/*curent_pose_sub = n_.subscribe("curent_pose", 1, ai_navigator::curent_pose_cb);
	red_plate_poses_sub = n_.subscribe("red_plate_poses", 1, ai_navigator::red_plate_poses_cb);
	green_plate_poses_sub = n_.subscribe("green_plate_poses", 1, ai_navigator::green_plate_poses_cb);
	obstacles_sub = n_.subscribe("obstacles", 1, ai_navigator::obstacles_cb);
	
	retractMsg_pub = n_.advertise<std_msgs::Bool>("retractMsg", 1);//true = retracts down, false = up;
	pid_XY_pub = n_.advertise<std_msgs::Int32MultiArray>("pid_XY", 1); //{p, i, d, min, max}
	pid_z_pub = n_.advertise<std_msgs::Int32MultiArray>("pid_z",1 );//{p, i, d, min, max}
	modeMsg_pub = n_.advertise<std_msgs::Int8>("modeMsg", 1);//0 = altitude hold, 1 = stabilize, 2 = land;*/
}
