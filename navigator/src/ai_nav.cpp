#include "ai_nav.h"
ai_navigator::ai_navigator()
{
	//info subs:
	curent_pose_sub = n_.subscribe("curent_pose", 1, &ai_navigator::current_pose_cb, this);
	red_plate_poses_sub = n_.subscribe("red_plate_poses", 1, &ai_navigator::red_plate_poses_cb, this);
	green_plate_poses_sub = n_.subscribe("green_plate_poses", 1, &ai_navigator::green_plate_poses_cb, this);
	obstacles_sub = n_.subscribe("obstacles", 1, &ai_navigator::obstacles_cb, this);
	
	//control pubs:
	retractMsg_pub = n_.advertise<std_msgs::Bool>("retractMsg", 1);//true = retracts down, false = up;
	pid_XY_pub = n_.advertise<std_msgs::Int32MultiArray>("pid_XY", 1); //{p, i, d, min, max}
	pid_z_pub = n_.advertise<std_msgs::Int32MultiArray>("pid_z",1 );//{p, i, d, min, max}
	modeMsg_pub = n_.advertise<std_msgs::Int8>("modeMsg", 1);//0 = altitude hold, 1 = stabilize, 2 = land;

	
}

void ai_navigator::init()
{
	ros::Rate nav_rate(20);
	while (ros::ok())
	{
		switch(determine_state())
		{
			case TakeOff:
			cur_state = TakeOff;
			case RandomTraversal:
			cur_state = TakeOff;
			case VerifyRobotRotation:
			cur_state = TakeOff;
			case InteractWithRobot:
			cur_state = TakeOff;
			case AvoidObstacle:
			cur_state = TakeOff;
			case HoldPosition:
			cur_state = TakeOff;
			case Land:
			cur_state = TakeOff;
		}
		ros::spinOnce;
		nav_rate.sleep();
	}
}


///Determine State:
ai_navigator::state ai_navigator::determine_state()
{
	
}


/**************************************************************
 * Action Functions:
 * ***********************************************************/
void ai_navigator::take_off()
{
	
}

void ai_navigator::random_traversal()
{
	
}

void ai_navigator::verify_robot_rotation()
{
	
}

void ai_navigator::interact_with_robot()
{
	
}

void ai_navigator::avoid_obstacle()
{
	
}

void ai_navigator::land()
{
	
}

/*****************************************************************
 * Data callbacks:
 * ***************************************************************/
void ai_navigator::current_pose_cb(const geometry_msgs::PoseStamped msg)
{
	
}

void ai_navigator::red_plate_poses_cb(const geometry_msgs::PoseArray msg)
{
	
}

void ai_navigator::green_plate_poses_cb(const geometry_msgs::PoseArray msg)
{
	
}

void ai_navigator::obstacles_cb(const geometry_msgs::PoseArray msg)
{
	
}
