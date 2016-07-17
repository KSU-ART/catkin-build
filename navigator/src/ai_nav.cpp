
#include "ai_nav.h"
ai_navigator::ai_navigator()
{
	start_time = ros::Time::now().toSec();
	
	//initialize vars:
	cur_state = Land;
	
	//info subs:
	curent_pose_sub = n_.subscribe("/curent_pose", 1, &ai_navigator::current_pose_cb, this);
	red_plate_poses_sub = n_.subscribe("/observer/red_plate_poses", 1, &ai_navigator::red_plate_poses_cb, this);
	green_plate_poses_sub = n_.subscribe("/observer/green_plate_poses", 1, &ai_navigator::green_plate_poses_cb, this);
	obstacles_sub = n_.subscribe("/observer/obstacles", 1, &ai_navigator::obstacles_cb, this);
	
	//control pubs:
	setpoint_pub = n_.advertise<geometry_msgs::Point>("/ai_nav/setpoint", 1);
	retractMsg_pub = n_.advertise<std_msgs::Bool>("/ai_nav/retractMsg", 1);//true = retracts down, false = up;
	pid_XY_pub = n_.advertise<std_msgs::Int32MultiArray>("/ai_nav/pid_XY", 1); //{p, i, d, min, max}
	pid_z_pub = n_.advertise<std_msgs::Int32MultiArray>("/ai_nav/pid_z",1 );//{p, i, d, min, max}
	modeMsg_pub = n_.advertise<std_msgs::Int8>("/ai_nav/modeMsg", 1);//0 = altitude hold, 1 = stabilize, 2 = land;

	
}

void ai_navigator::init()
{
	ros::Rate nav_rate(20);
	while (ros::ok())
	{
		switch(determine_state())
		{
			case TakeOff:
			take_off();
			case RandomTraversal:
			random_traversal();
			case VerifyRobotRotation:
			verify_robot_rotation();
			case InteractWithRobot:
			interact_with_robot();
			case AvoidObstacle:
			avoid_obstacle();
			case HoldPosition:
			hold_position();
			case Land:
			land();
		}
		ros::spinOnce;
		nav_rate.sleep();
	}
}


///Determine State:
ai_navigator::state ai_navigator::determine_state()
{
	if(	abs(current_pose.pose.position.x - setpoint.x) < 0.1 &&
		abs(current_pose.pose.position.y - setpoint.y) < 0.1 &&
		abs(current_pose.pose.position.z - setpoint.z) < 0.1)
	{
		at_setpoint = true;
	}
	else
	{
		at_setpoint = false;
	}
	if( (ros::Time::now().toSec() > (start_time + 5.00) ) && !at_setpoint )
	{
		if(cur_state != TakeOff)
			new_state = true;
		cur_state = TakeOff;
		return cur_state;
	}
	else
	{
		if(cur_state != HoldPosition)
			new_state = true;
		cur_state = HoldPosition;
		return cur_state;
	}
}


/**************************************************************
 * Action Functions:
 * ***********************************************************/
void ai_navigator::take_off()
{
	if(new_state)
	{
		state_time = ros::Time::now().toSec();
		new_state = false;
	}
	setpoint.x = 0;
	setpoint.y = 0;
	setpoint.z = 1;
	setpoint_pub.publish(setpoint);
}

void ai_navigator::random_traversal()
{
		if(new_state)
	{
		state_time = ros::Time::now().toSec();
		new_state = false;
	}
}

void ai_navigator::verify_robot_rotation()
{
	if(new_state)
	{
		state_time = ros::Time::now().toSec();
		new_state = false;
	}
}

void ai_navigator::interact_with_robot()
{
	if(new_state)
	{
		state_time = ros::Time::now().toSec();
		new_state = false;
	}
}

void ai_navigator::avoid_obstacle()
{
	if(new_state)
	{
		state_time = ros::Time::now().toSec();
		new_state = false;
	}
}

void ai_navigator::hold_position()
{
	if(new_state)
	{
		state_time = ros::Time::now().toSec();
		new_state = false;
	}
	
}

void ai_navigator::land()
{
	if(new_state)
	{
		state_time = ros::Time::now().toSec();
		new_state = false;
	}
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
