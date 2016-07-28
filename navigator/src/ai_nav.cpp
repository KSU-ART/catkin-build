
#include "ai_nav.h"
ai_navigator::ai_navigator()
{
	/// ************** Constants *****************
	SETPOINT_INTERVAL = 3.0;
	DEBUG = true;
	TARGET_ALTITUDE = 1.0;
	GOAL_ANGLE = PI/180 * 45;
	
	start_time = ros::Time::now().toSec();
	
	//initialize vars:
	cur_state = Land;
	
	//info subs:
	curent_pose_sub = n_.subscribe("/localizer/current_pose", 1, &ai_navigator::current_pose_cb, this);
	red_plate_poses_sub = n_.subscribe("/observer/red_plate_poses", 1, &ai_navigator::red_plate_poses_cb, this);
	green_plate_poses_sub = n_.subscribe("/observer/green_plate_poses", 1, &ai_navigator::green_plate_poses_cb, this);
	obstacles_sub = n_.subscribe("/observer/obstacles", 1, &ai_navigator::obstacles_cb, this);
	
	//control pubs:
	setpoint_pub = n_.advertise<geometry_msgs::Point>("/navigator/setpoint", 1);
	retractMsg_pub = n_.advertise<std_msgs::Bool>("/navigator/retractMsg", 1);//true = retracts down, false = up;
	pid_XY_pub = n_.advertise<std_msgs::Int32MultiArray>("/navigator/pid_XY", 1); //{p, i, d, min, max}
	pid_z_pub = n_.advertise<std_msgs::Int32MultiArray>("/navigator/pid_z",1 );//{p, i, d, min, max}
	modeMsg_pub = n_.advertise<std_msgs::Int8>("/ainavigator_nav/modeMsg", 1);//0 = altitude hold, 1 = stabilize, 2 = land;
	
}

void ai_navigator::init()
{
	ros::Rate nav_rate(20);
	while (ros::ok())
	{
		determine_state();
		switch(cur_state)
		{
			case TakeOff:
				take_off();
				break;
			case RandomTraversal:
				random_traversal();
				break;
			case TargetGR:
				target_ground_robot();
				break;
			case InteractWithRobot:
				interact_with_robot();
				break;
			case AvoidObstacle:
				avoid_obstacle();
				break;
			case HoldPosition:
				hold_position();
				break;
			case Land:
				land();
				break;
		}
		if (DEBUG)
		{
			std::cout << "Current State" << cur_state << std::endl;
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
	
	//If it is the first five seconds the state should be takeoff
	if( (ros::Time::now().toSec() <= (start_time + 5.00) ) )
	{
		if(cur_state != TakeOff)
			new_state = true;
		cur_state = TakeOff;
		return cur_state;
	}
}

bool ai_navigator::find_target()
{
	if (found_red || found_green)
	{
		/// finds the closest between red or green
		double dist_r = 100000;
		double dist_g = 100000;
		if (found_red)
		{
			double x = min_loc_r.position.x - current_pose.pose.position.x;
			double y = min_loc_r.position.y - current_pose.pose.position.y;
			dist_r = x*x + y*y;
		}
		if (found_green)
		{
			double x = min_loc_g.position.x - current_pose.pose.position.x;
			double y = min_loc_g.position.y - current_pose.pose.position.y;
			dist_g = x*x + y*y;
		}
		if (dist_r < dist_g )
		{
			target_gr = min_loc_r;
		}
		else
		{
			target_gr = min_loc_g;
		}
		return true;
	}
	return false;
}

void ai_navigator::crop_angle(double& angle)
{
	// set within 0 to 2pi
	if (angle < 0)
	{
		while (angle < 0)
		{
			angle += 2*PI;
		}
	}
	else if (angle >= 2*PI)
	{
		while (angle >= 2*PI)
		{
			angle -= 2*PI;
		}
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
	
	if( (ros::Time::now().toSec() > (start_time + 5.00) ) )
	{
		new_state = true;
		cur_state = TargetGR;
		return;
	}
	
	setpoint.x = 0;
	setpoint.y = 0;
	setpoint.z = TARGET_ALTITUDE;
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

void ai_navigator::target_ground_robot()
{
	if(new_state)
	{
		state_time = ros::Time::now().toSec();
		new_state = false;
		
		setpoint_start_time = state_time;
	}
	
	/// Find a ground robot and set a setpoint, if not go to random_traversal
	if (ros::Time::now().toSec() >= setpoint_start_time + SETPOINT_INTERVAL)
	{
		setpoint_start_time = ros::Time::now().toSec();
		
		if (find_target())
		{
			setpoint = target_gr.position;
			setpoint_pub.publish(setpoint);
			// checks for orientation (x == 1 if not found any angle)
			if (target_gr.orientation.x != 1) 
			{
				// in radians
				double gr_angle = 2*acos(target_gr.orientation.w); 
				double cur_angle = 2*acos(current_pose.pose.orientation.w); 
				double angle = gr_angle + cur_angle;
				
				crop_angle(angle);
				
				int num_of_presses = 0;
				while (angle < 180 - GOAL_ANGLE || angle > 180 + GOAL_ANGLE)
				{
					num_of_presses++;
					angle -= 45;
					crop_angle(angle);
				}
				if (num_of_presses > 0)
				{
					//go to interact ground robot
					new_state = true;
					cur_state = InteractWithRobot;
					return;
				}
			}
		}
		else
		{
			//go to random_traversal
			/// **************************** FOR TEST USING Hold position INSTEAD *************************************
			new_state = true;
			cur_state = HoldPosition;
			return;
		}
	}
}

void ai_navigator::interact_with_robot()
{
	if (new_state)
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
		
		std_msgs::Bool retract;
		retract.data = false;
		retractMsg_pub.publish(retract);
	}
	setpoint.x = 0;
	setpoint.y = 0;
	setpoint.z = TARGET_ALTITUDE;
	setpoint_pub.publish(setpoint);
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
void ai_navigator::current_pose_cb(const geometry_msgs::PoseStamped& msg)
{
	current_pose = msg;
}

void ai_navigator::red_plate_poses_cb(const geometry_msgs::PoseArray& msg)
{
	if (msg.poses.size() > 0)
	{
		found_red = true;
		double x = msg.poses[0].position.x - current_pose.pose.position.x;
		double y = msg.poses[0].position.y - current_pose.pose.position.y;
		double min_dist = x*x + y*y;
		for (int i = 1; i < msg.poses.size(); i++)
		{
			double x = msg.poses[i].position.x - current_pose.pose.position.x;
			double y = msg.poses[i].position.y - current_pose.pose.position.y;
			double dist = x*x + y*y;
			if (dist < min_dist)
			{
				min_dist = dist;
				min_loc_r = msg.poses[i];
			}
		}
	}
	else
	{
		found_red = false;
	}
}

void ai_navigator::green_plate_poses_cb(const geometry_msgs::PoseArray& msg)
{
	if (msg.poses.size() > 0)
	{
		found_green = true;
		double x = msg.poses[0].position.x - current_pose.pose.position.x;
		double y = msg.poses[0].position.y - current_pose.pose.position.y;
		double min_dist = x*x + y*y;
		for (int i = 1; i < msg.poses.size(); i++)
		{
			double x = msg.poses[i].position.x - current_pose.pose.position.x;
			double y = msg.poses[i].position.y - current_pose.pose.position.y;
			double dist = x*x + y*y;
			if (dist < min_dist)
			{
				min_dist = dist;
				min_loc_r = msg.poses[i];
			}
		}
	}
	else
	{
		found_green = false;
	}
}

void ai_navigator::obstacles_cb(const geometry_msgs::PoseArray& msg)
{
	
}
