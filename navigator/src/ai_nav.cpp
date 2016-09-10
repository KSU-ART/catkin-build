
#include "ai_nav.h"
//test
ai_navigator::ai_navigator()
{
	/// ************** Constants *****************
	SETPOINT_INTERVAL = 3.0;
	TARGET_ALTITUDE = 1.0;
	GOAL_ANGLE = PI / 180 * 45;

	hank3.start_pose.pose.position.y = 10;
	hank3.start_pose.pose.position.x = -1;
	hank3.start_pose.pose.position.z = UAV_HEIGHT;

	start_time = ros::Time::now().toSec();

	//initialize vars:
	cur_state = Land;

	//info subs:
	curent_pose_sub = n_.subscribe("/localizer/current_pose", 1, &ai_navigator::current_pose_cb, this);
	red_plate_poses_sub = n_.subscribe("/observer/red_plate_poses", 1, &ai_navigator::red_plate_poses_cb, this);
	green_plate_poses_sub = n_.subscribe("/observer/green_plate_poses", 1, &ai_navigator::green_plate_poses_cb, this);
	//obstacles_sub = n_.subscribe("/observer/obstacles", 1, &ai_navigator::obstacles_cb, this);

	//control pubs:
	setpoint_pub = n_.advertise<geometry_msgs::Point>("/navigator/setpoint", 1);
	retractMsg_pub = n_.advertise<std_msgs::Bool>("/navigator/retractMsg", 1);//true = retracts down, false = up;
	pid_XY_pub = n_.advertise<std_msgs::Int32MultiArray>("/navigator/pid_XY", 1); //{p, i, d, min, max}
	pid_z_pub = n_.advertise<std_msgs::Int32MultiArray>("/navigator/pid_z", 1);//{p, i, d, min, max}
	modeMsg_pub = n_.advertise<std_msgs::Int8>("/navigator_nav/modeMsg", 1);//0 = altitude hold, 1 = stabilize, 2 = land;

}

void ai_navigator::init()
{
	ros::Rate nav_rate(20);
	while (ros::ok())
	{
		determine_state();
		
		if(cur_state == TargetGR)
		{
			
			target_ground_robot();
		}

		// TODO ?
		if (at_setpoint)
		{
			switch (cur_state)
			{
			case TakeOff:
				take_off();
				break;
				
			case RandomTraversal:
				random_traversal();
				break;
				
			case InteractWithRobot:
				interact_with_robot();
				break;
				
			// TODO
			case AvoidObstacle:
				avoid_obstacle();
				break;
				
			case HoldPosition:
				hold_position();
				break;
				
			// TODO
			case Land:
				land();
				break;
			}
		}

#ifdef DEBUG_MODE
		std::cout << "Current State" << cur_state << std::endl;
#endif // DEBUG_MODE

		ros::spinOnce;
		nav_rate.sleep();
	}
}

///Determine State:
ai_navigator::state ai_navigator::determine_state()
{
	if(	abs(hank3.current_pose.pose.position.x - setpoint.x) < 0.1 &&
		abs(hank3.current_pose.pose.position.y - setpoint.y) < 0.1 &&
		abs(hank3.current_pose.pose.position.z - setpoint.z) < 0.1)
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
			double x = min_loc_r.position.x - hank3.current_pose.pose.position.x;
			double y = min_loc_r.position.y - hank3.current_pose.pose.position.y;
			dist_r = x*x + y*y;
		}
		if (found_green)
		{
			double x = min_loc_g.position.x - hank3.current_pose.pose.position.x;
			double y = min_loc_g.position.y - hank3.current_pose.pose.position.y;
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
	// TODO:: remove
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
	
	setpoint.x = hank3.start_pose.pose.position.x;
	setpoint.y = hank3.start_pose.pose.position.y;
	setpoint.z = TARGET_ALTITUDE;
	setpoint_pub.publish(setpoint);
}

// TODO:: make more flight plans
void ai_navigator::random_traversal()
{
	// TODO:: remove
	if(new_state)
	{
		state_time = ros::Time::now().toSec();
		new_state = false;
	}

	if (hank3.flight_plan_index >= size_of_default_flight_plan_xy)
	{
		hank3.flight_plan_index = 0;
	}

	setpoint.x = default_flight_plan_xy[hank3.flight_plan_index++];
	setpoint.y = default_flight_plan_xy[hank3.flight_plan_index++];
	setpoint.z = TARGET_ALTITUDE;
	setpoint_pub.publish(setpoint);
}

void ai_navigator::target_ground_robot()
{
	// TODO:: remove
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
				double cur_angle = 2*acos(hank3.current_pose.pose.orientation.w); 
				double angle = gr_angle + cur_angle;
				
				crop_angle(angle);
				
				while (angle < 180-GOAL_ANGLE || angle > 180+GOAL_ANGLE)
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
			// TODO
			//go to random_traversal
			/// **************************** FOR TEST USING Hold position INSTEAD *************************************
			new_state = true;
			
			//cur_state = HoldPosition;
			cur_state = RandomTraversal;
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
	
	if(hank3.rotate_gr_counter < num_of_presses)
	{		
		if (hank3.current_altitude_state == HOVERING_ABOVE_GR)
		{
			setpoint.x = target_gr.position.x;
			setpoint.y = target_gr.position.y;
			setpoint.z = INTERACTING_WITH_GR;
		} 
		else if (hank3.current_altitude_state == INTERACTING_WITH_GR)
		{
			setpoint.x = target_gr.position.x;
			setpoint.y = target_gr.position.y;
			setpoint.z = HOVERING_ABOVE_GR;
			hank3.rotate_gr_counter++;
		}
		
	}
	else
	{
		if (hank3.current_altitude_state == INTERACTING_WITH_GR)
		{			
			setpoint.x = hank3.current_pose.pose.position.x;
			setpoint.y = hank3.current_pose.pose.position.y;
			setpoint.z = TARGET_ALTITUDE;
		}
		
		hank3.rotate_gr_counter = 0;
		hank3.rotate_gr_state_desired = ROTATE_NONE;
		cur_state = TargetGR;
		new_state = true;
		
	}
	
	setpoint_pub.publish(setpoint);
}

void ai_navigator::avoid_obstacle()
{
	if(new_state)
	{
		state_time = ros::Time::now().toSec();
		new_state = false;
	}

	if (hank3.current_altitude_state == HOVERING_ABOVE_OBSTACLES)
	{
		
	}
	else
	{
		
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
	
	setpoint.x = hank3.current_pose.pose.position.x;
	setpoint.y = hank3.current_pose.pose.position.y;
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
	setpoint.x = hank3.current_pose.pose.position.x;
	setpoint.y = hank3.current_pose.pose.position.y;
	setpoint.z = UAV_HEIGHT;
	setpoint_pub.publish(setpoint);
}

/*****************************************************************
 * Data callbacks:
 * ***************************************************************/
void ai_navigator::current_pose_cb(const geometry_msgs::PoseStamped& msg)
{
	hank3.current_pose = msg;

	double hank3_x = hank3.current_pose.pose.position.x + (UAV_WIDTH/2);
	double hank3_y = hank3.current_pose.pose.position.y + (UAV_LENGTH/2);
	double hank3_z = hank3.current_pose.pose.position.z + (UAV_HEIGHT/2);

	// TODO:: uav_state.update( altitude )
	if (hank3_z > ARENA_MAX_Z)
	{
		hank3.current_altitude_state = ABOVE_ARENA;
	}
	else if (hank3_z > MAX_PILLAR_HEIGHT)
	{
		hank3.current_altitude_state = HOVERING_ABOVE_OBSTACLES;
	}
	else if (hank3_z >= 1)
	{
		hank3.current_altitude_state = HOVERING_ABOVE_GR;
	}
	else if (hank3_z > GROUND_ROBOT_HEIGHT)
	{
		hank3.current_altitude_state = INTERACTING_WITH_GR;
	}
	else if (hank3.current_pose.pose.position.z == hank3.start_pose.pose.position.z)
	{
		hank3.current_altitude_state = ON_GROUND;
	}

	// TODO:: uav_state.update( is in arena )
	if (hank3_x > ARENA_MIN_X
	&& hank3_x < ARENA_MAX_X)
	{
		// inside x bounds
		hank3.inside_arena = true;
	}
	else
	{
		// outside x bounds
		hank3.inside_arena = false;
		
		// move to known set point inside arena		
		setpoint.x = 10;
		setpoint.y = 2;
		setpoint.z = SAFE_FLYING_ALTITUDE;
	}
	
	if (hank3_y > ARENA_MIN_Y
	&& hank3_y < ARENA_MAX_Y)
	{
		// inside y bounds
		hank3.inside_arena = true;
	}
	else
	{
		// outside y bounds
		hank3.inside_arena = false;
		
		// move to known set point inside arena		
		setpoint.x = 10;
		setpoint.y = 2;
		setpoint.z = SAFE_FLYING_ALTITUDE;
	}

	// setpoint based on current flight plan
	setpoint_pub.publish(setpoint);
}

void ai_navigator::red_plate_poses_cb(const geometry_msgs::PoseArray& msg)
{
	if (msg.poses.size() > 0)
	{
		found_red = true;

		// calc MINIMUM distance uav is from FIRST red plate XY_POINT
		double x = msg.poses[0].position.x - hank3.current_pose.pose.position.x;
		double y = msg.poses[0].position.y - hank3.current_pose.pose.position.y;
		double min_dist = x*x + y*y;

		for (int i = 1; i < msg.poses.size(); i++)
		{
			// calc distance uav is from plate XY_POINT
			double x = msg.poses[i].position.x - hank3.current_pose.pose.position.x;
			double y = msg.poses[i].position.y - hank3.current_pose.pose.position.y;
			double dist = x*x + y*y;

			if (dist < min_dist)
			{
				min_dist = dist;
				min_loc_r = msg.poses[i];
			}
		}
		
		// TODO:: verify centered above target_gr
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

		// calc MINIMUM distance uav is from FIRST green plate XY_POINT
		double x = msg.poses[0].position.x - hank3.current_pose.pose.position.x;
		double y = msg.poses[0].position.y - hank3.current_pose.pose.position.y;
		double min_dist = x*x + y*y;
		
		for (int i = 1; i < msg.poses.size(); i++)
		{
			// calc distance uav is from plate XY_POINT
			double x = msg.poses[i].position.x - hank3.current_pose.pose.position.x;
			double y = msg.poses[i].position.y - hank3.current_pose.pose.position.y;
			double dist = x*x + y*y;

			if (dist < min_dist)
			{
				min_dist = dist;
				min_loc_r = msg.poses[i];
			}
		}
		
		// TODO:: verify centered above target_gr
		
	}
	else
	{
		found_green = false;
	}
}

/*
void ai_navigator::obstacles_cb(const geometry_msgs::PoseArray msg)
{
	//printf( "frame_id: %s stamp: %d\n", g_oa.header.frame_id.c_str(), g_oa.header.stamp.sec );
	//printf( "obstacle distance: [%f %f %f %f %f]\n", g_oa.ranges[0], g_oa.ranges[1], g_oa.ranges[2], g_oa.ranges[3], g_oa.ranges[4] );
	
	for (int i = 0; i < msg.poses.size(); i++)
	{
		obstacle_ground_robot obstacle_gr;
		obstacle_gr.current_pose = msg.poses[i];

		if (hank3.nearby_obstacle_gr_list.empty())
		{
			hank3.nearby_obstacle_gr_list.push_back(obstacle_gr);
		}
		else
		{

		}
	}
}
*/
