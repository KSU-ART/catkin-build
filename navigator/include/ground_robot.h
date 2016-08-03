#pragma once

// units: meters
#define GROUND_ROBOT_WIDTH 0.33
#define GROUND_ROBOT_LENGTH 0.33
#define GROUND_ROBOT_HEIGHT 0.087

// units: meters/second
#define GROUND_ROBOT_SPEED 0.33

enum gr_state
{
	UNKOWN,
	TURNING_45_DEGREES,
	TURNING_180_DEGREES,
	MOVING_FORWARD
};

class ground_robot
{
public:
	ground_robot();
	~ground_robot();

	// current_pose is at the center of the ground robot
	geometry_msgs::PoseStamped current_pose;
	gr_state current_state;
	bool inside_arena;

private:

};

ground_robot::ground_robot()
{
	current_state = gr_state::UNKOWN;
	inside_arena = true;
}

ground_robot::~ground_robot()
{
}
