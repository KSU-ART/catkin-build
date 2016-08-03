#pragma once

#include <vector>
#include "collision_model.h"
#include "herdable_ground_robot.h"
#include "obstacle_ground_robot.h"

///PWM values can range from 1000 to 2000
#define LOW_PWM 1000
#define MID_PWM 1500
#define HIGH_PWM 2000

#define LAND_MODE 1000
#define STABILIZE_MODE 1500
#define ALT_HOLD_MODE 2000

///could be useful later:
//#define ROLL_TRIM 1513
//#define PITCH_TRIM 1510

// units: meters
#define UAV_WIDTH 1.1
#define UAV_LENGTH 1.1
#define UAV_HEIGHT 0.147

// altitude_state.ABOVE_OBSTACLES
#define SAFE_FLYING_ALTITUDE 2.5

enum rc_channel
{
	ROLL = 0, // CHANNEL #1
	PITCH,
	THROTTLE,
	YAW,
	MODE,
	GRIP,
	RETRACT,
	NOT_USED,
	MANUAL // CHANNEL #9
};

enum altitude_state
{
	UNKOWN,
	ON_GROUND,
	HOVERING_ABOVE_OBSTACLES,
	HOVERING_ABOVE_GR,
	INTERACTING_WITH_GR,
	ABOVE_ARENA
};

enum rotate_gr_state
{
	NONE,
	45_DEGREES,
	90_DEGREES,
	135_DEGREES,
	180_DEGREES,
	225_DEGREES,
	270_DEGREES,
	315_DEGREES
};

// cen_x ... middle of arena ... x
// cen_y ... middle of arena ... y
//static const float point_cen_x_min_y = {10, 2}
//static const float point_cen_x_max_y = {10, 18}
//static const float point_min_x_cen_y = {2, 10}
//static const float point_max_x_cen_y = {18, 10}

// circle the arena 2 meters away from the edge
// and move 2 meters each step
// x[0], y[0], x[1], y[1], ... x[N], y[N]
static const float default_flight_plan_xy[] =
{
	10, 2,
	12, 2,
	14, 2,
	16, 2,
	18, 2,

	18, 4,
	18, 6,
	18, 8,
	18, 10,
	18, 12,

	18, 14,
	18, 16,
	18, 18,
	16, 18,
	14, 18,

	12, 18,
	10, 18,
	8, 18,
	6, 18,
	4, 18,

	2, 18,
	2, 16,
	2, 14,
	2, 12,
	2, 10,

	2, 8,
	2, 6,
	2, 4,
	2, 2,
	4, 2,

	6, 2,
	8, 2
};

static const int size_of_default_flight_plan_xy = 64;

class uav
{
public:
	uav();
	~uav();

	//void take_off();
	//void land();
	//void hover();
	//bool herd_ground_robot(herdable_ground_robot target_ground_robot);

	// poses are at the center of the uav
	geometry_msgs::PoseStamped current_pose;
	geometry_msgs::PoseStamped start_pose;
	
	// TODO
	//collision_model uav_collision_model;

	altitude_state current_altitude_state;
	bool inside_arena;
	int flight_plan_index;
	//bool calibrated;
	
	rotate_gr_state rotate_gr_state_actual;
	rotate_gr_state rotate_gr_state_desired;
	
	//std::vector<geometry_msgs::PoseStamped> current_flight_path;

	//std::vector<herdable_ground_robot> nearby_herdable_gr_list;
	//std::vector<obstacle_ground_robot> nearby_obstacle_gr_list;
private:
	
};

uav::uav()
{
	current_altitude_state = altitude_state::ON_GROUND;
	inside_arena = false;
	flight_plan_index = 0;
	
	rotate_gr_state_actual = rotate_gr_state::None;
	rotate_gr_state_desired = rotate_gr_state::None;
}

uav::~uav()
{
}

/*
bool uav::herd_ground_robot(herdable_ground_robot target_ground_robot)
{
	
}
*/
