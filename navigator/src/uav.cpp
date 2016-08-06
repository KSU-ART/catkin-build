#include "uav.h"

uav::uav()
{
	current_altitude_state = ON_GROUND;
	inside_arena = false;
	flight_plan_index = 0;
	
	rotate_gr_counter = 0;
	rotate_gr_state_desired = ROTATE_NONE;
}

uav::~uav()
{
}

/*
bool uav::herd_ground_robot(herdable_ground_robot target_ground_robot)
{
	
}
*/
