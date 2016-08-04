#pragma once

#include <vector>
#include "obstacle_ground_robot.h"
#include "herdable_ground_robot.h"

/*
1. 20 m x 20 m Square
2. Red boundary at one Edge
3. Green Boundary at Opposite Edge
4. White Boundaries on either Side
5. White Center Line Bisecting Square and Parallel with Green and Red Edges
6. White 1 m Circle at Geometric Center of Arena to Facilitate Initial Robot Placement
7. Four White Dots to Mark starting Location for Obstacle Robots.
8. Boundary Line Width = ~8 cm (two-dimensional tape or paint on ground)
*/

// Dimension units in meters

// width  == x
<<<<<<< HEAD
const double ARENA_MAX_X = 20;
const double ARENA_MIN_X = 0;
// length == y
const double ARENA_MAX_Y = 20; 
const double ARENA_MIN_Y = 0;
// height == z
const double ARENA_MAX_Z = 3;
const double ARENA_MIN_Z = 0;
=======
#define ARENA_MAX_X 20
#define ARENA_MIN_X 0

// length == y
#define ARENA_MAX_Y 20
#define ARENA_MIN_Y 0

// height == z
#define ARENA_MAX_Z 3
#define ARENA_MIN_Z 0
>>>>>>> f8a486467cd392ccb3654c62785ca9e2999e52c9

class arena
{
public:
	arena();
	~arena();

	//nav_msgs::OccupancyGrid mini_map;
	//std::vector<herdable_ground_robot> *herdable_gr_list;
	//std::vector<obstacle_ground_robot> *obstacle_gr_list;
private:

};
<<<<<<< HEAD
=======

arena::arena()
{
	//herdable_gr_list = new std::vector<herdable_ground_robot>(10);
	//obstacle_gr_list = new std::vector<obstacle_ground_robot>(4);
}

arena::~arena()
{
	//delete herdable_gr_list;
	//delete obstacle_gr_list;
}
>>>>>>> f8a486467cd392ccb3654c62785ca9e2999e52c9
