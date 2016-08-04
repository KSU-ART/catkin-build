#pragma once

#include "ground_robot.h"

/*
1. Trajectory Noise Occurs = Never
2. Amplitude of Trajectory Noise = 0°
3. Period of Trajectory Reversal = Never
4. Direction of Trajectory Reversal = 0°
5. Rotation of Trajectory for Collision = 0°
6. Rotation of Trajectory for “Top Touch” = 0°
7. Speed = 0.33 m/s
8. Initial Radius of Obstacle Robots = 5 m
9. Initial Direction = “clockwise”
10. Trajectory = 10 m diameter circle (centered on arena)
*/

// units: meters
<<<<<<< HEAD
const int MAX_PILLAR_HEIGHT = 2;
=======
#define MAX_PILLAR_HEIGHT 2
>>>>>>> f8a486467cd392ccb3654c62785ca9e2999e52c9

class obstacle_ground_robot : ground_robot
{
public:
	obstacle_ground_robot();
	~obstacle_ground_robot();

	// units: meters
	double pillar_width = GROUND_ROBOT_WIDTH;
	double pillar_length = GROUND_ROBOT_LENGTH;
	double pillar_height = GROUND_ROBOT_HEIGHT + MAX_PILLAR_HEIGHT;

	// pillar_pose is at the center of the pillar
	geometry_msgs::PoseStamped pillar_pose;
	
private:

};

<<<<<<< HEAD
=======
obstacle_ground_robot::obstacle_ground_robot()
{
}

obstacle_ground_robot::~obstacle_ground_robot()
{
}
>>>>>>> f8a486467cd392ccb3654c62785ca9e2999e52c9
