#pragma once

#include "ground_robot.h"

/*
1. Trajectory Noise Occurs = 5 second interval
2. Amplitude of Trajectory Noise = 0 degrees <= noise <= 20 degrees
3. Period of Trajectory Reversal = 20 seconds
4. Direction of Trajectory Reversal = 180 degrees ("clockwise")
5. Rotation of Trajectory for Collision = 180 degrees ("clockwise")
6. Rotation of Trajectory for "Top Touch" = 45 degrees ("clockwise")
7. Speed = 0.33 m/s
8. Initial Radius of Mission Robots (all facing outward, equally spaced) = 1 m
*/

class herdable_ground_robot : ground_robot
{
public:
	herdable_ground_robot();
	~herdable_ground_robot();


private:

};

