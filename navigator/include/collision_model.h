#pragma once

class collision_model
{
public:
	collision_model();
	~collision_model();

	static int
		width,
		length,
		height;

	static int
		padding_x,
		padding_y,
		padding_z;

	geometry_msgs::PoseStamped center_pose;
private:

};

collision_model::collision_model()
{
}

collision_model::~collision_model()
{
}