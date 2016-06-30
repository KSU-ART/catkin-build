///plate_localizer
#include "plate_localizer.h"

using namespace std;
using namespace cv;
using namespace projection_;
plate_localizer::plate_localizer()
{
	c1.loadModel('1');
	c2.loadModel('2');
	c3.loadModel('3');
	c4.loadModel('4');
	c5.loadModel('5');
	c6.loadModel('6');
	c7.loadModel('7');
	
	curr_pose = s_.subscribe("curent_pose", 3, &plate_localizer::update_pose, this);
	r1 = s_.subscribe("r_cam_points_1", 3, &plate_localizer::cam_1r_callback, this);
	r2 = s_.subscribe("r_cam_points_2", 3, &plate_localizer::cam_2r_callback, this);
	r3 = s_.subscribe("r_cam_points_3", 3, &plate_localizer::cam_3r_callback, this);
	r4 = s_.subscribe("r_cam_points_4", 3, &plate_localizer::cam_4r_callback, this);
	r5 = s_.subscribe("r_cam_points_5", 3, &plate_localizer::cam_5r_callback, this);
	r6 = s_.subscribe("r_cam_points_6", 3, &plate_localizer::cam_6r_callback, this);
	r7 = s_.subscribe("r_cam_points_0", 3, &plate_localizer::cam_7r_callback, this);
	g1 = s_.subscribe("g_cam_points_1", 3, &plate_localizer::cam_1g_callback, this);
	g2 = s_.subscribe("g_cam_points_2", 3, &plate_localizer::cam_2g_callback, this);
	g3 = s_.subscribe("g_cam_points_3", 3, &plate_localizer::cam_3g_callback, this);
	g4 = s_.subscribe("g_cam_points_4", 3, &plate_localizer::cam_4g_callback, this);
	g5 = s_.subscribe("g_cam_points_5", 3, &plate_localizer::cam_5g_callback, this);
	g6 = s_.subscribe("g_cam_points_6", 3, &plate_localizer::cam_6g_callback, this);
	g7 = s_.subscribe("g_cam_points_0", 3, &plate_localizer::cam_7g_callback, this);
	
	rpp = s_.advertise<geometry_msgs::PoseArray>("red_plate_poses", 1);
	gpp = s_.advertise<geometry_msgs::PoseArray>("green_plate_poses", 1);
}
plate_localizer::~plate_localizer()
{
	
}
void plate_localizer::merge_positions_location(std::vector<geometry_msgs::Pose> po_v, char color)
{
	bool in_there = false;
	switch (color)
	{
		case 'g':
		for (int i = 0; i < po_v.size(); i++)
		{
			if (po_v[i].position.x < 20 && po_v[i].position.y < 20) //out of bounds?
			{
				for (int j = 0; j < green_groundbots_world_loc.poses.size(); j++)
				{
					if (	abs((green_groundbots_world_loc.poses[j].position.y - po_v[i].position.y)) < 0.3 && 
							abs((green_groundbots_world_loc.poses[j].position.x - po_v[i].position.x)) < 0.3 ) //near a groundbot already in the array? if so update position
					{
						green_groundbots_world_loc.poses[j].position.x = po_v[i].position.x;
						green_groundbots_world_loc.poses[j].position.y = po_v[i].position.y;
						green_groundbots_world_loc.poses[j].position.z = po_v[i].position.z;
						in_there = true;
					}
				}
				if (in_there)
				{
				in_there = false;
				continue;
				}
				else
				{
					green_groundbots_world_loc.poses.push_back(po_v[i]);
				}		
			}
		}
		gpp.publish(green_groundbots_world_loc);
		
		case 'r':
		for (int i = 0; i < po_v.size(); i++)
		{
			if (po_v[i].position.x < 20 && po_v[i].position.y < 20) //out of bounds?
			{
				for (int j = 0; j < red_groundbots_world_loc.poses.size(); j++)
				{
					if (	abs((red_groundbots_world_loc.poses[j].position.y - po_v[i].position.y)) < 0.3 && 
							abs((red_groundbots_world_loc.poses[j].position.x - po_v[i].position.x)) < 0.3 ) //near a groundbot already in the array? if so update position
					{
						red_groundbots_world_loc.poses[j].position.x = po_v[i].position.x;
						red_groundbots_world_loc.poses[j].position.y = po_v[i].position.y;
						red_groundbots_world_loc.poses[j].position.z = po_v[i].position.z;
						in_there = true;
					}
				}
				if (in_there)
				{
					in_there = false;
					continue;
				}
				else
				{
					red_groundbots_world_loc.poses.push_back(po_v[i]);
				}
			}
		}
		rpp.publish(red_groundbots_world_loc);
	}
}
void plate_localizer::point_callback(const std_msgs::Int32MultiArray& msg, char camID, char color)
{
		vector<geometry_msgs::Pose> po_v;
	vector<cv::Point> pt_v;
	cv::Point p_;
	for (int i = 0; i < msg.data.size(); i+=2)
	{
		p_.x = i;
		p_.y = i + 2;
		pt_v.push_back(p_);
	}
	switch (camID)
	{
		case '1':
		for (int i = 0; i < pt_v.size(); i++)
		{
			po_v.push_back( c1.getPlateWorldLocation(uavPose_, pt_v[i]) );
		}
		case '2':
		for (int i = 0; i < pt_v.size(); i++)
		{
			po_v.push_back( c2.getPlateWorldLocation(uavPose_, pt_v[i]) );
		}
		case '3':
		for (int i = 0; i < pt_v.size(); i++)
		{
			po_v.push_back( c3.getPlateWorldLocation(uavPose_, pt_v[i]) );
		}
		case '4':
		for (int i = 0; i < pt_v.size(); i++)
		{
			po_v.push_back( c4.getPlateWorldLocation(uavPose_, pt_v[i]) );
		}
		case '5':
		for (int i = 0; i < pt_v.size(); i++)
		{
			po_v.push_back( c5.getPlateWorldLocation(uavPose_, pt_v[i]) );
		}
		case '6':
		for (int i = 0; i < pt_v.size(); i++)
		{
			po_v.push_back( c6.getPlateWorldLocation(uavPose_, pt_v[i]) );
		}
		case '7':
		for (int i = 0; i < pt_v.size(); i++)
		{
			po_v.push_back( c7.getPlateWorldLocation(uavPose_, pt_v[i]) );
		}
	}
	plate_localizer::merge_positions_location(po_v, color);
}
	
void plate_localizer::update_pose(const geometry_msgs::PoseStamped& cur_loc) 
{
	uavPose_ = cur_loc;
}

void plate_localizer::cam_1r_callback(const std_msgs::Int32MultiArray& msg)
{
	plate_localizer::point_callback(msg, '1', 'r');
}
void plate_localizer::cam_2r_callback(const std_msgs::Int32MultiArray& msg)
{
	plate_localizer::point_callback(msg, '2', 'r');
}
void plate_localizer::cam_3r_callback(const std_msgs::Int32MultiArray& msg)
{
	plate_localizer::point_callback(msg, '3', 'r');
}
void plate_localizer::cam_4r_callback(const std_msgs::Int32MultiArray& msg)
{
	plate_localizer::point_callback(msg, '4', 'r');
}
void plate_localizer::cam_5r_callback(const std_msgs::Int32MultiArray& msg)
{
	plate_localizer::point_callback(msg, '5', 'r');
}
void plate_localizer::cam_6r_callback(const std_msgs::Int32MultiArray& msg)
{
	plate_localizer::point_callback(msg, '6', 'r');
}
void plate_localizer::cam_7r_callback(const std_msgs::Int32MultiArray& msg)
{
	plate_localizer::point_callback(msg, '7', 'r');
}

void plate_localizer::cam_1g_callback(const std_msgs::Int32MultiArray& msg)
{
	plate_localizer::point_callback(msg, '1', 'g');
}
void plate_localizer::cam_2g_callback(const std_msgs::Int32MultiArray& msg)
{
	plate_localizer::point_callback(msg, '2', 'g');
}
void plate_localizer::cam_3g_callback(const std_msgs::Int32MultiArray& msg)
{
	plate_localizer::point_callback(msg, '3', 'g');
}
void plate_localizer::cam_4g_callback(const std_msgs::Int32MultiArray& msg)
{
	plate_localizer::point_callback(msg, '4', 'g');
}
void plate_localizer::cam_5g_callback(const std_msgs::Int32MultiArray& msg)
{
	plate_localizer::point_callback(msg, '5', 'g');
}
void plate_localizer::cam_6g_callback(const std_msgs::Int32MultiArray& msg)
{
	plate_localizer::point_callback(msg, '6', 'g');
}
void plate_localizer::cam_7g_callback(const std_msgs::Int32MultiArray& msg)
{
	plate_localizer::point_callback(msg, '7', 'g');
}


		
