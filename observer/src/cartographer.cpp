///cartographer
#include "cartographer.h"

using namespace std;
using namespace cv;
using namespace projection_;
cartographer::cartographer()
{
	c1.loadModel('1');
	c2.loadModel('2');
	c3.loadModel('3');
	c4.loadModel('4');
	c5.loadModel('5');
	c6.loadModel('6');
	c7.loadModel('7');
	curr_pose = s_.subscribe("curent_pose", 3, &cartographer::update_pose, this);
	r1 = s_.subscribe("r_cam_points_1", 3, &cartographer::cam_1_callback, this);
	r2 = s_.subscribe("r_cam_points_2", 3, &cartographer::cam_2_callback, this);
	r3 = s_.subscribe("r_cam_points_3", 3, &cartographer::cam_3_callback, this);
	r4 = s_.subscribe("r_cam_points_4", 3, &cartographer::cam_4_callback, this);
	r5 = s_.subscribe("r_cam_points_5", 3, &cartographer::cam_5_callback, this);
	r6 = s_.subscribe("r_cam_points_6", 3, &cartographer::cam_6_callback, this);
	r7 = s_.subscribe("r_cam_points_7", 3, &cartographer::cam_7_callback, this);
	g1 = s_.subscribe("g_cam_points_1", 3, &cartographer::cam_1_callback, this);
	g2 = s_.subscribe("g_cam_points_2", 3, &cartographer::cam_2_callback, this);
	g3 = s_.subscribe("g_cam_points_3", 3, &cartographer::cam_3_callback, this);
	g4 = s_.subscribe("g_cam_points_4", 3, &cartographer::cam_4_callback, this);
	g5 = s_.subscribe("g_cam_points_5", 3, &cartographer::cam_5_callback, this);
	g6 = s_.subscribe("g_cam_points_6", 3, &cartographer::cam_6_callback, this);
	g7 = s_.subscribe("g_cam_points_7", 3, &cartographer::cam_7_callback, this);
}
cartographer::~cartographer()
{
	
}
void cartographer::verify_positions_location()
{
	
}
void cartographer::sort_positions_time()
{
	
}
void cartographer::update_pose(const geometry_msgs::PoseStamped& cur_loc) 
{
	
}
void cartographer::cam_1_callback(const std_msgs::UInt32MultiArray& msg)
{
	
}
void cartographer::cam_2_callback(const std_msgs::UInt32MultiArray& msg)
{
	
}
void cartographer::cam_3_callback(const std_msgs::UInt32MultiArray& msg)
{
	
}
void cartographer::cam_4_callback(const std_msgs::UInt32MultiArray& msg)
{
	
}
void cartographer::cam_5_callback(const std_msgs::UInt32MultiArray& msg)
{
	
}
void cartographer::cam_6_callback(const std_msgs::UInt32MultiArray& msg)
{
	
}
void cartographer::cam_7_callback(const std_msgs::UInt32MultiArray& msg)
{
	
}


		
