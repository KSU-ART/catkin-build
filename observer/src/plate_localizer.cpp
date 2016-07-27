///plate_localizer
#include "plate_localizer.h"
//~ #include "camera_model.h"

using namespace std;
using namespace cv;
using namespace projection_;

///constructor (init vars, startup maintainence)
plate_localizer::plate_localizer()
{
	//load cameramodels
	c1.loadModel('1');
	c2.loadModel('2');
	c3.loadModel('3');
	c4.loadModel('4');
	c5.loadModel('5');
	c6.loadModel('6');
	c0.loadToMem(2.2e-6, 2.2e-6, (int)480, (int)640, 442.198764d,441.266354d,326.721422d,218.200761d,0.0d,0.0d,-0.1d,1.0d,0.0d,0.0d,0.0d);
	
	//initiate subscribers
	curr_pose = s_.subscribe("/localizer/curent_pose", 3, &plate_localizer::update_pose, this);
	r_plate_angle = s_.subscribe("/observer/red_plate_angle", 3, &plate_localizer::update_g_angle, this);
	g_plate_angle = s_.subscribe("/observer/green_plate_angle", 3, &plate_localizer::update_r_angle, this);
	r1 = s_.subscribe("/observer/r_cam_points_1", 3, &plate_localizer::cam_1r_callback, this);
	r2 = s_.subscribe("/observer/r_cam_points_2", 3, &plate_localizer::cam_2r_callback, this);
	r3 = s_.subscribe("/observer/r_cam_points_3", 3, &plate_localizer::cam_3r_callback, this);
	r4 = s_.subscribe("/observer/r_cam_points_4", 3, &plate_localizer::cam_4r_callback, this);
	r5 = s_.subscribe("/observer/r_cam_points_5", 3, &plate_localizer::cam_5r_callback, this);
	r6 = s_.subscribe("/observer/r_cam_points_6", 3, &plate_localizer::cam_6r_callback, this);
	r0 = s_.subscribe("/observer/r_cam_points_0", 3, &plate_localizer::cam_0r_callback, this);
	g1 = s_.subscribe("/observer/g_cam_points_1", 3, &plate_localizer::cam_1g_callback, this);
	g2 = s_.subscribe("/observer/g_cam_points_2", 3, &plate_localizer::cam_2g_callback, this);
	g3 = s_.subscribe("/observer/g_cam_points_3", 3, &plate_localizer::cam_3g_callback, this);
	g4 = s_.subscribe("/observer/g_cam_points_4", 3, &plate_localizer::cam_4g_callback, this);
	g5 = s_.subscribe("/observer/g_cam_points_5", 3, &plate_localizer::cam_5g_callback, this);
	g6 = s_.subscribe("/observer/g_cam_points_6", 3, &plate_localizer::cam_6g_callback, this);
	g0 = s_.subscribe("/observer/g_cam_points_0", 3, &plate_localizer::cam_0g_callback, this);
	
	//initiate publishers
	rpp = s_.advertise<geometry_msgs::PoseArray>("/observer/red_plate_poses", 1);
	gpp = s_.advertise<geometry_msgs::PoseArray>("/observer/green_plate_poses", 1);
}

///destructor (unused, could be optimized in future)
plate_localizer::~plate_localizer()
{
	
}

void plate_localizer::update_g_angle(const std_msgs::Float32 &msg)
{
	gAngleTime = ros::Time::now().toSec();
	gAngle = msg.data;
}

void plate_localizer::update_r_angle(const std_msgs::Float32 &msg)
{
	rAngleTime = ros::Time::now().toSec();
	rAngle = msg.data;
}
void plate_localizer::checkTimes(char &color)
{
	for (int i = 0; i < gTimeStamps.size(); i++)
	{
		//verify < 3 secs old
		if ( (ros::Time::now().toSec() - gTimeStamps[i]) > 3) //3 secs old
		{
			//erase old element:
			if (color == 'g')
			{
				green_groundbots_world_loc.poses.erase(green_groundbots_world_loc.poses.begin() + i);
				gTimeStamps.erase(gTimeStamps.begin() + i);
				gRotTimeStamps.erase(gRotTimeStamps.begin() + i);
			}
			else
			{
				red_groundbots_world_loc.poses.erase(red_groundbots_world_loc.poses.begin() + i);
				rTimeStamps.erase(rTimeStamps.begin() + i);
				rRotTimeStamps.erase(rRotTimeStamps.begin() + i);
			}
		}
	}
}

///merge locations, discard out of bounds locations
void plate_localizer::merge_positions_location(std::vector<geometry_msgs::Pose> po_v, char color, bool downcam)
{
	bool in_there = false;
	bool angleAdded = false;
	
	if (color == 'g')
	{
		//step through pose vector
		for (int i = 0; i < po_v.size(); i++)
		{
			//less than 20m away from us
			if (po_v[i].position.x < 20.0d && po_v[i].position.y < 20.0) 
			{
				//if directly below us, add angle
				if ( downcam
					&& (po_v[i].position.x < 0.5d && po_v[i].position.y < 0.5d)  // < 0.5 meter away
					&& ( abs(ros::Time::now().toSec() - gAngleTime) < 1.0d ) ) // < 1 sec
				{
					angleAdded = true;
					po_v[i].orientation.x = 0.0d; 
					po_v[i].orientation.y = 0.0d; //always 0
					po_v[i].orientation.z = sin(gAngle/2.0f);
					po_v[i].orientation.w = cos(gAngle/2.0f);
				}
				else
				{
					po_v[i].orientation.x = 0.0d;//always 0
					po_v[i].orientation.y = 0.0d; //always 0
					po_v[i].orientation.z = 0.0d;//unknown
					po_v[i].orientation.w = 0.0d;//unknown
				}
				//check if already in the pose vector
				for (int j = 0; j < green_groundbots_world_loc.poses.size(); j++)
				{
					//near a groundbot already in the array? if so update position
					//if timestamp is under 10 secs old, orientation data is kept
					if (	abs((green_groundbots_world_loc.poses[j].position.y - po_v[i].position.y)) < 1.0d
							&& abs((green_groundbots_world_loc.poses[j].position.x - po_v[i].position.x)) < 1.0d
							&& !(-0.0001d < gRotTimeStamps[j] < 0.0001d)
							&& abs(gRotTimeStamps[j] - ros::Time::now().toSec()) < 10.0d)
					{
						green_groundbots_world_loc.poses[j].position = po_v[i].position;
						
						if (angleAdded) //if orientation measurement taken, orientation updated
						{
							gRotTimeStamps[j] = ros::Time::now().toSec();
							green_groundbots_world_loc.poses[j].orientation = po_v[i].orientation;
							angleAdded = false;
						}
						
						in_there = true;
					}
					//else, forget angle:
					else if(abs((green_groundbots_world_loc.poses[j].position.y - po_v[i].position.y)) < 1.0d
							&& abs((green_groundbots_world_loc.poses[j].position.x - po_v[i].position.x)) < 1.0d)
					{
						gRotTimeStamps[j] = 0.0d;
						green_groundbots_world_loc.poses[j] = po_v[i];
						in_there = true;
					}
				}
				//dont push to array if already in there
				if (in_there)
				{
					in_there = false;
					continue;
				}
				else
				{
					if (angleAdded)
					{
						gRotTimeStamps.push_back(ros::Time::now().toSec());
						angleAdded = false;
					}
					else
						gRotTimeStamps.push_back(0.0d);
						
					green_groundbots_world_loc.poses.push_back(po_v[i]);
					gTimeStamps.push_back(ros::Time::now().toSec());
				}		
			}
		}
		checkTimes(color);
		gpp.publish(green_groundbots_world_loc);
		
	}//end ifGreen
	else//(if red)
	{
		for (int i = 0; i < po_v.size(); i++)
		{
			if (po_v[i].position.x < 20.0d && po_v[i].position.y < 20.0d) // < 20 m away
			{
				//if directly below us, add angle
				if ( downcam
					&& (po_v[i].position.x < 0.5d && po_v[i].position.y < 0.5d)  // < 0.5 meter away
					&& ( (ros::Time::now().toSec() - rAngleTime) < 1.0d ) ) // < 1 sec
				{
					angleAdded = true;
					//qx is always 0, so storing data in it:
					po_v[i].orientation.x = 0.0d; 
					po_v[i].orientation.y = 0.0d; //always 0
					po_v[i].orientation.z = sin(gAngle/2.0f);
					po_v[i].orientation.w = cos(gAngle/2.0f);
				}
				else
				{
					po_v[i].orientation.x = 0.0d;//always 0
					po_v[i].orientation.y = 0.0d; //always 0
					po_v[i].orientation.z = 0.0d;//unknown
					po_v[i].orientation.w = 0.0d;//unknown
				}
				//near a groundbot already in the array? if so update posititon
				for (int j = 0; j < red_groundbots_world_loc.poses.size(); j++)
				{
					if (	abs((red_groundbots_world_loc.poses[j].position.y - po_v[i].position.y)) < 1.0d 
							&& abs((red_groundbots_world_loc.poses[j].position.x - po_v[i].position.x)) < 1.0d 
							&& !(-0.0001d < red_groundbots_world_loc.poses[j].orientation.x < 0.0001d)
							&& abs(red_groundbots_world_loc.poses[j].orientation.x - ros::Time::now().toSec()) < 10.0d )
					{
						red_groundbots_world_loc.poses[j].position = po_v[i].position;
						
						if (angleAdded)
						{
							rRotTimeStamps[j] = ros::Time::now().toSec();
							red_groundbots_world_loc.poses[j].orientation = po_v[i].orientation;
							angleAdded = false;
						}
						
						in_there = true;
						
					}
					else if (	abs((red_groundbots_world_loc.poses[j].position.y - po_v[i].position.y)) < 1.0d 
								&& abs((red_groundbots_world_loc.poses[j].position.x - po_v[i].position.x)) < 1.0d ) //near a groundbot already in the array? if so update position
					{
						rRotTimeStamps[j] = 0.0d;
						red_groundbots_world_loc.poses[j] = po_v[i];
						in_there = true;
					}
				}
				//dont push to array if already in there
				if (in_there)
				{
					in_there = false;
					continue;
				}
				else
				{
					if (angleAdded)
					{
						gRotTimeStamps.push_back(ros::Time::now().toSec());
						angleAdded = false;
					}
					else
						gRotTimeStamps.push_back(0.0d);
						
					red_groundbots_world_loc.poses.push_back(po_v[i]);
					rTimeStamps.push_back(ros::Time::now().toSec());
				}
			}
		}
		checkTimes(color);
		rpp.publish(red_groundbots_world_loc);
	}//end ifRed
}
void plate_localizer::point_callback(const std_msgs::Int32MultiArray& msg, char camID, char color)
{
	vector<geometry_msgs::Pose> po_v;
	vector<cv::Point> pt_v;
	cv::Point p_;
	bool downcam = false;
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
		case '0':
		for (int i = 0; i < pt_v.size(); i++)
		{
			downcam = true;
			po_v.push_back( c0.getPlateWorldLocation(uavPose_, pt_v[i]) );
		}
	}
	plate_localizer::merge_positions_location(po_v, color, downcam);
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
void plate_localizer::cam_0r_callback(const std_msgs::Int32MultiArray& msg)
{
	plate_localizer::point_callback(msg, '0', 'r');
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
void plate_localizer::cam_0g_callback(const std_msgs::Int32MultiArray& msg)
{
	plate_localizer::point_callback(msg, '0', 'g');
}

int main(int argc, char** argv)
{	
	waitKey(5000);
   ros::init(argc, argv, "/observer/plate_localizer");
   plate_localizer l1;
   ros::spin(); //use 1 thread
}

		
