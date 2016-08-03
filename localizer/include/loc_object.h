/**
 * 
 * 
 ****/

#ifndef LOCALIZER_POSE_DATA_H
#define LOCALIZER_POSE_DATA_H

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include "quaternion.h"
#include "vector.h"


class sensor_processor
{
private:
	/// Weight of the velocity guidance data (0-1)
	float GUIDANCE_VEL_WEIGHT;
	float OFFSET_X; 
	float OFFSET_Y; 
	
	ros::NodeHandle n;
	ros::Publisher pose_pub; // in global reference (meters)
	ros::Subscriber sub_guidance_velocity, sub_guidance_sonar, sub_pixhawk_imu, sub_hokuyo, sub_gridflow_position, sub_zero_position; //sub_guidance_imu
	
	/// sensor fused data, global values (world oriented)
	// message for Position and Orientation
	geometry_msgs::PoseStamped pos_fused_msg;
	geometry_msgs::Point grid_flow_point;
	Vector pos_fused;
	geometry_msgs::Vector3Stamped pre_pos_fused;
	
	Quaternion orientation_fused;
	
	/// passed values through callbacks
	Vector vel_pix_imu;
	Vector vel_guid_imu;
	Quaternion orient_pix_imu;
	
	/// Prevalues for integration;
	geometry_msgs::Vector3Stamped pre_acc_guid;
	geometry_msgs::Vector3Stamped pre_acc_pix;
	
	Vector vel_pix_G;
	Vector vel_guid_G;
	
	double sonar_altitude;
	double fused_altitude;
	double ALT_OFFSET;
	
	bool pos_reset;
	bool vel_reset;
	
	double* vel_calib;
	bool first_calib;
	
public:
	///Final function merging most recent data and publishing
	void merge_and_publish(ros::Time current_time);

	/// Setup the ros handling
	sensor_processor();
	~sensor_processor();
	
	/// main loop
	void system_loop();
	
	/// Global referace
	/// Fusion with integrated IMUs
	/// integrate to position estimate
	void guidance_vel_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
	
	/// Integrate acceleration
	/// passin fusion orientation
	/// calculate Global velocity
	//void guidance_imu_callback(const geometry_msgs::TransformStamped::ConstPtr& msg);
	
	/// Pass altitude value to Hokuyo
	void guidance_sonar_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
	
	/// Integrate to velocity
	/// Set to global referance
	/// Sent value to guidance IMU
	void pixhawk_imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
	
	/// fuse data with sonar
	void hokuyo_sub(const sensor_msgs::LaserScan::ConstPtr& msg);
	
	///subscribe to reset topic and reset position if true	
	void zero_position_callback(const std_msgs::Bool msg);
	
	
	///subscribe to grid_flow position
	void gridflow_cb(const geometry_msgs::Point& msg);
};

#endif
