/**
 * 
 * 
 ****/

#ifndef LOCALIZER_POSE_DATA_H
#define LOCALIZER_POSE_DATA_H

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>


class sensor_processor
{
private:	
	ros::NodeHandle n;
	ros::Publisher pos_pub; // in global reference (meters)
	ros::Subscriber sub_guidance_velocity, sub_guidance_imu, sub_guidance_sonar, sub_pixhawk_imu, sub_hokuyo;
	
	/// sensor fused data
	geometry_msgs::Point pose_fused;
	geometry_msgs::Quaternion orientation_fused;
	
	/// passed values through callbacks
	geometry_msgs::Point vel_pixhawk_imu;
	geometry_msgs::Point vel_guidance_imu;
	geometry_msgs::Quaternion orient_pixhawk_imu;
	
	/// Prevalues for integration;
	geometry_msgs::Vector3Stamped prev_guidance_imu_acc;
	geometry_msgs::Vector3Stamped prev_pixhawk_imu_acc;
	
	geometry_msgs::Vector3Stamped guidance_vel_estimation;
	geometry_msgs::Vector3Stamped pixhawk_vel_estimation;
	
	bool vel_reset_zero;
	
public:
	/// Setup the ros handling
	sensor_processor();
	
	/// Main loop
	void start_sensor_sub();
	
	/// Global referace
	/// Fusion with integrated IMUs
	/// integrate to position estimate
	void guidance_vel_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
	
	/// Integrate acceleration
	/// passin fusion orientation
	/// calculate Global velocity
	void guidance_imu_callback(const geometry_msgs::TransformStamped& msg);
	
	/// Pass altitude value to Hokuyo
	void guidance_sonar_callback(const sensor_msgs::LaserScan& msg);
	
	/// Integrate to velocity
	/// Set to global referance
	/// Sent value to guidance IMU
	void pixhawk_imu_callback(const sensor_msgs::Imu& msg);
	
	/// fuse data with sonar
	void hokuyo_sub(const sensor_msgs::LaserScan& msg);
	
};

#endif
