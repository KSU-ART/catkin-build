#include "loc_object.h"
#include <iostream>
#include <algorithm>

/// Setup the ros handling
sensor_processor::sensor_processor()
{
	pose_pub = n.advertise<geometry_msgs::PoseStamped>("/localizer/current_pose", 1);
	
	//init important vars
	ALT_OFFSET = 0.0f;
	GUIDANCE_VEL_WEIGHT = 1;
	OFFSET_X = -1; 
	OFFSET_Y = 10; 
	
	//clear vital variables:
	orientation_fused.v.x = 0;
	orientation_fused.v.y = 0;
	orientation_fused.v.z = 0;
	orientation_fused.w = 0;
	grid_flow_position.x = 0;
	grid_flow_position.y = 0;
	grid_flow_position.z = 0;
	
	vel_calib = new double[3];
	std::fill_n(vel_calib, 3, 0);
	first_calib = true;
	
	// resets
	vel_reset = true;
	grid_pos_zero = false;
	
	//subs
	sub_zero_position = n.subscribe("/ground_station/zero_position", 1, &sensor_processor::zero_position_callback, this);
	sub_guidance_velocity = n.subscribe("/guidance/velocity", 1, &sensor_processor::guidance_vel_callback, this);
	sub_gridflow_position = n.subscribe("/observer/grid_pos", 1, &sensor_processor::gridflow_cb, this);
	sub_guidance_sonar = n.subscribe("/guidance/ultrasound", 1, &sensor_processor::guidance_sonar_callback, this);
	sub_pixhawk_imu = n.subscribe("/mavros/imu/data", 1, &sensor_processor::pixhawk_imu_callback, this);
	sub_hokuyo = n.subscribe("/scan2", 1, &sensor_processor::hokuyo_sub, this);
	
	
	startTime = ros::Time::now().toSec();
}

sensor_processor::~sensor_processor()
{
	delete vel_calib;
}

/// main loop
void sensor_processor::system_loop()
{
	ros::Rate loop_rate(120);
	while (ros::ok())
	{
		
		ros::spinOnce();
		loop_rate.sleep();
	}
}

///merge and publish most recent data
void sensor_processor::merge_and_publish(ros::Time current_time)
{
	pos_fused_msg.header.seq++;
	pos_fused_msg.header.stamp = ros::Time::now();
	
///do not use guidance
/*	if( ros::Time::now().toSec() - startTime <  6.00)
=======
	if( ros::Time::now().toSec() - startTime <  6.00)
>>>>>>> f8a486467cd392ccb3654c62785ca9e2999e52c9
	{
		pos_fused_msg.pose.position.x = pos_fused.x;
		pos_fused_msg.pose.position.y = pos_fused.y;
	}
	else
<<<<<<< HEAD
	{*/
		pos_fused_msg.pose.position.x = grid_flow_position.x;
		pos_fused_msg.pose.position.y = grid_flow_position.y;
	//}
	
	pos_fused_msg.pose.position.z = fused_altitude;
	pos_fused_msg.pose.orientation.x = orientation_fused.v.x;
	pos_fused_msg.pose.orientation.y = orientation_fused.v.y;
	pos_fused_msg.pose.orientation.z = orientation_fused.v.z;
	pos_fused_msg.pose.orientation.w = orientation_fused.w;
	pose_pub.publish(pos_fused_msg);
}

void sensor_processor::gridflow_cb(const geometry_msgs::PointStamped& msg)
{
	if (grid_pos_zero)
	{
		OFFSET_X =  -(msg.point.x);
		OFFSET_Y = -(msg.point.y);
		grid_pos_zero = false;
	}
	double dt = msg.header.stamp.toSec() - prev_grid_position.header.stamp.toSec();
	double dvx = msg.point.x - prev_grid_position.point.x;
	double dvy = msg.point.y - prev_grid_position.point.x;
	double v_x = dvx/dt;
	double v_y = dvy/dt;
	ROS_INFO("vel_x: %f", v_x);
	ROS_INFO("vel_y: %f", v_y);
	
	grid_flow_position.x = msg.point.x + OFFSET_X;
	grid_flow_position.y = msg.point.y + OFFSET_Y;	
	prev_grid_position = msg;

	//msg.header.stamp.toSec()
}

/// Global referace
/// Fusion with integrated IMUs
/// integrate to position estimate
void sensor_processor::guidance_vel_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
	double low_val_plus = 0.1;
	double low_val_minus = -0.1;
	
	double vel_x = -vel_calib[0] - (msg->vector.x);//guidance x is opposite hank x
	double vel_y = -vel_calib[1] + msg->vector.y;
	double vel_z = -vel_calib[2] - (msg->vector.z);//guidance z is opposite hank z

	
	//First iteration Calibration (zero out the values at the beginning)
	if (first_calib)
	{
		first_calib = false;
		vel_calib[0] = vel_x;
		vel_calib[1] = vel_y;
		vel_calib[2] = vel_z;
	}
	
	if (vel_x > low_val_plus)
	{
		vel_x = low_val_plus;
	}
	if (vel_x < low_val_minus)
	{
		vel_x = low_val_minus;
	}
	if (vel_y > low_val_plus)
	{
		vel_y = low_val_plus;
	}
	if (vel_y < low_val_minus)
	{
		vel_y = low_val_minus;
	}
	if (vel_z > low_val_plus)
	{
		vel_z = low_val_plus;
	}
	if (vel_z < low_val_minus)
	{
		vel_z = low_val_minus;
	}
	
	
	// global velocity
	Vector vel_G(vel_x, vel_y, vel_z);
	vel_G = orientation_fused*vel_G;
	
	// sensor fuse velocity
	vel_G = (vel_G*GUIDANCE_VEL_WEIGHT + vel_pix_G*(1-GUIDANCE_VEL_WEIGHT));
	
	// integrate velocity
	ros::Time current_time = msg->header.stamp;
	if (guidance_pos_reset)
	{
		pos_fused.x = 0;
		pos_fused.y = 0;
		pos_fused.z = 0;
		guidance_pos_reset = false;
	}
	else
	{
		double sec_diff = current_time.toSec() - pre_pos_fused.header.stamp.toSec();
		pos_fused.x += (vel_G.x + pre_pos_fused.vector.x)/2*sec_diff;
		pos_fused.y += (vel_G.y + pre_pos_fused.vector.y)/2*sec_diff;
		pos_fused.z += (vel_G.z + pre_pos_fused.vector.z)/2*sec_diff;
	}
    
	pre_pos_fused.header.stamp = current_time;
	pre_pos_fused.vector = msg->vector;
    
    merge_and_publish(current_time);
}

/// Pass altitude value to Hokuyo
void sensor_processor::guidance_sonar_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	// Not sure how to get sonar Altitude yet
	/// min value 0.07
	
}

/// Set to global referance
/// Sent value to guidance IMU
void sensor_processor::pixhawk_imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	double acc_x = msg->linear_acceleration.x;
	double acc_y = msg->linear_acceleration.y;
	double acc_z = msg->linear_acceleration.z;
	
	ros::Time current_time = msg->header.stamp;
	
	if (vel_reset)
	{
		vel_pix_imu.x = 0;
		vel_pix_imu.y = 0;
		vel_pix_imu.z = 0;
		vel_reset = false;
	}
	else
	{
		double sec_diff = current_time.toSec() - pre_acc_pix.header.stamp.toSec();
		vel_pix_imu.x += (acc_x + pre_acc_pix.vector.x)/2*sec_diff;
		vel_pix_imu.y += (acc_y + pre_acc_pix.vector.y)/2*sec_diff;
		vel_pix_imu.z += (acc_z + pre_acc_pix.vector.z)/2*sec_diff;
	}
	
	pre_acc_pix.header.stamp = current_time;
	pre_acc_pix.vector = msg->linear_acceleration;
	
	//orientation
	orientation_fused.v.x = msg->orientation.x;
	orientation_fused.v.y = msg->orientation.y;
	orientation_fused.v.z = msg->orientation.z;
	orientation_fused.w = msg->orientation.w;
	
	//global vel est
	vel_pix_G = orientation_fused*vel_pix_imu;
	
	merge_and_publish(current_time);
}

///get altitude from lidar
void sensor_processor::hokuyo_sub(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	double avg = 0;
	std::size_t range_size = sizeof(msg->ranges)/4;
    for(std::size_t i = 0; i < range_size; i++) {
        if(msg->ranges[i] < 4) {
            avg += (double)(msg->ranges[i]/range_size);
        }
        else {
            avg += (double)(4/(sizeof(4)));
        }
    }
    fused_altitude = avg + ALT_OFFSET;
}

void sensor_processor::zero_position_callback(const std_msgs::Bool msg)
{
	if (msg.data == true)
	{
		grid_pos_zero = true;
	}
}
