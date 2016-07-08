#include "loc_object.h"
#include <iostream>

/// Setup the ros handling
sensor_processor::sensor_processor()
{
	pose_pub = n.advertise<geometry_msgs::PoseStamped>("current_pose", 1);
	
	//clear vital variables:
	orientation_fused.v.x = 0;
	orientation_fused.v.y = 0;
	orientation_fused.v.z = 0;
	orientation_fused.w = 0;
	pos_reset = true;
	
	// resets
	vel_reset = true;
	pos_reset = true;
	
	//subs
	sub_zero_position = n.subscribe("/ground_station/zero_position", 1, &sensor_processor::zero_position_callback, this);
	sub_guidance_velocity = n.subscribe("/guidance/velocity", 1, &sensor_processor::guidance_vel_callback, this);
	//sub_guidance_imu = n.subscribe("/guidance/imu", 1, &sensor_processor::guidance_imu_callback, this);
	sub_guidance_sonar = n.subscribe("/guidance/ultrasound", 1, &sensor_processor::guidance_sonar_callback, this);
	sub_pixhawk_imu = n.subscribe("/mavros/imu/data", 1, &sensor_processor::pixhawk_imu_callback, this);
	sub_hokuyo = n.subscribe("/scan2", 1, &sensor_processor::hokuyo_sub, this);
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
	pos_fused_msg.header.stamp = current_time;
	pos_fused_msg.pose.position.x = pos_fused.x;
	pos_fused_msg.pose.position.y = pos_fused.y;
	pos_fused_msg.pose.position.z = pos_fused.z;
	pos_fused_msg.pose.orientation.x = orientation_fused.v.x;
	pos_fused_msg.pose.orientation.y = orientation_fused.v.y;
	pos_fused_msg.pose.orientation.z = orientation_fused.v.z;
	pos_fused_msg.pose.orientation.w = orientation_fused.w;
	pose_pub.publish(pos_fused_msg);
}

/// Global referace
/// Fusion with integrated IMUs
/// integrate to position estimate
void sensor_processor::guidance_vel_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
	double low_val_plus = 0.1;
	double low_val_minus = -0.1;
	double vel_x = -(msg->vector.x);//guidance x is opposite hank x
	if (vel_x > low_val_plus)
	{
		vel_x = low_val_plus;
	}
	if (vel_x < low_val_minus)
	{
		vel_x = low_val_minus;
	}
	double vel_y = msg->vector.y;
	if (vel_y > low_val_plus)
	{
		vel_y = low_val_plus;
	}
	if (vel_y < low_val_minus)
	{
		vel_y = low_val_minus;
	}
	double vel_z = -(msg->vector.z);
	if (vel_z > low_val_plus)
	{
		vel_z = low_val_plus;
	}
	if (vel_z < low_val_minus)
	{
		vel_z = low_val_minus;
	}
	
	std::cout << "Vel X: 0" << vel_x << std::endl;
	std::cout << "Vel Y: 0" << vel_y << std::endl;
	std::cout << "Vel Z: 0" << vel_z << std::endl;
	
	// global velocity
	Vector vel_G(vel_x, vel_y, vel_z);
	vel_G = orientation_fused*vel_G;
	
	// sensor fuse velocity
	vel_G = (vel_G*GUIDANCE_VEL_WEIGHT + vel_pix_G*(1-GUIDANCE_VEL_WEIGHT));
	
	// integrate velocity
	ros::Time current_time = msg->header.stamp;
	if (pos_reset)
	{
		pos_fused.x = 0;
		pos_fused.y = 0;
		pos_fused.z = 0;
		pos_reset = false;
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
    
    //could add cout for debugging if needed:
	/*cout << "goal: " << nav_path.poses[current_goal] << endl;
    cout << "est: " << pos_est << endl;
    cout << "x-corr: " << x_out << ", y-corr: " << y_out << endl;*/
    
    merge_and_publish(current_time);
}

/// Integrate acceleration
/// passin fusion orientation
/// calculate Global velocity
/*
void sensor_processor::guidance_imu_callback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	double acc_x = msg->transform.translation.x;
	double acc_y = msg->transform.translation.y;
	double acc_z = msg->transform.translation.z;
	
	ros::Time current_time = msg->header.stamp;
	
	if (vel_reset)
	{
		vel_guid_imu.x = 0;
		vel_guid_imu.y = 0;
		vel_guid_imu.z = 0;
		vel_reset = false;
	}
	else
	{
		double sec_diff = current_time.toSec() - pre_acc_guid.header.stamp.toSec();
		vel_guid_imu.x += (acc_x + pre_acc_guid.vector.x)/2*sec_diff;
		vel_guid_imu.y += (acc_y + pre_acc_guid.vector.y)/2*sec_diff;
		vel_guid_imu.z += (acc_z + pre_acc_guid.vector.z)/2*sec_diff;
	}
	
	pre_acc_guid.header.stamp = current_time;
	pre_acc_guid.vector = msg->transform.translation;
	
	//orientation
	//*********    without the sensor fusion of pixhawks IMUs    **********
	orientation_fused.v.x = -(msg->transform.rotation.x);
	orientation_fused.v.y = msg->transform.rotation.y;
	orientation_fused.v.z = -(msg->transform.rotation.z);
	orientation_fused.w   = msg->transform.rotation.w;
	
	//global vel est
	vel_guid_G = orientation_fused*vel_guid_imu;
	
	// sensor fuse vel, simple average
	vel_guid_G = (vel_pix_G+vel_guid_G)/2;
}
*/

/// Pass altitude value to Hokuyo
void sensor_processor::guidance_sonar_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	// Not sure how to get sonar Altitude yet
	/// min value 0.07
	
}

/// Integrate to velocity
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
}

/// fuse data with sonar
void sensor_processor::hokuyo_sub(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	double avg = 0;
	std::size_t range_size = sizeof(msg->ranges)/4;
    for(std::size_t i = 0; i < range_size; i++) {
        if(msg->ranges[i] < 4) { // change from 4 to max, make case for min
            avg += (double)(msg->ranges[i]/range_size);
        }
        else {
			/// ************************** Test min/max ranges *****************************
            avg += (double)(4/(sizeof(4)));
        }
    }
    // delay sensor fusion for altitude
    fused_altitude = avg;
}

void sensor_processor::zero_position_callback(const std_msgs::Bool msg)
{
	if (msg.data == true)
	{
		pos_reset = true;
	}
}
