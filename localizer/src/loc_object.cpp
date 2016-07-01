#include "loc_object.h"

/// Setup the ros handling
sensor_processor::sensor_processor()
{
	pose_pub = n.advertise<geometry_msgs::PoseStamped>("localizer/pose", 1);
	
	//clear vital variables:
	orientation_fused.v.x = 0;
	orientation_fused.v.y = 0;
	orientation_fused.v.z = 0;
	orientation_fused.w = 0;
	pos_reset = true;
	
	// integration prevalues
	
	vel_reset_zero = false;
	pos_reset = true;
	
	//subs
	sub_guidance_velocity = n.subscribe("/guidance/velocity", 1, &sensor_processor::guidance_vel_callback, this);
	sub_guidance_imu = n.subscribe("/guidance/imu", 1, &sensor_processor::guidance_imu_callback, this);
	sub_guidance_sonar = n.subscribe("/guidance/ultrasound", 1, &sensor_processor::guidance_sonar_callback, this);
	sub_pixhawk_imu = n.subscribe("/mavros/imu/data", 1, &sensor_processor::pixhawk_imu_callback, this);
	sub_hokuyo = n.subscribe("/scan2", 1, &sensor_processor::hokuyo_sub, this);
}

/// Main loop
void sensor_processor::start_sensor_sub()
{
	
}

/// Global referace
/// Fusion with integrated IMUs
/// integrate to position estimate
void sensor_processor::guidance_vel_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
	double vel_x = -(msg->vector.x); //guidance x is opposite hank x
	double vel_y = msg->vector.y;
	//double vel_z = -(msg->vector.z); //could use this later

	ros::Time current_time = msg->header.stamp;
	if (pos_reset)
	{
		pose_fused_msg.header.stamp = current_time;
		pose_fused_msg.pose.position.x = 0;
		pose_fused_msg.pose.position.y = 0;
		pose_fused_msg.pose.position.z = 0;
		pos_reset = false;
	}
		
	pose_fused_msg.pose.position.x += (vel_x + guidance_vel_estimation.vector.x)/2*(current_time.toSec() - pose_fused_msg.header.stamp.toSec());
	pose_fused_msg.pose.position.y += (vel_y + guidance_vel_estimation.vector.y)/2*(current_time.toSec() - pose_fused_msg.header.stamp.toSec());
	pose_fused_msg.pose.position.z = fused_altitude;
	pose_fused_msg.pose.orientation = orientation_fused_msg;
	pose_fused_msg.header.seq++;
	pose_fused_msg.header.stamp = current_time;
	pose_pub.publish(pose_fused_msg);
    
    //could add cout for debugging if needed:
   /*cout << "goal: " << nav_path.poses[current_goal] << endl;
    cout << "est: " << pos_est << endl;
    cout << "x-corr: " << x_out << ", y-corr: " << y_out << endl;*/
    
    guidance_vel_estimation = *msg;
}

/// Integrate acceleration
/// passin fusion orientation
/// calculate Global velocity
void sensor_processor::guidance_imu_callback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	double acc_x = msg->transform.translation.x;
	double acc_y = msg->transform.translation.y;
	
	ros::Time current_time = msg->header.stamp;
	
	if (vel_reset_zero)
	{
		guidance_vel_estimation.vector.x = 0;
		guidance_vel_estimation.vector.y = 0;
	}
	else
	{
		double sec_diff = current_time.toSec() - guidance_vel_estimation.header.stamp.toSec();
		guidance_vel_estimation.vector.x += (acc_x + prev_guidance_imu_acc.x)/2*sec_diff;
		guidance_vel_estimation.vector.y += (acc_y + prev_guidance_imu_acc.y)/2*sec_diff;
	}
	
	guidance_vel_estimation.header.stamp = current_time;
	prev_guidance_imu_acc = msg->transform.translation;
	
	//orientation
	orientation_fused.v.x = msg->transform.rotation.x; //without the sensor fusion of pixhawks IMUs
	orientation_fused.v.y = msg->transform.rotation.y;
	orientation_fused.v.z = msg->transform.rotation.z;
	orientation_fused.w   = msg->transform.rotation.w;
	
	//global vel est
	Vector vel_est(guidance_vel_estimation.vector.x, guidance_vel_estimation.vector.y, 0);
	Quaternion o;
	Vector global_vel_est = vel_est;
	//Vector global_vel_est = orientation_fused*vel_est;
	global_guidance_vel_estimation.vector.x = global_vel_est.x;
	global_guidance_vel_estimation.vector.y = global_vel_est.y;
}

/// Pass altitude value to Hokuyo
void sensor_processor::guidance_sonar_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	
}

/// Integrate to velocity
/// Set to global referance
/// Sent value to guidance IMU
void sensor_processor::pixhawk_imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	double acc_x = msg->linear_acceleration.x;
	double acc_y = msg->linear_acceleration.y;
	
	ros::Time current_time = msg->header.stamp;
	
	if (vel_reset_zero)
	{
		pixhawk_vel_estimation.vector.x = 0;
		pixhawk_vel_estimation.vector.y = 0;
	}
	else
	{
		double sec_diff = current_time.toSec() - pixhawk_vel_estimation.header.stamp.toSec();
		pixhawk_vel_estimation.vector.x += (acc_x + prev_pixhawk_imu_acc.x)/2*sec_diff;
		pixhawk_vel_estimation.vector.y += (acc_y + prev_pixhawk_imu_acc.y)/2*sec_diff;
	}
	
	pixhawk_vel_estimation.header.stamp = current_time;
	prev_pixhawk_imu_acc = msg->linear_acceleration;
	
	//orientation
	//orient_pixhawk_imu = msg->orientation;
	
	
	//global vel est
}

/// fuse data with sonar
void sensor_processor::hokuyo_sub(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	double avg = 0;
    for(int i = 0; i < sizeof(msg->ranges)/4; i++) {
        if(msg->ranges[i] < 4) {
            avg += (double)(msg->ranges[i]/(sizeof(msg->ranges)/4));
        }
        else {
            avg += (double)(4/(sizeof(4)));
        }
    }
    // delay sensor fusion for altitude
    fused_altitude = avg;
}


