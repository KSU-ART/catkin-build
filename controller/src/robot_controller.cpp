// More documentation are found in the *.h corresponding files in the /include file

#include "robot_controller.h"


// Ros Subscribers
void robot_controller::ai_reset_cb(const std_msgs::Bool& msg){
	if(msg.data){
		pids.reset_all();
	}
}

void robot_controller::current_altitude_cb(const std_msgs::Float32& msg){
	current_altitude = msg.data;
}

void robot_controller::target_altitude_cb(const std_msgs::Float32& msg){
	std::cout << "set Altitude " << msg.data << std::endl; 
	pids.getThrottlePID().targetSetpoint(msg.data);
}

void robot_controller::yolo_x_cb(const std_msgs::Int16& msg){
	// update mode
	state_mode = Yolo;
	current_yolo_yaw = msg.data - camera_width/2;
}

void robot_controller::yolo_y_cb(const std_msgs::Int16& msg){
	// update mode
	state_mode = Yolo;
	current_yolo_pitch = msg.data - camera_height;
}

void robot_controller::delta_down_cam_x_cb(const std_msgs::Int16& msg){
	// update mode
	state_mode = DownCam;
	current_down_cam_roll = msg.data;
}

void robot_controller::delta_down_cam_y_cb(const std_msgs::Int16& msg){
	//update mode
	state_mode = DownCam;
	current_down_cam_pitch = msg.data;
}

void robot_controller::obstacle_pitch_cb(const std_msgs::Float32& msg){
	// update mode
	state_mode = Obstacle;
	current_obstacle_pitch = msg.data;
}

void robot_controller::obstacle_roll_cb(const std_msgs::Float32& msg){
	// update mode
	state_mode = Obstacle;
	current_obstacle_roll = msg.data;
}

void robot_controller::random_traversal_cb(const std_msgs::Float32& msg){
	// update mode
	state_mode = RandomTraversal;
	current_traversal_yaw = msg.data;
}

void robot_controller::edge_detect_x_cb(const std_msgs::Float32& msg){
	// update mode
	state_mode = EdgeDetect;
	current_edge_detect_roll = msg.data;
}

void robot_controller::edge_detect_y_cb(const std_msgs::Float32& msg){
	// update mode
	state_mode = EdgeDetect;
	current_edge_detect_pitch = msg.data;
}

//******************** applying pid controls **************************
double robot_controller::get_throttle_control(){
	switch(state_mode){
	case DownCam:
	case Obstacle:
	case Yolo:
	case EdgeDetect:
	case RandomTraversal:
		// std::cout << "*******current altitude: " << current_altitude << std::endl;
		return pids.getThrottlePID().calc(current_altitude);
	default:
		return 0;
	}
}

double robot_controller::get_roll_control(){
	switch(state_mode){
	case DownCam:
		return pids.getRollPID().calc(current_down_cam_roll);
	case Obstacle:
		return pids.getRollPID().calc(-current_obstacle_roll);
	case EdgeDetect:
		return pids.getRollPID().calc(current_edge_detect_roll);
	default:
		return 0;
	}
}

double robot_controller::get_pitch_control(){
	switch(state_mode){
	case DownCam:
		if(DEBUG){
			std::cout << "*******DownCam_pitch: " << current_down_cam_pitch << std::endl;
		}
		return pids.getPitchPID().calc(current_down_cam_pitch);
	case Obstacle:
		if(DEBUG){
			std::cout << "*******Obstacle_pitch: " << current_obstacle_pitch << std::endl;
		}
		return pids.getPitchPID().calc(-current_obstacle_pitch);
	case Yolo:
		if(DEBUG){
			std::cout << "*******Yolo_pitch: " << current_yolo_pitch << std::endl;
		}
		return pids.getPitchPID().calc(current_yolo_pitch);
	case EdgeDetect:
		if(DEBUG){
			std::cout << "*******Edge_pitch: " << current_edge_detect_pitch << std::endl;
		}
		return pids.getPitchPID().calc(current_edge_detect_pitch);
	default:
		return 0;
	}
}

double robot_controller::get_yaw_control(){
	switch(state_mode){
	case Yolo:
		return pids.getYawPID().calc(current_yolo_yaw);
	case RandomTraversal:
		return pids.getYawPID().calc(current_traversal_yaw);
	default:
		return 0;
	}
}

void robot_controller::init_loop(){
	// initialize target pids
	pids.initialize_zero_target();
	state_mode = Yolo;
}

void robot_controller::nav_loop(){
	// set states from state machine to the pid handler
	if(state_mode != pre_state_mode){
		switch(state_mode){
		case DownCam:
			// std::cout << "*************************************************" << std::endl;
			pids.set_pitch_mode("DownCam");
			pids.set_roll_mode("DownCam");
			break;
		case Obstacle:
			pids.set_pitch_mode("Obstacle");
			pids.set_roll_mode("Obstacle");
			break;
		case Yolo:
			pids.set_pitch_mode("Yolo");
			pids.set_yaw_mode("Yolo");
			break;
		case EdgeDetect:
			pids.set_pitch_mode("EdgeDetect");
			pids.set_roll_mode("EdgeDetect");
			break;
		case RandomTraversal:
			pids.set_yaw_mode("RandomTraversal");
			break;
		default:
			break;
		}
		pre_state_mode = state_mode;
	}

	// update the control values
	roll = MID_PWM - get_roll_control() + ROLL_TRIM; // Roll value for Left is negative
	pitch = MID_PWM - get_pitch_control() - PITCH_TRIM; // Pitch value for Forward is negative
	yaw = MID_PWM - get_yaw_control() + YAW_TRIM;
	throttle = MID_PWM + get_throttle_control();

	mav.update_loop(roll, pitch, yaw, throttle);
	
	ros::spinOnce();
	fcuCommRate.sleep();
}

void robot_controller::end_loop(){
	// do shutdown sequence
	mav.disable_rc_overide();
}
