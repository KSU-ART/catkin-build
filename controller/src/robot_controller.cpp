/****************************************************************************************************
 * This is the class using the pid controller to 
 * generate output to the pixhawk.
 * 
 * CLASS INVARIANCE:
 * 		X+ => Forward
 * 		Y+ => Left
 * 		Z+ => Up
 * 
 * MSG GUIDE:
 * Subscribed Topic:	Data Type					Usage
 * "curent_pose"		geometry_msgs::PoseStamped	current location
 * "manOverrideMsg"		std_msgs::Bool				true = override, false = disabled
 * "EMERGENCY_LAND"		std_msgs::Bool				true = emergencyland, false = normal
 * "/ai_nav/setpoint"	geometry_msgs::Point		setpoint (desired location)
 * "/ai_nav/modeMsg"	std_msgs::Int8				0 = altitude hold, 1 = stabilize, 2 = land;
 * "/ai_nav/retractMsg"	std_msgs::Bool				true = retracts down, false = up;
 * "/ai_nav/pid_XY"		std_msgs::Int32MultiArray	{p, i, d, min, max} Creates new pid variables
 * "/ai_nav/pid_z"		std_msgs::Int32MultiArray	{p, i, d, min, max} Creates new pid variables
 * 
 * MSG CONSTANCES:
 * MSG.CHAN_RELEASE = 0
 * MSG.CHAN_NOCHANGE = 65535
 * 
 * PRIORITY OF CONTROL:
 * physical mannual-override switch overrides software switch
 * MANUAL_OVERRIDE overrides emergency_land
 * emergency_land overrides ai
 * 
 * 
 * Mannual Override Switch: 
 * channel 9 (8 in {0-8}) HIGH_PWM is land
 * 
 **************************************************************************************************/

#include "robot_controller.h"


class mavros_handler{
private:
	ros::NodeHandle n;
	ros::Publisher rc_pub, ai_reset_pub;

	mavros_msgs::OverrideRCIn RC_MSG;

	short roll, pitch, yaw, throttle = MID_PWM;

	enum flight_mode_enum{
		ai,
		land,
		manual
	};

	flight_mode_enum flight_mode = ai;

	char landChar;

	bool LAND_TOGGLE, DEBUG;

public:
	mavros_handler(){
		LAND_TOGGLE = false;
		DEBUG = true;

		// just anything not space
		landChar = 'f';

		rc_pub = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);
		ai_reset_pub = n.advertise<std_msgs::Bool>("/IARC/ai_reset", 1);

		n.subscribe("/mavros/rc/in", 1, &mavros_handler::RCIn_callback, this);
	}

	void release_msg_channels(){
		RC_MSG.channels[MODE_CHANNEL] = RC_MSG.CHAN_RELEASE;

		RC_MSG.channels[ROLL_CHANNEL] = RC_MSG.CHAN_RELEASE;
		RC_MSG.channels[PITCH_CHANNEL] = RC_MSG.CHAN_RELEASE;
		RC_MSG.channels[THROTTLE_CHANNEL] = RC_MSG.CHAN_RELEASE;
		RC_MSG.channels[YAW_CHANNEL] = RC_MSG.CHAN_RELEASE;
	}

	void land_msg_channels(){
		RC_MSG.channels[MODE_CHANNEL] = LAND_MODE;

		RC_MSG.channels[ROLL_CHANNEL] = RC_MSG.CHAN_RELEASE;
		RC_MSG.channels[PITCH_CHANNEL] = RC_MSG.CHAN_RELEASE;
		RC_MSG.channels[YAW_CHANNEL] = MID_PWM;
		RC_MSG.channels[THROTTLE_CHANNEL] = MID_PWM;
	}
	
	void ai_msg_channels(){
		RC_MSG.channels[MODE_CHANNEL] = mode;

		RC_MSG.channels[ROLL_CHANNEL] = roll + ROLL_TRIM;
		RC_MSG.channels[PITCH_CHANNEL] = pitch + PITCH_TRIM;
		RC_MSG.channels[YAW_CHANNEL] = yaw + YAW_TRIM;
		RC_MSG.channels[THROTTLE_CHANNEL] = throttle;
	}

	int getNonBlocking()
	{
		struct termios initial_settings, new_settings;
		int n;

		unsigned char key;

		tcgetattr(0, &initial_settings);

		new_settings = initial_settings;
		new_settings.c_lflag &= ~ICANON;
		new_settings.c_lflag &= ~ECHO;
		new_settings.c_lflag &= ~ISIG;
		new_settings.c_cc[VMIN] = 0;
		new_settings.c_cc[VTIME] = 0;

		tcsetattr(0, TCSANOW, &new_settings);
		n = getchar();
		key = n;
		tcsetattr(0, TCSANOW, &initial_settings);

		return key;
	}

	void update_loop(){
		// get land command
		landChar = getNonBlocking();

		// one time only toggle to land mode
		if (landChar == ' ' && !LAND_TOGGLE){
			LAND_TOGGLE = true;
			flight_mode = land;
			std::cout << "LAND MODE ENGAGED" << std::endl;
		}

		switch(flight_mode){
		case ai:
			ai_msg_channels();
			break;
		case land:
			land_msg_channels();
			break;
		default:
			release_msg_channels();
			break;
		}
		if (DEBUG){
			std::cout 
				<< "mode: " 	<< RC_MSG.channels[MODE_CHANNEL] << std::endl
				<< "roll: "		<< RC_MSG.channels[ROLL_CHANNEL] << std::endl
				<< "pitch: "	<< RC_MSG.channels[PITCH_CHANNEL] << std::endl
				<< "yaw: " 		<< RC_MSG.channels[YAW_CHANNEL] << std::endl
				<< "throttle: " << RC_MSG.channels[THROTTLE_CHANNEL] << std::endl;
		}
	}

	void RCIn_callback(const mavros_msgs::RCIn& msg)
	{
		if (msg.channels[MANUAL_CONTROL] >= MID_PWM){
			if(!LAND_TOGGLE){
				std::cout << "manual_mode\n";
				flight_mode = manual;
			}
		}
		else{
			// must reset ai
			std_msgs::Bool reset_signal;
			reset_signal.data = true;
			ai_reset_pub.publish(reset_signal);
		}
	}

}

class robot_controller 
{
private:
	ros::NodeHandle n;
	
	short throttle, roll, pitch, yaw, mode, retracts;
	int camera_width, camera_height;
	double current_altitude,
		   target_altitude,

		   current_yolo_yaw,
		   target_yolo_yaw,

		   current_yolo_pitch,
		   target_yolo_pitch,

		   current_down_cam_pitch,
		   target_down_cam_pitch,

		   current_down_cam_roll,
		   target_down_cam_roll,

		   current_obstacle_pitch,
		   target_obstacle_pitch,

		   current_obstacle_roll,
		   target_obstacle_roll;
	
public:
	//contructor
	robot_controller()
	{
		debug = true;
		
		//initial values
		/// camera dimentions
		camera_width 			= 640;
		camera_height 			= 480;
		/// altitude is in meters with the point lidar
		current_altitude		= 0;
		target_altitude 		= 0;
		/// yolo yaw is the x value in pixels from the front camera
		current_yolo_yaw		= camera_width / 2;
		target_yolo_yaw			= camera_width / 2;
		/// yolo pitch is the y value in pixels from the front camera
		current_yolo_pitch		= camera_height;
		target_yolo_pitch		= camera_height;
		/// down cam pitch is the delta y value of the target to the center of the camera
		current_down_cam_pitch	= 0
		target_down_cam_pitch	= 0
		/// down cam pitch is the delta x value of the target to the center of the camera
		current_down_cam_roll	= 0
		target_down_cam_roll	= 0
		/// obstacle pitch is the y component of the obstacle avoidence vector, NOT the actual obstacle vector
		current_obstacle_pitch	= 0
		target_obstacle_pitch	= 0
		/// obstacle pitch is the x component of the obstacle avoidence vector, NOT the actual obstacle vector
		current_obstacle_roll	= 0
		target_obstacle_roll	= 0
		
		//subs:
		n.subscribe("/IARC/ai_reset", 1, &robot_controller::ai_reset_cb, this);
		n.subscribe("/IARC/currentAltitude", 1, &robot_controller::current_altitude_cb, this);
		n.subscribe("/IARC/setAltitude", 1, &robot_controller::target_altitude_cb, this);
		n.subscribe("/IARC/YOLO/target/x", 1, &robot_controller::target_yolo_x_cb, this);
		n.subscribe("/IARC/YOLO/target/y", 1, &robot_controller::target_yolo_y_cb, this);
		n.subscribe("/IARC/OrientationNet/pos/x", 1, &robot_controller::target_down_cam_x_cb, this);
		n.subscribe("/IARC/OrientationNet/pos/y", 1, &robot_controller::target_down_cam_y_cb, this);
		n.subscribe("/IARC/Obstacle/PitchPID", 1, &robot_controller::obstacle_pitch_cb, this);
		n.subscribe("/IARC/Obstacle/RollPID", 1, &robot_controller::obstacle_roll_cb, this);
	}

	void ai_reset_cb(const std_msgs::Bool& msg){

	}

	void current_altitude_cb(const std_msgs::Float32& msg){

	}

	void target_altitude_cb(const std_msgs::Float32& msg){

	}

	void target_yolo_x_cb(const std_msgs::Int16& msg){

	}

	void target_yolo_y_cb(const std_msgs::Int16& msg){

	}

	void target_down_cam_x_cb(const std_msgs::Int16& msg){

	}

	void target_down_cam_y_cb(const std_msgs::Int16& msg){

	}

	void obstacle_pitch_cb(const std_msgs::Float32& msg){

	}

	void obstacle_roll_cb(const std_msgs::Float32& msg){

	}

	void start_nav()
	{
		//speed of dx9 controller:
		ros::Rate fcuCommRate(45);
		
		xPosCtrl->targetSetpoint(target_x);
		yPosCtrl->targetSetpoint(target_y);
		zPosCtrl->targetSetpoint(target_z);
		
		while (ros::ok())
		{
			landChar = getchNonBlocking();   // call non-blocking input function to get keyboard inputs

			if (landChar == ' ' || EMERGENCY_LAND){
				if (!EMERGENCY_LAND){
					EMERGENCY_LAND = true;
					std::cout << "EMERGENCY LAND ENGAGED" << std::endl;
				}
			}
			//This will only work if our coordinate system is consistant.
			pitch = MID_PWM - xPosCtrl->calc(current_x); // Pitch value for Forward is negative
			roll = MID_PWM - yPosCtrl->calc(current_y); // Roll value for Left is negative			
			throttle = MID_PWM + zPosCtrl->calc(current_z); 
			
			if (!NAV_CONNECT)
			{
				throttle = LOW_PWM + zPosCtrl->calc(current_z);
			}
			
			if(EMERGENCY_LAND)
			{
				land_msg_channels();
			}			
			else if(MANUAL_OVERRIDE)
			{
				release_msg_channels();
			}
			else
			{
				ai_msg_channels();
			}
			
			if (debug)
			{
				std::cout << "roll: "<< RC_MSG.channels[ROLL_CHANNEL] << std::endl
					<< "pitch: "<< RC_MSG.channels[PITCH_CHANNEL] << std::endl
					<< "throttle: " << RC_MSG.channels[THROTTLE_CHANNEL] << std::endl
					<< "mode: " << RC_MSG.channels[MODE_CHANNEL] << std::endl
					<< "yaw: " << RC_MSG.channels[YAW_CHANNEL] << std::endl
					<< "retracts: " << RC_MSG.channels[RETRACT_CHANNEL] << std::endl;
			}
			rc_pub.publish(RC_MSG);
			
			ros::spinOnce();
			fcuCommRate.sleep();
		}
	}
	
	void release_msg_channels()
	{
		RC_MSG.channels[MODE_CHANNEL] = STABILIZE_MODE;

		RC_MSG.channels[ROLL_CHANNEL] = RC_MSG.CHAN_RELEASE;
		RC_MSG.channels[PITCH_CHANNEL] = RC_MSG.CHAN_RELEASE;
		RC_MSG.channels[THROTTLE_CHANNEL] = RC_MSG.CHAN_RELEASE;
		RC_MSG.channels[YAW_CHANNEL] = RC_MSG.CHAN_RELEASE;

		RC_MSG.channels[RETRACT_CHANNEL]=HIGH_PWM;
	}
	
	void land_msg_channels()
	{
		RC_MSG.channels[MODE_CHANNEL] = LAND_MODE;

		RC_MSG.channels[ROLL_CHANNEL] = RC_MSG.CHAN_RELEASE;
		RC_MSG.channels[PITCH_CHANNEL] = RC_MSG.CHAN_RELEASE;
		RC_MSG.channels[THROTTLE_CHANNEL] = MID_PWM;
		RC_MSG.channels[YAW_CHANNEL] = MID_PWM;

		RC_MSG.channels[RETRACT_CHANNEL]=HIGH_PWM;
	}
	
	void ai_msg_channels()
	{
		RC_MSG.channels[MODE_CHANNEL] = mode;

		RC_MSG.channels[ROLL_CHANNEL] = roll + 82;
		RC_MSG.channels[PITCH_CHANNEL] = pitch - 36;
		RC_MSG.channels[THROTTLE_CHANNEL] = throttle;
		RC_MSG.channels[YAW_CHANNEL] = MID_PWM;

		RC_MSG.channels[RETRACT_CHANNEL]=retracts;
	}
	
	void setpoint_callback(const geometry_msgs::Point& setpoint)
	{
		if (!NAV_CONNECT)
			NAV_CONNECT = true;
		target_x = setpoint.x;
		target_y = setpoint.y;
		if (setpoint.z > MAX_HEIGHT)
			target_z = MAX_HEIGHT;
		else
			target_z = setpoint.z;
			
	    //Update the setpoint on the controllers
	    xPosCtrl->targetSetpoint(target_x);
	    yPosCtrl->targetSetpoint(target_y);
	    zPosCtrl->targetSetpoint(target_z);
	}
	
	void loc_callback(const geometry_msgs::PoseStamped& cur_loc)
	{
		current_x = cur_loc.pose.position.x;
		current_y = cur_loc.pose.position.y;
		current_z = cur_loc.pose.position.z;
		std::cout << "altitude: " << current_z << std::endl;
		if (current_z > 5.0)
			EMERGENCY_LAND = true;
	}
	
	void MANUAL_OVERRIDE_callback(const std_msgs::Bool& msg)
	{
			MANUAL_OVERRIDE = msg.data;
	}
	
	
	void emer_land_callback(const std_msgs::Bool& msg)
	{
		EMERGENCY_LAND = msg.data;
	}
	
	
	void mode_callback(const std_msgs::Int8& msg)
	{
		if (msg.data == 0)
			mode = ALT_HOLD_MODE;
		else if(msg.data == 1)
			mode = STABILIZE_MODE;
		else
			mode = LAND_MODE;
	}
	
	void retract_callback(const std_msgs::Bool& msg)
	{
		if (msg.data == true)
			retracts = HIGH_PWM;
		else
			retracts = LOW_PWM;
	}
	
	void pidXY_callback(const std_msgs::Int32MultiArray& arr_msg)
	{
		PIDController* tmp;
		tmp = xPosCtrl;
		xPosCtrl = new PIDController(arr_msg.data[0], arr_msg.data[1], arr_msg.data[2], arr_msg.data[3], arr_msg.data[4]);
		delete tmp;
		tmp = yPosCtrl;
		yPosCtrl = new PIDController(arr_msg.data[0], arr_msg.data[1], arr_msg.data[2], arr_msg.data[3], arr_msg.data[4]);
		delete tmp;
		
	}
	
	void pidZ_callback(const std_msgs::Int32MultiArray& arr_msg)
	{
		PIDController* tmp;
		tmp = zPosCtrl;
		zPosCtrl = new PIDController(arr_msg.data[0], arr_msg.data[1], arr_msg.data[2], arr_msg.data[3], arr_msg.data[4]);
		delete tmp;
	}
	
	
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "controller");
	robot_controller c_1;
	c_1.start_nav();
}
	
