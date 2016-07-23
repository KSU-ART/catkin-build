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
 * mannual_override overrides emergency_land
 * emergency_land overrides ai
 * 
 * 
 * Mannual Override Switch: 
 * channel 9 (8 in {0-8}) HIGH_PWM is land
 * 
 **************************************************************************************************/

#include "robot_controller.h"

class robot_controller 
{
private:
	ros::NodeHandle n,s;
	ros::Publisher rc_pub;
	ros::Subscriber setpoint_sub, loc_sub, mode_sub, retract_sub, man_override_sub, land_sub, pid_sub, subRCIn;
	
	short throttle, roll, pitch, yaw, mode, retracts;
	double target_x, current_x, target_y, current_y, target_z, current_z;
	char landChar;
	//RC msg container that will be sent to the FC @ fcuCommRate hz
	mavros_msgs::OverrideRCIn RC_MSG;

	
	PIDController* xPosCtrl;
	PIDController* yPosCtrl;
	PIDController* zPosCtrl;

	bool MANNUAL_OVERRIDE, EMERGENCY_LAND, MAN_SWITCH, NAV_CONNECT;
	bool debug;
	
public:
	robot_controller()
	{
		//pubs:
		rc_pub = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);
		
		//this part is important:
		MANNUAL_OVERRIDE = true;
		EMERGENCY_LAND = false;
		debug = true;
		
		//initial values
		throttle = LOW_PWM;
		roll = MID_PWM;
		pitch = MID_PWM;
		yaw = MID_PWM;
		mode = ALT_HOLD_MODE;
		retracts = HIGH_PWM; 
		target_x = 0;
		current_x = 0;
		target_y = 0;
		current_y = 0;
		target_z = 0;
		current_z = 0;
		landChar = 'f';
		NAV_CONNECT = false;
		
		//default PIDs
		xPosCtrl = new PIDController(80, 0, 0, -250, 250);
		yPosCtrl = new PIDController(80, 0, 0, -250, 250);
		zPosCtrl = new PIDController(250, 0, 0, -500, 500);
		
		xPosCtrl->on();
		yPosCtrl->on();
		zPosCtrl->on();
		
		//subs:
		subRCIn = s.subscribe("/mavros/rc/in", 1, &robot_controller::RCIn_callback, this);
		setpoint_sub = s.subscribe("/ai_nav/setpoint", 1, &robot_controller::setpoint_callback, this);
		loc_sub = s.subscribe("/localizer/curent_pose", 1, &robot_controller::loc_callback, this);
		man_override_sub = s.subscribe("manOverrideMsg", 1, &robot_controller::mannual_override_callback, this);
		land_sub = s.subscribe("EMERGENCY_LAND", 1, &robot_controller::emer_land_callback, this);
		mode_sub = s.subscribe("/ai_nav/modeMsg", 1, &robot_controller::mode_callback, this);
		retract_sub = s.subscribe("/ai_nav/retractMsg", 1, &robot_controller::retract_callback, this);
		pid_sub = s.subscribe("/ai_nav/pid_XY", 1, &robot_controller::pidXY_callback, this);
		pid_sub = s.subscribe("/ai_nav/pid_Z", 1, &robot_controller::pidZ_callback, this);
		
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
			//xPosCtrl->targetSetpoint(target_x);
			pitch = MID_PWM - xPosCtrl->calc(current_x); // Pitch value for Forward is negative
			//yPosCtrl->targetSetpoint(target_y);
			roll = MID_PWM - yPosCtrl->calc(current_y); // Roll value for Left is negative
			//zPosCtrl->targetSetpoint(target_z);
			throttle = MID_PWM + zPosCtrl->calc(current_z); 
			if (!NAV_CONNECT)
			{
				throttle = LOW_PWM + zPosCtrl->calc(current_z);
			}
			
			if(EMERGENCY_LAND)
			{
				land_msg_channels();
			}			
			else if(MANNUAL_OVERRIDE)
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
	
	int getchNonBlocking()
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
	void release_msg_channels()
	{
		RC_MSG.channels[ROLL_CHANNEL] = RC_MSG.CHAN_RELEASE;
		RC_MSG.channels[PITCH_CHANNEL] = RC_MSG.CHAN_RELEASE;
		RC_MSG.channels[THROTTLE_CHANNEL] = RC_MSG.CHAN_RELEASE;
		RC_MSG.channels[MODE_CHANNEL] = STABILIZE_MODE;
		RC_MSG.channels[YAW_CHANNEL] = RC_MSG.CHAN_RELEASE;
		RC_MSG.channels[RETRACT_CHANNEL]=HIGH_PWM;
	}
	
	void land_msg_channels()
	{
		RC_MSG.channels[ROLL_CHANNEL] = RC_MSG.CHAN_RELEASE;
		RC_MSG.channels[PITCH_CHANNEL] = RC_MSG.CHAN_RELEASE;
		RC_MSG.channels[THROTTLE_CHANNEL] = MID_PWM;
		RC_MSG.channels[MODE_CHANNEL] = LAND_MODE;
		RC_MSG.channels[YAW_CHANNEL] = MID_PWM;
		RC_MSG.channels[RETRACT_CHANNEL]=HIGH_PWM;
	}
	
	void ai_msg_channels()
	{
		RC_MSG.channels[ROLL_CHANNEL] = roll;
		RC_MSG.channels[PITCH_CHANNEL] = pitch;
		RC_MSG.channels[THROTTLE_CHANNEL] = throttle;
		RC_MSG.channels[MODE_CHANNEL] = mode;
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
	}
	
	void loc_callback(const geometry_msgs::PoseStamped& cur_loc)
	{
		current_x = cur_loc.pose.position.x;
		current_y = cur_loc.pose.position.y;
		current_z = cur_loc.pose.position.z;
		if (current_z > 5.0)
			EMERGENCY_LAND = true;
	}
	
	void mannual_override_callback(const std_msgs::Bool& msg)
	{
			MANNUAL_OVERRIDE = msg.data;
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
	
	void RCIn_callback(const mavros_msgs::RCIn& msg)
	{
		if (msg.channels[MANUAL_CONTROL] >= MID_PWM)
		{
			std::cout << "mannual_mode\n";
			MANNUAL_OVERRIDE = true;
			MAN_SWITCH = true;
		}
		else
		{
			MAN_SWITCH = false;
			MANNUAL_OVERRIDE = false;
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "controller");
	robot_controller c_1;
	c_1.start_nav();
}
	
