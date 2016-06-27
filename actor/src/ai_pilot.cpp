/***********************************************
 * This is the class using the pid controller to 
 * generate output to the pixhawk.
 **********************************************/
#include "ai_pilot.h"
class ai_pilot 
{
private:
	ros::NodeHandle n,s;
	ros::Publisher rc_pub;
	ros::Subscriber setpoint_sub, loc_sub;
	
	int throttle, roll, pitch, yaw, mode, retract;
	double target_x, current_x, target_y, current_y, target_z, current_z;
	
	//RC msg container that will be sent to the FC @ fcuCommRate hz
	mavros_msgs::OverrideRCIn RC_MSG;

	
	PIDController* xPosCtrl;
	PIDController* yPosCtrl;
	PIDController* zPosCtrl;

	bool MANNUAL_OVERRIDE, EMERGENCY_LAND;
	
public:
	ai_pilot()
	{
		rc_pub = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);
		
		setpoint_sub = s.subscribe("setpoint", 1, &ai_pilot::setpoint_callback, this);
		loc_sub = s.subscribe("location", 1, &ai_pilot::loc_callback, this);

		//this part is important:
		MANNUAL_OVERRIDE = true;
		EMERGENCY_LAND = false;
		
		//initial values
		throttle = LOW_PWM;
		roll = MID_PWM;
		pitch = MID_PWM;
		yaw = MID_PWM;
		mode = ALT_HOLD_MODE;
		retract = HIGH_PWM; 

		target_x = 0;
		current_x = 0;
		target_y = 0;
		current_y = 0;
		target_z = 0;
		current_z = 0;
		
		xPosCtrl = new PIDController(80, 0, 0, -250, 250);
		yPosCtrl = new PIDController(80, 0, 0, -250, 250);
		zPosCtrl = new PIDController(250, 0, 0, -500, 500);
		
		xPosCtrl->on();
		yPosCtrl->on();
		zPosCtrl->on();
		
		
	}
	
	void start_nav()
	{
		//speed of dx9 controller:
		ros::Rate fcuCommRate(45);
		
		while (ros::ok())
		{
			//This will only work if our coordinate system is consistant.
			xPosCtrl->targetSetpoint(target_x);
			roll = MID_PWM + xPosCtrl->calc(current_x);
			yPosCtrl->targetSetpoint(target_y);
			pitch = MID_PWM + yPosCtrl->calc(current_y);
			zPosCtrl->targetSetpoint(target_z);
			throttle = MID_PWM + zPosCtrl->calc(current_z);
						
			if(MANNUAL_OVERRIDE)
			{
				release_msg_channels();
			}
			else if(EMERGENCY_LAND)
			{
				land_msg_channels();
			}
			else
			{
				populate_msg_channels();
			}
			
			rc_pub.publish(RC_MSG);
			
			fcuCommRate.sleep();
			ros::spinOnce();
		}
	}
	
	/*void pwmVector(int mag, double theta, int*  xVar, int* yVar) 
	{
		if (mag > 500) 
		{
			mag = 500;
		}
		else if (mag < -500) 
		{
			mag = -500;
		}
		theta = theta * (M_PI / 180);
		
		*xVar = int(MID_PWM - mag * sin(theta));
		*yVar = int(MID_PWM - mag * cos(theta));
	}
	//*/
	
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
	
	void populate_msg_channels()
	{
		RC_MSG.channels[ROLL_CHANNEL] = roll;
		RC_MSG.channels[PITCH_CHANNEL] = pitch;
		RC_MSG.channels[THROTTLE_CHANNEL] = throttle;
		RC_MSG.channels[MODE_CHANNEL] = mode;
		RC_MSG.channels[YAW_CHANNEL] = MID_PWM;
		RC_MSG.channels[RETRACT_CHANNEL]=retract;
	}
	
	void setpoint_callback(const geometry_msgs::Point& setpoint)
	{
		target_x = setpoint.x;
		target_y = setpoint.y;
		target_z = setpoint.z;
	}
	
	void loc_callback(const geometry_msgs::PoseStamped& cur_loc)
	{
		current_x = cur_loc.pose.position.x;
		current_y = cur_loc.pose.position.y;
		current_z = cur_loc.pose.position.z;
	}
};
int main(int argc, char **argv)
{
	ros::init(argc, argv, "actor");
	ai_pilot ai_1;
	ai_1.start_nav();
}
	
