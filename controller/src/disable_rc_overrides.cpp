#include "robot_controller.h"

using namespace std;       

int main(int argc, char **argv)
{
  //ROS node init and NodeHandle init
  ros::init(argc, argv, "disable_rc_overrides");
  ros::NodeHandle n;
  
  //Mavros rc override publisher
  ros::Publisher rc_pub = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);
  
  //RC msg container that will be sent to the FC @ fcuCommRate hz
  mavros_msgs::OverrideRCIn msg;
  ros::Rate fcuCommRate(45); // emulating speed of dx9 controller
  
  
    //While node is alive send RC values to the FC @ fcuCommRate hz
    while(ros::ok()){
			msg.channels[ROLL_CHANNEL]=msg.CHAN_RELEASE;
			msg.channels[PITCH_CHANNEL]=msg.CHAN_RELEASE;
           	msg.channels[THROTTLE_CHANNEL]=msg.CHAN_RELEASE;
			msg.channels[YAW_CHANNEL]=msg.CHAN_RELEASE;
			msg.channels[MODE_CHANNEL]=msg.CHAN_RELEASE;
			msg.channels[RETRACT_CHANNEL]=HIGH_PWM;
			msg.channels[GIMBAL_PITCH_CHANNEL]= msg.CHAN_RELEASE; //constrain(GIMBAL_TILT_MAX,GIMBAL_TILT_MIN,GIMBAL_TILT_MAX);
			msg.channels[GIMBAL_YAW_CHANNEL]= msg.CHAN_RELEASE; //constrain(GIMBAL_ROLL_TRIM,GIMBAL_ROLL_MIN,GIMBAL_ROLL_MAX);
			rc_pub.publish(msg);  
			fcuCommRate.sleep();
		}
		
    return 0;
}
