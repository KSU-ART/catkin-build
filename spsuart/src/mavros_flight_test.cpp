#include "spsuart/autonomous.h"
#include <termios.h>

#define LANDING_ALTITUDE 0.5

using namespace std;
using namespace cv;
  
namespace enc = sensor_msgs::image_encodings;        

int throttle = LOW_PWM;
int roll = MID_PWM;
int pitch = MID_PWM;

bool xy_idle = false;

double target_altitude = 2.0;

// Instantiate PID controllers
PIDController* xPosCtrl = new PIDController(0.25,0,0,-60,60);
PIDController* yPosCtrl = new PIDController(0.32,0,0,-60,60);
PIDController* altPosCtrl = new PIDController(500,0,0,-300,300);
mavlink_message_t* msgt = NULL;
__mavlink_rangefinder_t* x = NULL;

void apmMavlinkmsgCallback(const mavros::Mavlink::ConstPtr& msg){
        if(msg->msgid==173){

                if(msgt == NULL)
                        msgt = new mavlink_message_t();

                if(x == NULL)
                        x = new __mavlink_rangefinder_t();

                msgt->seq = msg->seq;
                msgt->len = msg->len;
                msgt->sysid = msg->sysid;
                msgt->compid = msg->compid;
                msgt->msgid = msg->msgid;
                msgt->payload64[0] = msg->payload64[0];

                mavlink_msg_rangefinder_decode(msgt, x); 
                
                // increase (or decrease?) altitude based on value from pid
				cout<<"Altitude :"<<x->distance<<endl;                
		double out = altPosCtrl->calc(double(x->distance));
                cout << "PID Out: " << out << endl;
				throttle = MID_PWM + out;
		delete msgt;
                delete x;

                msgt = NULL;
                x = NULL;
        }
}

void callbackImagePoint(const ros_opencv::TrackingPoint::ConstPtr& msgs){

// Get X and Y coordinates of color center in image
double x=msgs->pointX;
double y=msgs->pointY;

// Get the current time for the PID controllers

// If color not detected in image, hold position and keep the current time up-to-date in the PID controllers
if(x<0 || y<0) {
                xy_idle = true;
                xPosCtrl->calc(xPosCtrl->getSetpoint());
                yPosCtrl->calc(yPosCtrl->getSetpoint());
        }
        // Color center detected in image, calculate roll and pitch corrections to move the hexacopter over the object.
        else {
                xy_idle = false;
                roll = MID_PWM + xPosCtrl->calc(x);
                pitch = MID_PWM - yPosCtrl->calc(y);
        }
       
}

int constrain(int value, int min, int max)
{
	if(value > max)
	{
		return max;
	}
	else if(value < min)
	{
		return min;
	}
	else
	{
		return value;
	}
}

int getchNonBlocking()
{
  struct termios initial_settings,
               new_settings;
  int n;
 
  unsigned char key;
 
 
 
  tcgetattr(0,&initial_settings);
 
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

int main(int argc, char **argv)
{
  //ROS node init and NodeHandle init
  ros::init(argc, argv, "mavros_flight_test");
  ros::NodeHandle n;
  
  //Image and mavlink message subscriber
  ros::Subscriber subpoint = n.subscribe("image_point", 1, callbackImagePoint);
  ros::Subscriber submav = n.subscribe("mavlink/from", 1, apmMavlinkmsgCallback);
  
  //Mavros rc override publisher
  ros::Publisher rc_pub = n.advertise<mavros::OverrideRCIn>("/mavros/rc/override", 1);
  
  //RC msg container that will be sent to the FC @ fcuCommRate hz
  mavros::OverrideRCIn msg;
  ros::Rate fcuCommRate(45); // emulating speed of dx9 controller
  
  // Set PID controller targets  
  //xPosCtrl->targetSetpoint(320); // target X coordinate in pixels
  //yPosCtrl->targetSetpoint(240); // target Y coordinate in pixels
  altPosCtrl->targetSetpoint(target_altitude); // target altitude in meters
  int inputChar = 'a';
  
  bool land = false;
  
    //While node is alive send RC values to the FC @ fcuCommRate hz
    while(ros::ok()){

		
		if(altPosCtrl->hasSettled() && target_altitude == 2.0)
		{
			target_altitude = LANDING_ALTITUDE;
  			altPosCtrl->targetSetpoint(target_altitude); 
		}
		
			//if(xy_idle)
			//{
				msg.channels[ROLL_CHANNEL]=msg.CHAN_RELEASE;
				msg.channels[PITCH_CHANNEL]=msg.CHAN_RELEASE;
		    //}	
		    // else{
			//	msg.channels[ROLL_CHANNEL]=roll;
			//	msg.channels[PITCH_CHANNEL]=pitch;
			//}
			msg.channels[THROTTLE_CHANNEL]=throttle;
			msg.channels[YAW_CHANNEL]=msg.CHAN_RELEASE;
			msg.channels[MODE_CHANNEL]=ALT_HOLD_MODE;
			msg.channels[NOT_USED_CHANNEL]=msg.CHAN_RELEASE;
			msg.channels[GIMBAL_TILT_CHANNEL]=constrain(GIMBAL_TILT_MAX,GIMBAL_TILT_MIN,GIMBAL_TILT_MAX);
			msg.channels[GIMBAL_ROLL_CHANNEL]=constrain(GIMBAL_ROLL_TRIM,GIMBAL_ROLL_MIN,GIMBAL_ROLL_MAX);
		
		inputChar = getchNonBlocking();   // call non-blocking input function
		
		if(inputChar == ' ' || land){
			if(!land){
				land= true;
				cout<<"EMERGENCY LAND ENGAGED"<<endl;
		    }
			
			msg.channels[ROLL_CHANNEL]=msg.CHAN_RELEASE;
			msg.channels[PITCH_CHANNEL]=msg.CHAN_RELEASE;
			msg.channels[THROTTLE_CHANNEL]=msg.CHAN_RELEASE;
			msg.channels[MODE_CHANNEL]=LAND_MODE;

		}
		
		rc_pub.publish(msg);  
        ros::spinOnce();
        fcuCommRate.sleep();
		
	}
    return 0;
}
