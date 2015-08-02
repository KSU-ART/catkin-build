#include "spsuart/autonomous.h"
#include "px_comm/OpticalFlow.h"
#include <termios.h>
#include "sensor_msgs/LaserScan.h"

using namespace std;
using namespace cv;
  
namespace enc = sensor_msgs::image_encodings;        

int throttle = LOW_PWM;
int roll = MID_PWM;
int pitch = MID_PWM;

double target_altitude = 1.5;

double ground_distance;

// Instantiate PID controllers
PIDController* xVelCtrl = new PIDController(1,0,0,-100,100);
PIDController* yVelCtrl = new PIDController(1,0,0,-100,100);
PIDController* altPosCtrl = new PIDController(500,0,0,-300,300);
mavlink_message_t* msgt = NULL;
__mavlink_rangefinder_t* x = NULL;

/*
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
*/

void splitScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {	
	double avg = 0;
	for(int i = 0; i < sizeof(msg->ranges)/4; i++) {
		if(msg->ranges[i] < 4){
			avg += (double)(msg->ranges[i]/(sizeof(msg->ranges)/4));
		}
		else{	
			avg += (double)(4/(sizeof(4)));
		}
	}
	ground_distance = avg;	
	ROS_DEBUG("Altitude: %d", avg);
	double out = altPosCtrl->calc(ground_distance);
	ROS_DEBUG("PID Out: %d", out);
	throttle = MID_PWM + out;
	
}

void optFlowCallback(const px_comm::OpticalFlow::ConstPtr& msg) 
{ 
   //double vel_x = msg->velocity_x;
   // double vel_y = msg->velocity_y;
   // pos_z = msg->ground_distance; 
   
   // double x_out = xVelCtrl->calc(vel_x);
   // double y_out = yVelCtrl->calc(vel_y);

   //roll = MID_PWM + x_out;
   //pitch = MID_PWM + y_out; 
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
  ros::Subscriber subflow = n.subscribe("px4flow/opt_flow",1,optFlowCallback);
  //ros::Subscriber submav = n.subscribe("mavlink/from", 1, apmMavlinkmsgCallback);
  ros::Subscriber subHokuyo = n.subscribe("scan3", 1, splitScanCallback); 
  //Mavros rc override publisher
  ros::Publisher rc_pub = n.advertise<mavros::OverrideRCIn>("/mavros/rc/override", 1);
  
  //RC msg container that will be sent to the FC @ fcuCommRate hz
  mavros::OverrideRCIn msg;
  ros::Rate fcuCommRate(45); // emulating speed of dx9 controller
  
  // Set PID controller targets  
  // xVelCtrl->targetSetpoint(0); // target X coordinate in pixels
  // yVelCtrl->targetSetpoint(0); // target Y coordinate in pixels
  altPosCtrl->targetSetpoint(target_altitude); // target altitude in meters
  int inputChar = 'a';
  
  bool land = false;
  
    //While node is alive send RC values to the FC @ fcuCommRate hz
    while(ros::ok()){

            //msg.channels[ROLL_CHANNEL]=roll;
			//msg.channels[PITCH_CHANNEL]=pitch;
            msg.channels[ROLL_CHANNEL]=msg.CHAN_RELEASE;
			msg.channels[PITCH_CHANNEL]=msg.CHAN_RELEASE;
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
