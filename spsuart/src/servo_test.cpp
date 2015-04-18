#include "spsuart/autonomous.h"

using namespace std;
using namespace cv;
  
namespace enc = sensor_msgs::image_encodings;        

int throttle = LOW_PWM;
int roll = MID_PWM;
int pitch = MID_PWM;

double alt = 5.0; // current altitude in meters

// Instantiate PID controllers
//PIDController* xPosCtrl = PIDController();
//PIDController* yPosCtrl = PIDController();
PIDController* altPosCtrl = new PIDController();
mavlink_message_t* msgt = NULL;
__mavlink_rangefinder_t* x = NULL;

// Time reference for PID controllers
double nowTimeALT = 0;
double nowTimeImagePoint = 0;


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
                alt = x->distance;
                
                nowTimeALT = ros::Time::now().toSec();
                
                // increase (or decrease?) altitude based on value from pid
                throttle = MID_PWM + altPosCtrl->calc(alt, nowTimeALT);

                delete msgt;
                delete x;

                msgt = NULL;
                x = NULL;
        }
}

void callbackImagePoint(const ros_opencv::TrackingPoint::ConstPtr& msgs){
/*
// Get X and Y coordinates of color center in image
double x=msgs->pointX;
double y=msgs->pointY;

// Get the current time for the PID controllers
nowTimeImagePoint = = ros::Time::now().toSec();

// If color not detected in image, hold position and keep the current time up-to-date in the PID controllers
if(x<0 || y<0) {
                roll=MID_PWM;
                pitch=MID_PWM;
                xPosCtrl->calc(xPosCtrl->getSetpoint(), nowTimeImagePoint);
                yPosCtrl->calc(yPosCtrl->getSetpoint(), nowTimeImagePoint);
        }
        // Color center detected in image, calculate roll and pitch corrections to move the hexacopter over the object.
        else {
                roll = MID_PWM - xPosCtrl->calc(x, nowTimeImagePoint);
                pitch = MID_PWM - yPosCtrl->calc(y, nowTimeImagePoint);
        }
*/        
}

int main(int argc, char **argv)
{
  //ROS node init and NodeHandle init
  ros::init(argc, argv, "servo_test");
  ros::NodeHandle n;
  
  //Image and mavlink message subscriber
  ros::Subscriber subpoint = n.subscribe("test/image_point", 1, callbackImagePoint);
  ros::Subscriber submav = n.subscribe("mavlink/from", 1, apmMavlinkmsgCallback);
  
  //Mavros rc override publisher
  ros::Publisher rc_pub = n.advertise<mavros::OverrideRCIn>("/mavros/rc/override", 1);
  
  //RC msg container that will be sent to the FC @ fcuCommRate hz
  mavros::OverrideRCIn msg;
  ros::Rate fcuCommRate(45); // emulating speed of dx9 controller
  
  // The amount of PID stuff here is ridiculous but my constructors in the PIDController class are broken so this is the workaround =(
  // Initialize PID controller gains
  //xPosCtrl.setGains(1, 0, 0);
  //yPosCtrl.setGains(1, 0, 0);
  altPosCtrl->setGains(500, 0.0, 0);
  
  // Initialize PID controller constraints
  //xPosCtrl.setConstraints(-100, 100);
  //yPosCtrl.setConstraints(-100, 100);
  altPosCtrl->setConstraints(-300, 300);
  
  // Zero out differentiator, integrator, and time variables in the PID controllers
  //xPosCtrl.init();
  //yPosCtrl.init();
  altPosCtrl->init();
  
  // Set PID controller targets  
  //xPosCtrl.targetSetpoint(320); // target X coordinate in pixels
  //yPosCtrl.targetSetpoint(240); // target Y coordinate in pixels
  altPosCtrl->targetSetpoint(2.0); // target altitude in meters

	int svpwm = 1000;

    //While node is alive send RC values to the FC @ fcuCommRate hz
    while(ros::ok() && svpwm <2000){
        msg.channels[ROLL_CHANNEL]=msg.CHAN_NOCHANGE;
        msg.channels[PITCH_CHANNEL]=msg.CHAN_NOCHANGE;
        msg.channels[THROTTLE_CHANNEL]=msg.CHAN_NOCHANGE;
        msg.channels[YAW_CHANNEL]=msg.CHAN_NOCHANGE;
        msg.channels[MODE_CHANNEL]=STABILIZE_MODE;
        msg.channels[NOT_USED_CHANNEL]=msg.CHAN_NOCHANGE;
        msg.channels[GIMBAL_ROLL_CHANNEL]=svpwm;
        msg.channels[GIMBAL_TILT_CHANNEL]=svpwm;
        svpwm++;
        rc_pub.publish(msg);  
        ros::spinOnce();
        fcuCommRate.sleep();        
    }

    return 0;
}
