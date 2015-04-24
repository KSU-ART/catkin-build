#include "spsuart/autonomous.h"

using namespace std;
using namespace cv;
  
namespace enc = sensor_msgs::image_encodings;        

int throttle = LOW_PWM;
int roll = MID_PWM;
int pitch = MID_PWM;

double alt = 5.0; // current altitude in meters

// Instantiate PID controllers
//PIDController* xPosCtrl = PIDController(0.25,0,0,-60,60);
//PIDController* yPosCtrl = PIDController(0,32,0,0,-60,60);
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
                alt = x->distance;
                               
                // increase (or decrease?) altitude based on value from pid
                throttle = MID_PWM + altPosCtrl->calc(alt);

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

// If color not detected in image, hold position and keep the current time up-to-date in the PID controllers
if(x<0 || y<0) {
                roll=MID_PWM;
                pitch=MID_PWM;
                xPosCtrl->calc(xPosCtrl->getSetpoint());
                yPosCtrl->calc(yPosCtrl->getSetpoint());
        }
        // Color center detected in image, calculate roll and pitch corrections to move the hexacopter over the object.
        else {
                roll = MID_PWM - xPosCtrl->calc(x);
                pitch = MID_PWM - yPosCtrl->calc(y);
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
