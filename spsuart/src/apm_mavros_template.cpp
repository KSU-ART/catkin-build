#include "spsuart/autonomous.h"
#include "px_comm/OpticalFlow.h"
#include <termios.h>
#include "sensor_msgs/LaserScan.h"
#include "ros_opencv/TrackingPoint.h"
#include "ros_opencv/ObstacleDetected.h"
#include "ros_opencv/Alt.h"
#include "math.h"

using namespace std;
using namespace cv;

namespace enc = sensor_msgs::image_encodings;

int throttle = LOW_PWM;
int roll = MID_PWM;
int pitch = MID_PWM;
int yaw = MID_PWM;
int mode = ALT_HOLD_MODE;
int retract = HIGH_PWM;

double target_altitude = 1.5;
double ground_distance;

ros::Publisher alt_pub;

// Instantiate PID controllers
PIDController* altPosCtrl = new PIDController(250, 0, 0, -500, 500);

enum State {
    TakeOff = 0,
    Land = 1,
    ManualCtrl = 2
};

State currentState = TakeOff;

/* Lidar based alt-hold callback */
void splitScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    ros_opencv::Alt altmsg;

    double avg = 0;
    for (int i = 0; i < sizeof(msg->ranges) / 4; i++) {
        if (msg->ranges[i] < 4) {
            avg += (double)(msg->ranges[i] / (sizeof(msg->ranges) / 4));
        }
        else {
            avg += (double)(4 / (sizeof(4)));
        }
    }
    ground_distance = avg;
    ROS_DEBUG("Altitude: %d", avg);
    double out = altPosCtrl->calc(ground_distance);
    ROS_DEBUG("PID Out: %d", out);
    throttle = MID_PWM + out;
    /* If the vehicle is in the takeoff state and has settled switch to enter arena */
    if (currentState == TakeOff && ground_distance > 1.45 && ground_distance < 1.55) {
        currentState = ManualCtrl;
        cout << "Current state: Manual RPY Control" << endl;
    }

    //Retract the landing gear
    if(ground_distance > 1.1 && currentState != Land) {
        retract = LOW_PWM;
    }
    //Deploy the landing gear
    else if (ground_distance < .9 && currentState != Land) {
        retract = HIGH_PWM;
    }

    altmsg.alt = ground_distance;
    alt_pub.publish(altmsg);
}

int constrain(int value, int min, int max)
{
    if (value > max)
    {
        return max;
    }
    else if (value < min)
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

void pwmVector(int mag, double theta, int*  xVar, int* yVar) {
    if (mag > 500) {
        mag = 500;
    }
    else if (mag < -500) {
        mag = -500;
    }
    theta = theta * (M_PI / 180);
    *xVar = int(MID_PWM - mag * sin(theta));
    *yVar = int(MID_PWM - mag * cos(theta));
}

int main(int argc, char **argv)
{
    //ROS node init and NodeHandle init
    ros::init(argc, argv, "apm_mavros_template");
    ros::NodeHandle n;
	
	//LiDAR callback
    ros::Subscriber subHokuyo = n.subscribe("scan3", 1, splitScanCallback);
    
    //Mavros rc override publisher
    ros::Publisher rc_pub = n.advertise<mavros::OverrideRCIn>("/mavros/rc/override", 1);
    alt_pub = n.advertise<ros_opencv::Alt>("/spsuart/alt", 1);

    //RC msg container that will be sent to the FC @ fcuCommRate hz
    mavros::OverrideRCIn msg;
    ros::Rate fcuCommRate(45); // emulating speed of dx9 controller

    int inputChar = 'a';
    bool land = false;
    
    altPosCtrl->on();
    cout << "Current state: Take off" <<endl;

    //While node is alive send RC values to the FC @ fcuCommRate hz
    while (ros::ok()) {
        switch (currentState) {
        case TakeOff:
            pwmVector(30, 0, &roll, &pitch);
            yaw = msg.CHAN_RELEASE;
            altPosCtrl->targetSetpoint(1.5); // target altitude in meters
            mode = ALT_HOLD_MODE;
            break;
        case Land:
            roll = msg.CHAN_RELEASE;
            pitch = msg.CHAN_RELEASE;
            yaw = msg.CHAN_RELEASE;
            throttle = MID_PWM;
            mode = LAND_MODE;
            retract = HIGH_PWM;
            break;
        case ManualCtrl:
			roll = msg.CHAN_RELEASE;
			pitch = msg.CHAN_RELEASE;
			yaw = msg.CHAN_RELEASE;
			altPosCtrl->targetSetpoint(1.5); // target altitude in meters
            mode = ALT_HOLD_MODE;
        }

        msg.channels[ROLL_CHANNEL] = roll;
        msg.channels[PITCH_CHANNEL] = pitch;
        msg.channels[THROTTLE_CHANNEL] = throttle;
        msg.channels[MODE_CHANNEL] = mode;
        msg.channels[YAW_CHANNEL] = yaw;
        msg.channels[RETRACT_CHANNEL]=retract;
        msg.channels[GIMBAL_PITCH_CHANNEL]= msg.CHAN_RELEASE; //constrain(GIMBAL_TILT_MAX,GIMBAL_TILT_MIN,GIMBAL_TILT_MAX);
        msg.channels[GIMBAL_YAW_CHANNEL]= msg.CHAN_RELEASE; //constrain(GIMBAL_ROLL_TRIM,GIMBAL_ROLL_MIN,GIMBAL_ROLL_MAX);

        inputChar = getchNonBlocking();   // call non-blocking input function to get keyboard inputs

        if (inputChar == ' ' || land) {
            if (!land) {
                land = true;
                cout << "EMERGENCY LAND ENGAGED" << endl;
                currentState = Land;
                cout << "Current state: Landing..." << endl;
            }
        }

        // Send channel values to FCU
        rc_pub.publish(msg);
        ros::spinOnce();
        fcuCommRate.sleep();
    }
    return 0;
}
