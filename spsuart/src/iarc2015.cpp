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
PIDController* xVelCtrl = new PIDController(50, 0, 0, -100, 100);
PIDController* yVelCtrl = new PIDController(50, 0, 0, -100, 100);
PIDController* altPosCtrl = new PIDController(500, 0, 0, -200, 300);

enum class State { TakeOff, FlyFoward, FindRobots, InteractWithRobot, AvoidObstacle, Land };

State currentState = TakeOff;

/* Lidar based alt-hold callback */
void splitScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
	double avg = 0;
	for (int i = 0; i < sizeof(msg->ranges) / 4; i++) {
		if (msg->ranges[i] < 4){
			avg += (double)(msg->ranges[i] / (sizeof(msg->ranges) / 4));
		}
		else{
			avg += (double)(4 / (sizeof(4)));
		}
	}
	ground_distance = avg;
	ROS_DEBUG("Altitude: %d", avg);
	double out = altPosCtrl->calc(ground_distance);
	ROS_DEBUG("PID Out: %d", out);
	throttle = MID_PWM + out;

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

int main(int argc, char **argv)
{
	//ROS node init and NodeHandle init
	ros::init(argc, argv, "iarc2015");
	ros::NodeHandle n;

	//Image point subscriber
	ros::Subscriber subPoint = n.subscribe("opencv/image_point", 1, imagePointCallback);
	
	//Obstacle detection subscriber
	ros::Subscriber subObstacle = n.subscribe("spsuart/obstacle_detected", 1, obstacleDetectedCallback);

	ros::Subscriber subHokuyo = n.subscribe("scan3", 1, splitScanCallback);
	//Mavros rc override publisher
	ros::Publisher rc_pub = n.advertise<mavros::OverrideRCIn>("/mavros/rc/override", 1);

	//RC msg container that will be sent to the FC @ fcuCommRate hz
	mavros::OverrideRCIn msg;
	ros::Rate fcuCommRate(45); // emulating speed of dx9 controller

	// Set PID controller targets  
	xVelCtrl->targetSetpoint(0); // target X coordinate in pixels
	yVelCtrl->targetSetpoint(0); // target Y coordinate in pixels
	altPosCtrl->targetSetpoint(target_altitude); // target altitude in meters
	int inputChar = 'a';

	bool land = false;

	//While node is alive send RC values to the FC @ fcuCommRate hz
	while (ros::ok()){

		msg.channels[YAW_CHANNEL] = msg.CHAN_RELEASE;
		msg.channels[NOT_USED_CHANNEL] = msg.CHAN_RELEASE;
		msg.channels[GIMBAL_TILT_CHANNEL] = constrain(GIMBAL_TILT_MAX, GIMBAL_TILT_MIN, GIMBAL_TILT_MAX);
		msg.channels[GIMBAL_PITCH_CHANNEL] = constrain(GIMBAL_ROLL_TRIM, GIMBAL_ROLL_MIN, GIMBAL_ROLL_MAX);
		
		switch (currentState){
		case TakeOff:
			cout << "Current state: Take off" << endl;
			msg.channels[ROLL_CHANNEL] = msg.CHAN_RELEASE;
			msg.channels[PITCH_CHANNEL] = msg.CHAN_RELEASE;
			msg.channels[THROTTLE_CHANNEL] = throttle;
			msg.channels[MODE_CHANNEL] = ALT_HOLD_MODE;
			break;
		case FlyFoward:
			cout << "Current state: Fly forward" << endl;
			msg.channels[ROLL_CHANNEL] = msg.CHAN_RELEASE;
			msg.channels[PITCH_CHANNEL] = pitch;
			msg.channels[THROTTLE_CHANNEL] = throttle;
			msg.channels[MODE_CHANNEL] = ALT_HOLD_MODE;
			break;
		case FindRobots:
			cout << "Current state: Searching for ground robots" << endl;
			msg.channels[ROLL_CHANNEL] = msg.CHAN_RELEASE;
			msg.channels[PITCH_CHANNEL] = msg.CHAN_RELEASE;
			msg.channels[THROTTLE_CHANNEL] = throttle;
			msg.channels[MODE_CHANNEL] = ALT_HOLD_MODE;
			break;
		case InteractWithRobot:
			cout << "Current state: Engaging a ground robot" << endl;
			msg.channels[ROLL_CHANNEL] = roll;
			msg.channels[PITCH_CHANNEL] = pitch;
			msg.channels[THROTTLE_CHANNEL] = throttle;
			msg.channels[MODE_CHANNEL] = ALT_HOLD_MODE;
			break;
		case AvoidObstacle:
			cout << "Current state: AVODING OBSTACLE!!" << endl;
			msg.channels[ROLL_CHANNEL] = msg.CHAN_RELEASE;
			msg.channels[PITCH_CHANNEL] = msg.CHAN_RELEASE;
			msg.channels[THROTTLE_CHANNEL] = throttle;
			msg.channels[MODE_CHANNEL] = ALT_HOLD_MODE;
			break;
		case Land:
			case << "Current state: Landing..." << endl;
			msg.channels[ROLL_CHANNEL] = msg.CHAN_RELEASE;
			msg.channels[PITCH_CHANNEL] = msg.CHAN_RELEASE;
			msg.channels[THROTTLE_CHANNEL] = MID_PWM;
			msg.channels[MODE_CHANNEL] = LAND_MODE;
			break;
		}
		
		inputChar = getchNonBlocking();   // call non-blocking input function to get keyboard inputs

		if (inputChar == ' ' || land){
			if (!land){
				land = true;
				cout << "EMERGENCY LAND ENGAGED" << endl;
				currentState = Land;
			}
		}

		// Send channel values to FCU
		rc_pub.publish(msg);
		ros::spinOnce();
		fcuCommRate.sleep();
	}
	return 0;
}
