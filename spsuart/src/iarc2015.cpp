#include "spsuart/autonomous.h"
#include "px_comm/OpticalFlow.h"
#include <termios.h>
#include "sensor_msgs/LaserScan.h"
#include "ros_opencv/TrackingPoint.h"
#include "ros_opencv/ObstacleDetected.h"
#include "math.h"

using namespace std;
using namespace cv;

namespace enc = sensor_msgs::image_encodings;

int throttle = LOW_PWM;
int roll = MID_PWM;
int pitch = MID_PWM;
int mode = ALT_HOLD_MODE;

double target_altitude = 1.5;

double ground_distance;

// Instantiate PID controllers
PIDController* xVelCtrl = new PIDController(50, 0, 0, -100, 100);
PIDController* yVelCtrl = new PIDController(50, 0, 0, -100, 100);
PIDController* altPosCtrl = new PIDController(500, 0, 0, -200, 300);

enum State { TakeOff = 0, EnterArena = 1, RandomTraversal = 2,
InteractWithRobot = 3, AvoidObstacle = 4, Land = 5 };

State currentState = TakeOff;
bool enterArenaTimerStarted = false;

void imagePointCallback(const ros_opencv::TrackingPoint::ConstPtr& msg) {
	/*Only use this callback if the vehicle is in the InteractWithRobot state*/
	if (currentState == InteractWithRobot){

	}
	else
		return;
}

void obstacleDetectedCallback(const ros_opencv::ObstacleDetected::ConstPtr&
msg) {
	/* If an obstacle is detected override the state to avoid*/
	if (msg->obstacleDetected){
		currentState = AvoidObstacle;
	}
} 

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
	/* If the vehicle is in the takeoff state and has settled switch to enter arena */
	if (currentState == Takeoff && altitude > 1.45 && altitude < 1.55){
		currentState = EnterArena;
	}

}

/* */
void enterArenaTimerCallback(const ros::TimerEvent&){
	currentState = RandomTraversal;
	enterArenaTimerStarted = false;
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

void pwmVector(int mag, double theta, int*  xVar, int*  &yVar) {
	if (mag > 500) {
		mag = 500;
	}
	else if (mag < -500) {
		mag = -500;
	}
	*xVar = int(MID_PWM + mag * sin(theta));
	*yVar = int(MID_PWM + mag * cos(theta));
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
	int inputChar = 'a';

	bool land = false;
	

	//While node is alive send RC values to the FC @ fcuCommRate hz
	while (ros::ok()){
		
		switch (currentState){
		case TakeOff:
			cout << "Current state: Take off" << endl;
			roll = msg.CHAN_RELEASE;
			pitch = msg.CHAN_RELEASE;
			altPosCtrl->targetSetpoint(1.5); // target altitude in meters
			mode = ALT_HOLD_MODE;
			break;
		case EnterArena:
			cout << "Current state: Enter arena" << endl;
			altPosCtrl->targetSetpoint(1.5); // target altitude in meters
			pwmVector(30, 0, roll&, pitch&);
			cout << "Pitch: " << pitch << endl;
			mode = ALT_HOLD_MODE;
			if (!enterArenaTimerStarted){
				ros::Timer timer = nh.createTimer(ros::Duration(5), enterArenaTimerCallback, true);
				enterArenaTimerStarted = true;
			}
			break;
		case RandomTraversal:
			cout << "Current state: Random Traversal" << endl;
			roll = msg.CHAN_RELEASE;
			pitch = msg.CHAN_RELEASE;
			altPosCtrl->targetSetpoint(1.5); // target altitude in meters
			mode = ALT_HOLD_MODE;
			break;
		case InteractWithRobot:
			cout << "Current state: Engaging a ground robot" << endl;
			altPosCtrl->targetSetpoint(target_altitude); // target altitude in meters
			mode = ALT_HOLD_MODE;
			break;
		case AvoidObstacle:
			cout << "Current state: AVODING OBSTACLE!!" << endl;
			roll = msg.CHAN_RELEASE;
			pitch = msg.CHAN_RELEASE;
			altPosCtrl->targetSetpoint(2.5); // target altitude in meters
			mode = ALT_HOLD_MODE;
			break;
		case Land:
			cout << "Current state: Landing..." << endl;
			roll = msg.CHAN_RELEASE;
			pitch = msg.CHAN_RELEASE;
			throttle = MID_PWM;
			mode = LAND_MODE;
			break;
		}

		msg.channels[ROLL_CHANNEL] = roll;
		msg.channels[PITCH_CHANNEL] = pitch;
		msg.channels[THROTTLE_CHANNEL] = throttle;
		msg.channels[MODE_CHANNEL] = mode;
		msg.channels[YAW_CHANNEL] = msg.CHAN_RELEASE;
		msg.channels[NOT_USED_CHANNEL] = msg.CHAN_RELEASE;
		msg.channels[GIMBAL_TILT_CHANNEL] = constrain(GIMBAL_TILT_MAX, GIMBAL_TILT_MIN, GIMBAL_TILT_MAX);
		msg.channels[GIMBAL_ROLL_CHANNEL] = constrain(GIMBAL_ROLL_TRIM, GIMBAL_ROLL_MIN, GIMBAL_ROLL_MAX);
		
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
