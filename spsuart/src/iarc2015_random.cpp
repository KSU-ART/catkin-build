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
int centerRadius = 60;

double target_altitude = 1.5;

double ground_distance;

// Instantiate PID controllers
PIDController* xPosCtrl = new PIDController(1.5, 0, 0, -100, 100);
PIDController* yPosCtrl = new PIDController(1.5, 0, 0, -100, 100);
PIDController* altPosCtrl = new PIDController(700, 0, 0, -200, 500);

enum State {
	TakeOff = 0, EnterArena = 1, RandomTraversal = 2,
	InteractWithRobot = 3, AvoidObstacle = 4, Land = 5
};

State currentState = TakeOff;

ros::Time interactWithRobotStartTime = ros::Time();
ros::Time interactWithRobotCoolDown = ros::Time();
bool interactWithRobotTimeStarted = false;

void imagePointCallback(const ros_opencv::TrackingPoint::ConstPtr& msg) {

	if (currentState == RandomTraversal && msg->pointX != -1 && msg->pointY != -1 && interactWithRobotCoolDown < ros::Time::now()){
		currentState = InteractWithRobot;
		interactWithRobotTimeStarted = false;
		cout << "Current state: Engaging a ground robot" << endl;
	}
	// If we lose sight of the ground robot go back to random traversal state
	else if (currentState == InteractWithRobot && msg->pointX == -1 && msg->pointY == -1){
		currentState = RandomTraversal;
		cout << "Current state: Random Traversal" << endl;
	}
	else if (currentState == InteractWithRobot){
		roll = MID_PWM + xPosCtrl->calc(msg->pointX);
		pitch = MID_PWM - xPosCtrl->calc(msg->pointY);
		
		
		
		if (msg->pointX > (320 - centerRadius) && msg->pointX < (320 + centerRadius) && msg->pointY > (240 - centerRadius) && msg->pointY < (240 + centerRadius)){
			//If the ground robot is centered try to hover over it
			altPosCtrl->targetSetpoint(0.5);
			if (!interactWithRobotTimeStarted){
				interactWithRobotStartTime = ros::Time::now();
				interactWithRobotTimeStarted = true;
			}
			//If we have interacted with the ground robot for more than 15 seconds go back to random traversal
			else if (ros::Time::now() >= interactWithRobotStartTime + ros::Duration(15)){
				currentState = RandomTraversal;
				interactWithRobotCoolDown = ros::Time::now() + ros::Duration(10);
				cout << "Current state: Random Traversal" << endl;
			}
		}
		else{
			altPosCtrl->targetSetpoint(1.5);
			interactWithRobotTimeStarted = false;
		}
	}
}

void obstacleDetectedCallback(const ros_opencv::ObstacleDetected::ConstPtr&
	msg) {
	/* If an obstacle is detected override the state to avoid*/
	if (msg->obstacleDetected && ground_distance>.4){
		currentState = AvoidObstacle;
		cout << "Current state: AVODING OBSTACLE!!" << endl;
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
	if (currentState == TakeOff && ground_distance > 1.45 && ground_distance < 1.55){
		currentState = EnterArena;
		cout << "Current state: Enter arena" << endl;
	}
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
	ros::init(argc, argv, "iarc2015_random");
	ros::NodeHandle n;

	//Image point subscriber
	ros::Subscriber subPoint = n.subscribe("image_point", 1, imagePointCallback);

	//Obstacle detection subscriber
	ros::Subscriber subObstacle = n.subscribe("spsuart/obstacle_detected", 1, obstacleDetectedCallback);

	ros::Subscriber subHokuyo = n.subscribe("scan3", 1, splitScanCallback);
	//Mavros rc override publisher
	ros::Publisher rc_pub = n.advertise<mavros::OverrideRCIn>("/mavros/rc/override", 1);

	//RC msg container that will be sent to the FC @ fcuCommRate hz
	mavros::OverrideRCIn msg;
	ros::Rate fcuCommRate(45); // emulating speed of dx9 controller

	int inputChar = 'a';

	bool land = false;

	ros::Time enterArenaStartTime = ros::Time::now();
	ros::Time randomTraversalTime = ros::Time::now();
	ros::Time obstacleAvoidStartTime = ros::Time::now();

	bool enterArenaTimerStarted = false;
	bool randomTraversalTimeStarted = false;
	bool randomTraversalWait = false;
	bool obstacleTimerStarted = false;

	// Init cooldown
	interactWithRobotCoolDown = ros::Time::now();
	cout << "Current state: Take off" <<endl;

	//While node is alive send RC values to the FC @ fcuCommRate hz
	while (ros::ok()){

		switch (currentState){
		case TakeOff:
			roll = msg.CHAN_RELEASE;
			pitch = msg.CHAN_RELEASE;
			altPosCtrl->targetSetpoint(1.5); // target altitude in meters
			mode = ALT_HOLD_MODE;
			break;
		case EnterArena:
			altPosCtrl->targetSetpoint(1.5); // target altitude in meters
			pwmVector(40, 0, &roll, &pitch);
			mode = ALT_HOLD_MODE;
			if (!enterArenaTimerStarted){
				enterArenaStartTime = ros::Time::now();
				enterArenaTimerStarted = true;
			}
			else if (ros::Time::now() >= enterArenaStartTime + ros::Duration(5.0)){
				currentState = RandomTraversal;
				enterArenaTimerStarted = false;
				cout << "Current state: Random Traversal" << endl;
			}
			break;
		case RandomTraversal:
			altPosCtrl->targetSetpoint(1.5); // target altitude in meters
			mode = ALT_HOLD_MODE;
			if (!randomTraversalTimeStarted){
				randomTraversalTime = ros::Time::now();
				randomTraversalTimeStarted = true;
				if (!randomTraversalWait){
					double angle = (rand() % 359) + 1;
					pwmVector(60, angle, &roll, &pitch);
					cout << "Angle Pitch Roll " << angle << " " << pitch << " " << roll << endl;
				}
				else{
					roll = msg.CHAN_RELEASE;
					pitch = msg.CHAN_RELEASE;
				}
			}
			else if (ros::Time::now() >= randomTraversalTime + ros::Duration(1.0)){
				currentState = RandomTraversal;
				cout << "Current state: Random Traversal" << endl;
				randomTraversalTimeStarted = false;
				randomTraversalWait = !randomTraversalWait;
			}
			break;
		case InteractWithRobot:
			xPosCtrl->targetSetpoint(320); // target X coordinate in pixels
			yPosCtrl->targetSetpoint(240); // target Y coordinate in pixels
			
			mode = ALT_HOLD_MODE;
			break;
		case AvoidObstacle:
			roll = msg.CHAN_RELEASE;
			pitch = msg.CHAN_RELEASE;
			altPosCtrl->targetSetpoint(2.5); // target altitude in meters
			mode = ALT_HOLD_MODE;
			if (!obstacleTimerStarted){
				obstacleAvoidStartTime = ros::Time::now();
				obstacleTimerStarted = true;
			}
			else if (ros::Time::now() >= obstacleAvoidStartTime + ros::Duration(10)){
				/* After avoiding an obstacle go back to random traversal state*/
				currentState = RandomTraversal;
				obstacleTimerStarted = false;
				cout << "Current state: Random Traversal" << endl;
			}
			break;
		case Land:
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
