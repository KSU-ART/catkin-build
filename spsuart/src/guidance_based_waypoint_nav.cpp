#include "spsuart/autonomous.h"
#include <termios.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "nav_msgs/Path.h"
#include <cmath>

using namespace std;
using namespace cv;

namespace enc = sensor_msgs::image_encodings;

int throttle = LOW_PWM;
int roll = MID_PWM;
int pitch = MID_PWM;
int mode = ALT_HOLD_MODE;
int retract = HIGH_PWM;

bool land = false;
double target_altitude = 1.5;
double ground_distance;

geometry_msgs::PointStamped pos_est;
geometry_msgs::Vector3Stamped prev_vel;
nav_msgs::Path nav_path;
int current_goal = 0;

ros::Publisher pos_est_pub;

// Instantiate PID controllers
PIDController* xPosCtrl = new PIDController(100,0,0,-250,250);
PIDController* yPosCtrl = new PIDController(100,0,0,-250,250);
PIDController* altPosCtrl = new PIDController(250,0,0,-500,500);

mavlink_message_t* msgt = NULL;
__mavlink_rangefinder_t* x = NULL;

void splitScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    double avg = 0;
    for(int i = 0; i < sizeof(msg->ranges)/4; i++) {
        if(msg->ranges[i] < 4) {
            avg += (double)(msg->ranges[i]/(sizeof(msg->ranges)/4));
        }
        else {
            avg += (double)(4/(sizeof(4)));
        }
    }
    ground_distance = avg;
    ROS_DEBUG("Altitude: %d", avg);
    double out = altPosCtrl->calc(ground_distance);
    ROS_DEBUG("PID Out: %d", out);
    throttle = MID_PWM + out;

    if(ground_distance > 1.1) {
        retract = LOW_PWM;
    }
    else if (ground_distance < 0.9) {
        retract = HIGH_PWM;
    }
}

void guidanceVelocityCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
	double vel_x = msg->vector.x;
	double vel_y = msg->vector.y;

	ros::Time current_time = msg->header.stamp;
	
    updateWayPoints();

	pos_est.point.x += (vel_x + prev_vel.vector.x)/2*(current_time.toSec() - pos_est.header.stamp.toSec());
	pos_est.point.y += (vel_y + prev_vel.vector.y)/2*(current_time.toSec() - pos_est.header.stamp.toSec());
	pos_est.header.seq++;
	pos_est.header.stamp = current_time;
	pos_est_pub.publish(pos_est);

    double x_out = xPosCtrl->calc(pos_est.point.x);
    double y_out = yPosCtrl->calc(pos_est.point.y);
    
    cout << "goal: " << nav_path.poses[current_goal] << endl;
    cout << "est: " << pos_est << endl;
    cout << "x-corr: " << x_out << ", y-corr: " << y_out << endl;
    
    roll = MID_PWM + y_out;
    pitch = MID_PWM - x_out;
    
    prev_vel = *msg;
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

void updateWayPoints(){
    // Check to see if there are more goal poses
    if(current_goal+1 < nav_path.poses.size){
        // Check to see if the current pos is within a reasonable range of the goal
        if(abs(pos_est.point.x - nav_path.poses[current_goal].point.x) < .1 && abs(pos_est.point.y - nav_path.poses[current_goal].point.y) < .1){
            // Move on to the next goal and update set points 
            ++current_goal;
            xPosCtrl->targetSetpoint(nav_path.poses[current_goal].point.x);
            yPosCtrl->targetSetpoint(nav_path.poses[current_goal].point.y);
        }
    }
}

void defineWayPoints(){
    
    nav_path.header.seq = 0;
    nav_path.header.stamp = ros::Time::now();
    nav_path.header.frame_id = "global_frame";

    // Clear waypoints
    nav_path.poses.clear();

    // Add waypoints to path
    for (int i = 0; i < 1; i++)
    {
        geometry_msgs::PointStamped goal_pos;
        goal_pos.header.seq = i;
        goal_pos.header.stamp = ros::Time::now();
        goal_pos.header.frame_id = "global_frame";
        goal_pos.point.x = 0;
        goal_pos.point.y = 0;
        goal_pos.point.z = 0;
        nav_path.poses.push_back(goal_pos);
    }
}

int main(int argc, char **argv)
{
    //ROS node init and NodeHandle init
    ros::init(argc, argv, "of_loiter_guidance_test");
    ros::NodeHandle n;

    //Image and mavlink message subscriber
    ros::Subscriber subGuidanceVelocity = n.subscribe("/guidance/velocity",1,guidanceVelocityCallback);
    ros::Subscriber subHokuyo = n.subscribe("scan3", 1, splitScanCallback);

    //Mavros rc override publisher
    ros::Publisher rc_pub = n.advertise<mavros::OverrideRCIn>("/mavros/rc/override", 1);
    pos_est_pub = n.advertise<geometry_msgs::PointStamped>("/fatcat/pos_est", 1);

    //RC msg container that will be sent to the FC @ fcuCommRate hz
    mavros::OverrideRCIn msg;
    ros::Rate fcuCommRate(45); // emulating speed of dx9 controller

    //Define navigation waypoints
    defineWayPoints();

	// Init xy pose
	pos_est.header.seq = 0;
	pos_est.header.stamp = ros::Time::now();
	pos_est.header.frame_id = "global_frame";
	pos_est.point.x = 0;
	pos_est.point.y = 0;
	pos_est.point.z = 0;

	// Init PID controllers
    altPosCtrl->on();
    xPosCtrl->on();
    yPosCtrl->on();

    // Set PID controller targets
    xPosCtrl->targetSetpoint(0); // target X coordinate in pixels
    yPosCtrl->targetSetpoint(0); // target Y coordinate in pixels
    altPosCtrl->targetSetpoint(target_altitude); // target altitude in meters
    int inputChar = 'a';

    //While node is alive send RC values to the FC @ fcuCommRate hz
    while(ros::ok()) {
        if(ground_distance > 1.0) {
            msg.channels[ROLL_CHANNEL]=roll;
            msg.channels[PITCH_CHANNEL]=pitch;
        }
        else {
            msg.channels[ROLL_CHANNEL]=msg.CHAN_RELEASE;
            msg.channels[PITCH_CHANNEL]=msg.CHAN_RELEASE;
        }

        msg.channels[THROTTLE_CHANNEL]=throttle;
        msg.channels[YAW_CHANNEL]=msg.CHAN_RELEASE;
        msg.channels[MODE_CHANNEL]=mode;
        msg.channels[RETRACT_CHANNEL]=retract;
        msg.channels[GIMBAL_PITCH_CHANNEL]=msg.CHAN_RELEASE;
        msg.channels[GIMBAL_YAW_CHANNEL]=msg.CHAN_RELEASE;

        inputChar = getchNonBlocking();   // call non-blocking input function

        if(inputChar == ' ' || land) {
            if(!land) {
                land= true;
                cout<<"EMERGENCY LAND ENGAGED"<<endl;
            }

            msg.channels[ROLL_CHANNEL]=msg.CHAN_RELEASE;
            msg.channels[PITCH_CHANNEL]=msg.CHAN_RELEASE;
            msg.channels[THROTTLE_CHANNEL]=MID_PWM;
            msg.channels[MODE_CHANNEL]=LAND_MODE;
            msg.channels[RETRACT_CHANNEL]=HIGH_PWM;

        }

        rc_pub.publish(msg);
        ros::spinOnce();
        fcuCommRate.sleep();

    }
    return 0;
}
