#include <ros/ros.h>
class ai_navigator
{
private:
	ros::NodeHandle n_;
	ros::Subscriber curent_pose_sub, red_plate_poses_sub, green_plate_poses_sub, obstacles_sub;
	ros::Publisher retractMsg_pub, modeMsg_pub, setpoint_pub, EMERGENCY_LAND_pub, pid_XY_pub, pid_z_pub;
	enum state 
	{
		TakeOff = 0, RandomTraversal = 1,
		InteractWithRobot = 2, AvoidObstacle = 3, Land = 4
	};
	
public:
	ai_navigator();
		

};
