#include "ros/ros.h"
#include "mavros/SetMode.h"
#include "mavros/CommandBool.h"
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node");
  ros::NodeHandle nh;

  ros::Publisher mavros_control_pub =
nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",1000);
  geometry_msgs::PoseStamped fmu_controller_setpoint;

  ros::ServiceClient mavros_set_mode_client = nh.serviceClient<mavros::SetMode>("mavros/set_mode");
  mavros::SetMode set_mode;
  set_mode.request.custom_mode = "OFFBOARD";
  
  ros::ServiceClient mavros_nav_guided_client = nh.serviceClient<mavros::CommandBool>("/mavros/cmd/guided_enable");
  mavros::CommandBool nav_guided;
  nav_guided.request.value = true;
  
  bool offboard_commands_enabled = false;
  bool nav_guided_enabled = false;
  
  ros::Rate loop_rate(100.0);

  while(ros::ok())
  {
    if (!offboard_commands_enabled) {
      if (mavros_set_mode_client.call(set_mode))
      {
        ROS_INFO("Set mode: OFFBOARD enabled!");
		offboard_commands_enabled = true;
      }
      else
      {
        ROS_INFO("Offboard mode still not enabled!");
      }
    }

	fmu_controller_setpoint.pose.position.x = 5;
	fmu_controller_setpoint.pose.position.y = 5;
	fmu_controller_setpoint.pose.position.z = 2;

    mavros_control_pub.publish(fmu_controller_setpoint);

    if(!nav_guided_enabled)
    {
      if (mavros_nav_guided_client.call(nav_guided))
      {
		ROS_INFO("Nav guided: OFFBOARD enabled!");
		nav_guided_enabled = true;
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

}
