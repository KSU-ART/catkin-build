#include "ros/ros.h"
#include "roscopter/VFR_HUD.h"

using namespace std;
void callback(const roscopter::VFR_HUD::ConstPtr& msg)
{
cout<<"alt: "<<msg->alt<<endl;
}

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "test_mavlink_feedback_node");

  
  ros::NodeHandle n;

 
  ros::Subscriber sub = n.subscribe("/vfr_hud", 1, callback);
  ros::spin();
  return 0;
}


