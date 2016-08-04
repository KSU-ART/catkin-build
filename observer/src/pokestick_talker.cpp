#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <iostream>
#include "wiringPi.h"

#define GPIO_BUTTON_PIN 26

int main(int argc, char** argv)
{
  ros::init(argc, argv,"pokestick_talker");

  wiringPiSetup ();
  pinMode(GPIO_BUTTON_PIN, INPUT);

  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  ros::Publisher chatter_pub = n.advertise<std_msgs::Bool>("button_press", 10);

  std_msgs::Bool button_press;
  button_press.data = false;

  while (ros::ok())
  {
    if (!digitalRead(GPIO_BUTTON_PIN)) // Return True if button pressed
    {
      //ROS_INFO("Button Pressed");
      button_press.data = true;
      chatter_pub.publish(button_press);
    }
    else
    {
      if(button_press.data == true)
      {
      //ROS_INFO("Button Released");
      button_press.data = false;
      chatter_pub.publish(button_press);
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}

