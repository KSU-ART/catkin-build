
#include <ros/ros.h>
#include "std_msgs/Float32.h"

bool in = false;

class foo{

public:
	ros::NodeHandle n;
    ros::Subscriber sub_altitude;
    foo(){
        // ros::Subscriber sub_altitude = n.subscribe("/IARC/currentAltitude", 1, &foo::current_altitude_cb, this);
    }
    void set_subs();
    void current_altitude_cb(const std_msgs::Float32& msg);
};

void foo::current_altitude_cb(const std_msgs::Float32& msg){
    std::cout << "value: " << msg.data << std::endl;
    in = true;
}
void foo::set_subs(){
    sub_altitude = n.subscribe("/IARC/currentAltitude", 1, &foo::current_altitude_cb, this);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "test_sub");
    // ros::NodeHandle n;
    foo bar;
    bar.set_subs();
    // ros::Subscriber sub = bar.n.subscribe("/IARC/currentAltitude", 1, &foo::current_altitude_cb, &bar);
    
    ros::spin();
    return 0;
}