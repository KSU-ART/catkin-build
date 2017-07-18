#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
// #include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include <math.h>

ros::Publisher vis_pub;
visualization_msgs::Marker mark;

float vecX, vecY;

void get_x_cb(const std_msgs::Float32::ConstPtr& msg){
    vecX = -msg->data;
}
void get_y_cb(const std_msgs::Float32::ConstPtr& msg){
    vecY = msg->data;
}

visualization_msgs::Marker create_marker(double x, double y){
    double unit = sqrt(x*x + y*y);
    double unitX, unitY;
    double w = 0;
    if (unit == 0){
        w = 1;
        unitX = 0;
        unitY = 0;
    }
    else{
        unitX = x/unit;
        unitY = y/unit;
    }

    visualization_msgs::Marker marker;
    marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = unitX;
    marker.pose.orientation.y = unitY;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = w;
    marker.scale.x = unit;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    return marker;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "rviz_visualizer");
    ros::NodeHandle n;
    vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    ros::Subscriber subX = n.subscribe("/IARC/Obstacle/RollPID", 1000, get_x_cb);
    ros::Subscriber subY = n.subscribe("/IARC/Obstacle/PitchPID", 1000, get_y_cb);

    vecX = 0;
    vecY = 0;

    ros::Rate rate(60);
    while(ros::ok()){
        mark = create_marker(vecX, vecY);

        vis_pub.publish(mark);

        ros::spinOnce();
        rate.sleep();
    }

}
