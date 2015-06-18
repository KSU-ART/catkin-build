#include "ros/ros.h"
#include <mavros/OverrideRCIn.h>
#include <mavros/Mavlink.h>
#include <mavros/Waypoint.h>
#include "mavlink/v1.0/ardupilotmega/mavlink.h"
#include "px_comm/OpticalFlow.h"
#include "ros_opencv/TrackingPoint.h"
#include "sensor_msgs/NavSatFix.h"
#include "iostream"
#include <spsuart/PosEst.h>
#include "spsuart/pid-controller/PIDController.h"
#include "spsuart/iir/Iir.h"

using namespace std;

PIDController* altPosCtrl = new PIDController(500,0,0,-300,300);

Iir::Butterworth::LowPass<3> posXFilt;
Iir::Butterworth::LowPass<3> posYFilt;

//Lidar hack references
mavlink_message_t* msgt = NULL;
__mavlink_rangefinder_t* x = NULL;
double pos_x;
double pos_y;
double prevTime;
const double Meter_To_Latitude = 0.00000904366736689;
const double Meter_To_Longitude = 0.00000898451471479;

//Lidar hack : The product of pair programming
void apmMavlinkmsgCallback(const mavros::Mavlink::ConstPtr& msg){
        if(msg->msgid==173){
                if(msgt == NULL)
                        msgt = new mavlink_message_t();
                if(x == NULL)
                        x = new __mavlink_rangefinder_t();
                msgt->seq = msg->seq;
                msgt->len = msg->len;
                msgt->sysid = msg->sysid;
                msgt->compid = msg->compid;
                msgt->msgid = msg->msgid;
                msgt->payload64[0] = msg->payload64[0];
                mavlink_msg_rangefinder_decode(msgt, x); 
                
                // increase (or decrease?) altitude based on value from pid
                cout<<"Altitude :"<<x->distance<<endl;                
        double out = altPosCtrl->calc(double(x->distance));

        delete msgt;
                delete x;
                msgt = NULL;
                x = NULL;
        }
}

void imagePointCallback(const ros_opencv::TrackingPoint::ConstPtr& msg)
{
	
}

void optFlowCallback(const px_comm::OpticalFlow::ConstPtr& msg) 
{
   double currentTimeNano = ros::Time::now().toNSec();
   double deltaTime = currentTimeNano - prevTime;
   
   pos_x += posXFilt.filter(msg->velocity_x * deltaTime);
   pos_y += posYFilt.filter(msg->velocity_y * deltaTime);
   
   prevTime = ros::Time::now().toNSec();
}
  
  
  void translatePosToGPS(spsuart::PosEst &pos, sensor_msgs::NavSatFix &gps){
	gps.latitude = pos.x * Meter_To_Latitude;
	gps.longitude = pos.y * Meter_To_Longitude;
	
  } 
  
  void generateWaypoint(spsuart::PosEst &destPos, mavros::Waypoint &wp){
	wp.x_lat = destPos.x * Meter_To_Latitude;
	wp.y_long = destPos.y * Meter_To_Longitude;
	
  }

int main(int argc, char **argv)
{
    //ROS node init and NodeHandle init
    ros::init(argc, argv, "gps_interface_node");
    ros::NodeHandle n;
    
    //Init prev time for velocity calc
    prevTime = ros::Time::now().toNSec();
    
    //Init positions at (0,0)
    pos_x = 0;
    pos_y = 0;

    posXFilt.setup(3, 300, 50);
    posYFilt.setup(3, 300, 50);
    
    posXFilt.reset();
    posYFilt.reset();

    //Image and mavlink message subscriber
    ros::Subscriber subpoint = n.subscribe("image_point",1,imagePointCallback);
    ros::Subscriber subflow = n.subscribe("px4flow/OpticalFlow",1,optFlowCallback);
    
      //GPS publisher
      ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>("/gps/fix", 1);
      ros::Publisher pos_est = n.advertise<spsuart::PosEst>("pos_estimate", 1);
      ros::Publisher target_wp = n.advertise<mavros::Waypoint>("go_to", 1);
  
      //GPA msg container that will be sent to the FC
      //This represents our fake gps point at any point in time
      spsuart::PosEst prevPos;
      spsuart::PosEst currPos;
      spsuart::PosEst nextPos;
      //Default the goal pos to be 5,5
      nextPos.x = 5;
      nextPos.y = 5;
      sensor_msgs::NavSatFix prevGps;
      sensor_msgs::NavSatFix currGps;
      mavros::Waypoint nextWp;
      
      
      ros::Rate fcuCommRate(300); 
    
    //While node is alive send RC values to the FC @ fcuCommRate hz
      while(ros::ok())
      {
		  prevPos = currPos;
		  currPos.x=pos_x;
	      currPos.y=pos_y;
	      //publish position estimate in meters
	      pos_est.publish(currPos);
	      
	      //translate position estimate to gps point
	      translatePosToGPS(currPos, currGps);
	      //publish gps to Flight computer
	      gps_pub.publish(currGps);
	      
	      generateWaypoint(nextPos, nextWp);
	      //publish wp
	      target_wp.publish(nextWp);
	      
	      
          ros::spinOnce();
          fcuCommRate.sleep();
      }
    return 0;
}
