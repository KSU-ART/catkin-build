#include <vector>
#include <string>	
#include <ros/ros.h>
#include <camera_model.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <highgui.h>
#define max_displacement 20

class optical_flow
{
private:
	bool verbose;
	ros::NodeHandle n_h_;
	ros::Subscriber cam, orientation;
	ros::Publisher pose_pub;
	Quaternion cur_orientation;
	Quaternion prev_orientation;
	double cur_time;
	double prev_time;
	std::vector<cv::Point> old_features_world;
	std::vector<cv::Point> new_features_world;
	std::vector<double> displacements;
	double cur_velocity[2];
	double net_displacement[2];
	
public:
	optical_flow(std::string camID);
	void image_cb(const sensor_msgs::ImageConstPtr& msg);
	void find_grid_corners();
	void calc_displacements();
	void remove_outliers();
	void orientation_cb(const geometry_msgs::Quaternion msg);
	
}; //end gridflow
