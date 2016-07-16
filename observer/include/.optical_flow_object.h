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
#define PYRAMID_LEVELS		4
#define WINDOW_SIZE			7
class optical_flow
{
private:
	bool verbose;
	
	ros::NodeHandle n_h_;
	ros::Subscriber cam, orientation;
	ros::Publisher pose_pub;
	
	projection_::cameraModel c1;
	
	Quaternion cur_orientation;
	
	double cur_time;
	double prev_time;
	
	cvg_int maxNumFeatures;
	
	cvg_bool firstRun;
	
	Vector3 globalDisplacement;
	
public:
	optical_flow(std::string camID);
	~optical_flow();
	void image_cb(const sensor_msgs::ImageConstPtr& msg);
	cvg_bool process(cv::Mat *image, cvg_int *prevFrameNumFeatures, cv::Point2f **prevFrameFeatures, char **foundFeaturesInCurrentFrame, cv::Point2f **currentFrameFeatures);	
	void calc_displacements();
	void remove_outliers();
	void merge_and_publish();
	void orientation_cb(const geometry_msgs::Quaternion msg);
	
}; //end gridflow
