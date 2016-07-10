#include "optical_flow_object.h"

optical_flow::optical_flow(std::string camID)
{
	///initialize vars
	cur_time = 0;
	prev_time = 0;
	
	///init cam_model
	//projection::camera_model c1(camID.c_str(1));
	
	///init subs and pubs
	std::string s1 = "/usb_cam_" + camID + "/image_rect_color";
	cam = n_h_.subscribe(s1.c_str(), 1, &optical_flow::image_cb, this);
	orientation = n_h_.subscribe("/mavros/imu/data", 1, &optical_flow::orientation_cb, this);
	pose_pub = n_h_.advertise<geometry_msgs::PoseStamped>("/gridflow/current_pose",1);
	
	///debugging options
	verbose = true;
	
}	

/***************************************************
 * image callback - main processing of image
 * ************************************************/
void optical_flow::image_cb(const sensor_msgs::ImageConstPtr& msg)
{
	///time management:
	prev_time = cur_time;
	cur_time  = ros::Time::now().toSec();
	
	///copy ros image to iamge suitable for cv processing
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	if (verbose)
	{
		cv::namedWindow( "Input Image", CV_WINDOW_AUTOSIZE ); // Create a window for display.
		cv::imshow( "Input Image", cv_ptr->image ); 	//display image
	}
	
}

void optical_flow::orientation_cb(const geometry_msgs::Quaternion msg)
{
	
}

int main(int argc, char**argv)
{	
	ros::init(argc, argv, "optical_flow/optical_flow_node");
	optical_flow g1("1");
	ros::spin(); //use 1 thread
}
