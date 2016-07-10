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
	
	///process image into edge binary using canny
	cv::Mat edges;
	cv::Canny(cv_ptr->image, edges, 100, 200);
	if (verbose)
	{
		cv::namedWindow( "Canny", CV_WINDOW_AUTOSIZE ); // Create a window for display.
		cv::imshow( "Canny", edges ); 	//display image
	}
	
	
	
	/*********************************************
	///find shape edges from grid
	* ********************************************/
	std::vector<cv::Point> detected_points;
	cv::Mat contour_ends;
	if (verbose)
	{
		contour_ends = edges;
	}
	//find contours (essentially distinct shapes within an image):
	std::vector<std::vector<cv::Point> > contours_vec;
	cv::findContours
	(
		edges, //destroys edges image...
		contours_vec, 
		CV_RETR_LIST,//reconstruct full heirarchy of nested contours
		CV_CHAIN_APPROX_SIMPLE //compresses horizontal, vertical, and 
					//   diagonal segments to leave only their endpoints
	);
	//get approximate polygonal curbs from all contours:
	//array containing the approximation endpoints:
	std::vector<cv::Point> approx;	
	//loop through all the contours:
	for (int i = 0; i < contours_vec.size(); i++)
	{
		//approximate edge points with accuracy proportional
		//	to the contour perimeter
		cv::approxPolyDP
		(
			cv::Mat (contours_vec[i]),
			approx,
			cv::arcLength(cv::Mat(contours_vec[i]), true) * 0.02,
			true
		);
		//skip small or many-sided shapes
		if (std::fabs( cv::contourArea(contours_vec[i]) ) < 500 || 4 > (approx.size() > 15))
			continue;
/*		
		//ckeck for parallel lines:
		int paralellity;
		for (int k = 0; k < approx.size(); k++)
		{
			if (approx[k]
*/		
		for (int k = 0; k < approx.size(); k++)
			detected_points.push_back(approx[k]);
	}	
	if (verbose) //output detected intersections
	{
		for (int i = 0; i < detected_points.size(); i++)
		{
			circle( contour_ends, detected_points[i], 30,  cv::Scalar(0), 2, 8, 0 );
		}
		cv::namedWindow( "contour_ends", CV_WINDOW_AUTOSIZE ); // Create a window for display.
		cv::imshow( "contour_ends", contour_ends ); //display image
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
