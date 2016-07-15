#include "optical_flow_object.h"

optical_flow::optical_flow(std::string camID)
{
	///initialize vars
	
	
	///init cam_model
	c1.loadModel('0');
	
	///init subs and pubs
	std::string s1 = "/usb_cam_" + camID + "/image_rect_color";
	cam = n_h_.subscribe(s1.c_str(), 1, &optical_flow::image_cb, this);
	orientation = n_h_.subscribe("/mavros/imu/data", 1, &optical_flow::orientation_cb, this);
	pose_pub = n_h_.advertise<geometry_msgs::PoseStamped>("/gridflow/current_pose",1);
	
	///debugging options
	verbose = true;
	
}	

optical_flow::~optical_flow()
{
	
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
cvg_bool optical_flow::process(cv::Mat *image, cvg_int *prevFrameNumFeatures, cv::Point2f **prevFrameFeatures, char **foundFeaturesInCurrentFrame, cv::Point2f **currentFrameFeatures)
{
	cv::InputArray currentFrame;
	cv::InputArray lastFrame;
	cv::InputArray eigImage, tempImage;
	cv::OutputArray pyramid1, pyramid2;
	int lastFrameFeaturesCount;
	cv::Point2f *lastFrameFeatures, *newFeatures, *lastFrameFeaturesCopy;
	char *foundFeatures;
	float *errorFeatures;
	cvg_bool res = !firstRun;
	if (!firstRun) 
	{
		// Find displacements of the features
		cv::Size opticalFlowWindow = cvSize(WINDOW_SIZE,  WINDOW_SIZE);
		cv::TermCriteria terminationCriteria = cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.3);
		cv::calcOpticalFlowPyrLK(	lastFrame, currentFrame, pyramid1, pyramid2,
								lastFrameFeatures, newFeatures, lastFrameFeaturesCount,
								opticalFlowWindow, PYRAMID_LEVELS, foundFeatures, errorFeatures,
								terminationCriteria, 0
								);

		if (prevFrameNumFeatures != NULL) (*prevFrameNumFeatures) = lastFrameFeaturesCount;
		if (prevFrameFeatures != NULL) 
		{
			memcpy(lastFrameFeaturesCopy, lastFrameFeatures, lastFrameFeaturesCount * sizeof(cv::Point2f));
			(*prevFrameFeatures) = lastFrameFeaturesCopy;
		}
		if (foundFeaturesInCurrentFrame != NULL) (*foundFeaturesInCurrentFrame) = foundFeatures;
		if (currentFrameFeatures != NULL) (*currentFrameFeatures) = newFeatures;

	} else firstRun = false;

	lastFrameFeaturesCount = maxNumFeatures;
	// Find good features to track in the current frame
	cv::goodFeaturesToTrack(currentFrame, eigImage, tempImage, lastFrameFeatures, &lastFrameFeaturesCount, 0.01, 0.01, NULL);

	// Store current frame for the next iteration
	cvCopy(currentFrame, lastFrame);

	return res;
}
void optical_flow::orientation_cb(const geometry_msgs::Quaternion msg)
{
	
}

int main(int argc, char**argv)
{	
	ros::init(argc, argv, "optical_flow/optical_flow_node");
	optical_flow of1("1");
	ros::spin(); //use 1 thread
}
