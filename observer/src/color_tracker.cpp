/*******************************************************************
 * This program takes a camera feed and publishes:
 *  an image showing objects detected in the set color range, 
 *  binary images filtered for color,
 *  and arrays with xy coordinates of the center of detected objects.
 * There is also a calibrationMode for finding desired color range.
********************************************************************/
#include "color_tracker.h"
#include "camera_model.h"
namespace enc = sensor_msgs::image_encodings;

trackobjects::trackobjects()
: it_(nh_)
{
	angler = true;
	MIN_OBJECT_AREA = 50*50;
	MAX_NUM_OBJECTS=10;
	image_sub_ = it_.subscribe("/usb_cam_0/image_rect_color", 1, &trackobjects::track, this);
	ROS_INFO("Color tracker subscribed to camera  /usb_cam_2/image_rect_color");
    image_pub1_=it_.advertise("/observer/red_binary",1);
	image_pub2_=it_.advertise("/observer/green_binary",1);
	red_pub = a.advertise<std_msgs::Int32MultiArray>("/observer/r_cam_points_0", 1);
	green_pub = a.advertise<std_msgs::Int32MultiArray>("/observer/g_cam_points_0", 1);
}
trackobjects::trackobjects(std::string camID)
: it_(nh_)
{
	angler = false;
	MIN_OBJECT_AREA = 10*10;
	MAX_NUM_OBJECTS=15;
	string topicName = "/usb_cam_" + camID + "/image_rect_color";
	image_sub_ = it_.subscribe(topicName.c_str(), 1, &trackobjects::track, this);
	ROS_INFO("Color tracker subscribed to camera %s", topicName.c_str());
	topicName = "/observer/r_cam_points_" + camID;
	red_pub = a.advertise<std_msgs::Int32MultiArray>(topicName.c_str(), 1);
	topicName = "/observer/g_cam_points_" + camID;
	green_pub = a.advertise<std_msgs::Int32MultiArray>(topicName.c_str(), 1);
}

trackobjects::~trackobjects() { }

void trackobjects::track(const sensor_msgs::ImageConstPtr& original_image)
{
	//ROS_INFO("CAM CALLBACK");
	Mat cameraFeed;
	Mat threshold;
	Mat LAB;
	//Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		//Always copy, returning a mutable CvImage
		//OpenCV expects color images to use BGR channel order.
		cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		//if there is an error during conversion, display it
		ROS_ERROR("color_tracker.cpp::cv_bridge exception: %s", e.what());
		return;
	}
	cameraFeed = cv_ptr->image;
	cvtColor(cameraFeed,LAB,CV_BGR2Lab); //converts BGR image 'cameraFeed' to LAB image 'LAB'
	sensor_msgs::ImagePtr thresh;


	
	//create some temp fruit objects so that
	//we can use their member functions/information
	LAB_Object red("red"), green("green");
	
	//first track red objects
	inRange(LAB,red.getLABmin(),red.getLABmax(),threshold);
	
	if (angler)
	{
		thresh = cv_bridge::CvImage(std_msgs::Header(), "mono8", threshold).toImageMsg();
		image_pub1_.publish(thresh);
	}
	
	
	morphOps(threshold);
	/*imshow("red", threshold);
	waitKey(15);//*/
	trackFilteredObject(red,threshold);
	red_pub.publish(r_point_arr);
	//ROS_INFO("%d",r_point_arr.data.size());
	
	//then greens
	inRange(LAB,green.getLABmin(),green.getLABmax(),threshold);
	
	if (angler)
	{
		thresh = cv_bridge::CvImage(std_msgs::Header(), "mono8", threshold).toImageMsg();
		image_pub2_.publish(thresh);
	}
	
	morphOps(threshold);
	/*imshow("green", threshold);
	waitKey(15);//*/
	trackFilteredObject(green,threshold);
	green_pub.publish(g_point_arr);
	
	waitKey(15);
}

void broadcastAngles(bool angler){}

void trackobjects::setLocArrs(vector<LAB_Object>& theObjects)
{
	if (theObjects.at(0).getColor() == Scalar(0,255,0))
		g_point_arr.data.clear();
	if (theObjects.at(0).getColor() == Scalar(0,0,255))
		r_point_arr.data.clear();
	//set the location arrays for the green and red objects :
	//(the arrays are global data)
	for (int i = 0; i<theObjects.size(); i++)
	{
		
		//theObjects.at(i).getYPos();
		if (theObjects.at(i).getColor() == Scalar(0,255,0))//green objects
		{
			g_point_arr.data.push_back( theObjects.at(i).getXPos() );
			g_point_arr.data.push_back( theObjects.at(i).getYPos() );
		}
		else if (theObjects.at(i).getColor() == Scalar(0,0,255))//red objects
		{
			r_point_arr.data.push_back( theObjects.at(i).getXPos() );
			r_point_arr.data.push_back( theObjects.at(i).getYPos() );
		}
	}
}


void trackobjects::morphOps(Mat&thresh)
{

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle
	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));

	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);

	dilate(thresh,thresh,dilateElement);
	dilate(thresh,thresh,dilateElement);
}


void trackobjects::trackFilteredObject(LAB_Object& theObject, Mat threshold)
{
	objects.clear();
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(threshold.clone(),contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );// posible optimization
	//clear the message arrays:
	////greenArr.data.clear();
	////redArr.data.clear();
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) 
	{
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if(numObjects<MAX_NUM_OBJECTS)
		{
			for (int index = 0; index >= 0; index = hierarchy[index][0]) 
			{

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if(area>MIN_OBJECT_AREA)
				{

					LAB_Object object;

					object.setXPos(moment.m10/area);
					object.setYPos(moment.m01/area);
					object.setType(theObject.getType());
					object.setColor(theObject.getColor());

					objects.push_back(object);

					objectFound = true;

				}
				else objectFound = false;
			}
			//let user know you found an object
			if(objectFound ==true)
			{
				setLocArrs(objects);
			}
		}
	}
}//void trackfilteredobjects

int main(int argc, char** argv)
{	
	waitKey(2000);
	ros::init(argc, argv, "observer_color_tracker");
	ROS_INFO("Color_tracking node initialized");
	trackobjects /*top_cam("1"), top_right_cam("2"), bottom_right_cam("3"), 
				bottom_cam("4"), bottom_left_cam("5"), top_left_cam("6"),*/
				bottom_cam("1"),
				down_cam;
	ros::MultiThreadedSpinner spinner(1); //use 8 threads
	spinner.spin(); //will not return until node is shut down
}

 

