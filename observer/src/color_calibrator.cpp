/*******************************************************************
 * This program is a calibration tool to find the desired LAB color 
 * 	range for color based plate tracking.
********************************************************************/
#include <ros/ros.h>
#include <stdlib.h>
#include <iostream>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include <string>
#include <iostream>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int16MultiArray.h"
#include "LAB_Object.h"

using namespace std;
using namespace cv;

namespace enc = sensor_msgs::image_encodings;

int L_MIN = 0;
int L_MAX = 255;
int A_MIN = 0;
int A_MAX = 255;
int B_MIN = 0;
int B_MAX = 255;

//default capture width and height
const int FRAME_WIDTH = 1280;
const int FRAME_HEIGHT = 720;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20*20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
//names that will appear at the top of each window
const string windowName = "Original Image";
const string windowName1 = "LAB Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";

bool calibrationMode = true;

//The following for location publishers
std_msgs::Int16MultiArray greenArr;
std_msgs::Int16MultiArray redArr;
std_msgs::Int16MultiArray testArr;

int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
string window_name = "Edge Map";


void on_trackbar( int, void* )
{//This function gets called whenever a
	// trackbar position is changed

}

string intToString(int number){

	std::stringstream ss;
	ss << number;
	return ss.str();
}

void createTrackbars(){
	//create window for trackbars
	namedWindow(trackbarWindowName,0);
	//create memory to store trackbar name on window
	char TrackbarName[50];

	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH),
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->
	createTrackbar( "L_MIN", trackbarWindowName, &L_MIN, L_MAX, on_trackbar );
	createTrackbar( "L_MAX", trackbarWindowName, &L_MAX, L_MAX, on_trackbar );
	createTrackbar( "A_MIN", trackbarWindowName, &A_MIN, A_MAX, on_trackbar );
	createTrackbar( "A_MAX", trackbarWindowName, &A_MAX, A_MAX, on_trackbar );
	createTrackbar( "B_MIN", trackbarWindowName, &B_MIN, B_MAX, on_trackbar );
	createTrackbar( "B_MAX", trackbarWindowName, &B_MAX, B_MAX, on_trackbar );
}
void setLocArrs(vector<LAB_Object> theObjects,Mat &frame, Mat &temp, vector< vector<Point> > contours, vector<Vec4i> hierarchy)
{
	//set the location arrays for the green and red objects :
	//(the arrays are global data)
	for (int i = 0; i<theObjects.size(); i++)
	{
		if (theObjects.at(i).getColor() == Scalar(0,255,0))//green objects
		{
			greenArr.data.push_back(theObjects.at(i).getXPos());
			greenArr.data.push_back(theObjects.at(i).getYPos());
		}
		if (theObjects.at(i).getColor() == Scalar(0,0,255))//red objects
		{
			redArr.data.push_back(theObjects.at(i).getXPos());
			redArr.data.push_back(theObjects.at(i).getYPos());
		}
	}
}
void setLocArrs(vector<LAB_Object> theObjects,Mat &frame)
{
	for (int i = 0; i < theObjects.size(); i++)
	{
		testArr.data.push_back(theObjects.at(i).getXPos());
		testArr.data.push_back(theObjects.at(i).getYPos());
	}
}

void drawObject(vector<LAB_Object> theObjects,Mat &frame, Mat &temp, vector< vector<Point> > contours, vector<Vec4i> hierarchy){

	for(int i =0; i<theObjects.size(); i++){
	cv::drawContours(frame,contours,i,theObjects.at(i).getColor(),3,8,hierarchy);
	cv::circle(frame,cv::Point(theObjects.at(i).getXPos(),theObjects.at(i).getYPos()),5,theObjects.at(i).getColor());
	cv::putText(frame,intToString(theObjects.at(i).getXPos())+ " , " + intToString(theObjects.at(i).getYPos()),cv::Point(theObjects.at(i).getXPos(),theObjects.at(i).getYPos()+20),1,1,theObjects.at(i).getColor());
	cv::putText(frame,theObjects.at(i).getType(),cv::Point(theObjects.at(i).getXPos(),theObjects.at(i).getYPos()-20),1,2,theObjects.at(i).getColor());	
	}
}

void drawObject(vector<LAB_Object> theObjects,Mat &frame){

	for(int i =0; i<theObjects.size(); i++){

	cv::circle(frame,cv::Point(theObjects.at(i).getXPos(),theObjects.at(i).getYPos()),10,cv::Scalar(0,0,255));
	cv::putText(frame,intToString(theObjects.at(i).getXPos())+ " , " + intToString(theObjects.at(i).getYPos()),cv::Point(theObjects.at(i).getXPos(),theObjects.at(i).getYPos()+20),1,1,Scalar(0,255,0));
	cv::putText(frame,theObjects.at(i).getType(),cv::Point(theObjects.at(i).getXPos(),theObjects.at(i).getYPos()-30),1,2,theObjects.at(i).getColor());
	}
}

void morphOps(Mat &thresh){

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

void trackFilteredObject(Mat threshold,Mat LAB, Mat &cameraFeed) //LAB not used??
{
	vector <LAB_Object> objects;
	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//clear message data:
	greenArr.data.clear();
	redArr.data.clear();
	testArr.data.clear();
	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
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

					objects.push_back(object);

					objectFound = true;

				}
				else objectFound = false;
			}
			//let user know you found an object
			if(objectFound ==true)
			{
				//draw object location on screen
				drawObject(objects,cameraFeed);
				setLocArrs(objects,cameraFeed);
			}
		}
		else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
	}
}

void trackFilteredObject(LAB_Object theObject,Mat threshold,Mat LAB, Mat &cameraFeed) //LAB not used??
{
	vector <LAB_Object> objects;
	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	//clear the message arrays:
	greenArr.data.clear();
	redArr.data.clear();
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if(numObjects<MAX_NUM_OBJECTS){
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
				//draw object location on screen
				drawObject(objects,cameraFeed,temp,contours,hierarchy);
				setLocArrs(objects,cameraFeed,temp,contours,hierarchy);
				putText(cameraFeed,"Objects Found",Point(0,50),1,2,Scalar(0,0,255),2);
			}
			}	
			else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
	}
} 

class trackobjects
{
    ros::NodeHandle loc_node;
    ros::Publisher green_loc_arr;
    ros::Publisher red_loc_arr;
    ros::Publisher test_loc_arr;
	ros::NodeHandle nh_;
    image_transport::ImageTransport it_;    
    image_transport::Subscriber image_sub_; //image subscriber 
    image_transport::Publisher image_pub_; //image publisher
    image_transport::Publisher image_pub2_;
    image_transport::Publisher image_pub3_;
    
    std_msgs::String msg;

public:
    trackobjects()
    : it_(nh_)
    {
        image_sub_ = it_.subscribe("/usb_cam_0/image_raw", 1, &trackobjects::imageCb, this);
        image_pub_= it_.advertise("/usb_cam/image_tracked",1);
        image_pub2_=it_.advertise("green_binary",1);
        image_pub3_=it_.advertise("red_binary",1);
		red_loc_arr = loc_node.advertise<std_msgs::Int16MultiArray>("red_loc", 50);
		green_loc_arr = loc_node.advertise<std_msgs::Int16MultiArray>("green_loc", 50);
		test_loc_arr = loc_node.advertise<std_msgs::Int16MultiArray>("calibrating_loc", 100);
    }

    ~trackobjects()
    {
		
    }

    void imageCb(const sensor_msgs::ImageConstPtr& original_image)
    {
		

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
            ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
            return;
        }
        Mat cameraFeed;
		Mat threshold;
		//Mat HSV;
		Mat LAB;
		cameraFeed = cv_ptr->image;
        //cvtColor(cameraFeed,HSV,COLOR_BGR2HSV); //converts BGR image 'cameraFeed' to HSV image 'HSV' 
        cvtColor(cameraFeed,LAB,CV_BGR2Lab); //converts BGR image 'cameraFeed' to LAB image 'LAB'
		sensor_msgs::ImagePtr thresh;
        
        
        if(calibrationMode==true){

			//need to find the appropriate color range values
			// calibrationMode must be false

			//if in calibration mode, we track objects based on the LAB slider values.			
			
			inRange(LAB,Scalar((L_MIN),(A_MIN),(B_MIN)),Scalar((L_MAX),(A_MAX),(B_MAX)),threshold);
			
			morphOps(threshold);
			thresh = cv_bridge::CvImage(std_msgs::Header(), "mono8", threshold).toImageMsg();
			image_pub2_.publish(thresh);
			
			imshow(windowName2,threshold);
			trackFilteredObject(threshold,LAB,cameraFeed);
			imshow(windowName,cameraFeed);
			test_loc_arr.publish(testArr);
		}
        else{
			//create some temp fruit objects so that
			//we can use their member functions/information
			LAB_Object red("red"), green("green");
			
			
/*			To make program in LAB instead of HSV format, must change Object to be all in LAB format methods and 
 * 				save values in that format. Set the max and min values for the color that you want to detect.
 * 			Then must use inRange here in the else statement of imageCb to compare LAB to the color objects min and
 * 				max and the threshhold set.
*/

			//forst track red objects
			inRange(LAB,red.getLABmin(),red.getLABmax(),threshold);
			thresh = cv_bridge::CvImage(std_msgs::Header(), "mono8", threshold).toImageMsg();
			image_pub3_.publish(thresh);
			morphOps(threshold);
			trackFilteredObject(red,threshold,LAB,cameraFeed);
			red_loc_arr.publish(redArr);
			//then greens
			inRange(LAB,green.getLABmin(),green.getLABmax(),threshold);
			thresh = cv_bridge::CvImage(std_msgs::Header(), "mono8", threshold).toImageMsg();
			image_pub2_.publish(thresh);
			morphOps(threshold);
			trackFilteredObject(green,threshold,LAB,cameraFeed);
			green_loc_arr.publish(greenArr);
		}
		//show frames
		//imshow(windowName2,threshold);
		//use to imshow tracked objects
		//imshow(windowName1,LAB);
		//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command
		waitKey(30);
		image_pub_.publish(cv_ptr->toImageMsg());
	}
};
 
 
int main(int argc, char** argv)
{

	if(calibrationMode){
		//create slider bars for LAB filtering
		createTrackbars();
	}
	
	waitKey(1000);
    ros::init(argc, argv, "find_objectblob");
    trackobjects ic;
    ros::spin();

    return 0;
}
