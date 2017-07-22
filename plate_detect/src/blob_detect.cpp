#include "ros/ros.h"
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <math.h>
#include "color.h"
#include <iostream>
#include <string>
#include "imageDecoder.h"

using namespace std;
using namespace cv;

string path = "/home/marvin/catkin_ws/src/catkin-build/plate_detect/include";
Mat src;

// bool sortFunc (int i, int j) { return (i>j); }

template <typename T>
vector<size_t> sort_indexes(const vector<T> &v) {

	// initialize original index locations
	vector<size_t> idx(v.size());
	iota(idx.begin(), idx.end(), 0);

	// sort indexes based on comparing values in v
	sort(idx.begin(), idx.end(), [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});

	return idx;
}

float getSquareDist(Point p){
	return p.x*p.x + p.y*p.y;
}

void image_callback(const sensor_msgs::Image::ConstPtr& msg){
	cout << "call" << endl;
	cv_bridge::CvImagePtr cv_ptr;
	try{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	src = cv_ptr->image;
}

int main( int argc, char** argv ){
	int cam_width = 640;
	int cam_height = 480;
	int areaThresh = 400;
	bool DEBUG = true;

	ros::init(argc, argv,"roomba_pose");
	ros::NodeHandle nh;

	// ros::Subscriber imageSub = nh.subscribe("/usb_cam_1/image_raw", 1, image_callback);
	imageDecoder im("/sensor/compressed/downCam");

	ros::Publisher pubx = nh.advertise<std_msgs::Int16>("/IARC/OrientationNet/pos/x", 1);
	ros::Publisher puby = nh.advertise<std_msgs::Int16>("/IARC/OrientationNet/pos/y", 1);
	ros::Publisher detectPub = nh.advertise<std_msgs::Bool>("/IARC/OrientationNet/detected", 1);

	Mat labcs, labThresh, Thresh1, Thresh2;
	//Define color of blobs to track
	Color blob1('r', path);
	Color blob2('g', path);
	std_msgs::Bool detected;
	while(ros::ok())
	{
		//Passing video footage
		src = im.get_image();

		if(!src.empty()){
			// std::cout << "image found" << std::endl;
			//Define CIE Lab img and smooth it
			cvtColor(src, labcs, COLOR_BGR2Lab);
			medianBlur(labcs,labcs,11);
			//Std Altitude Threshold
			inRange(labcs, blob1.get_min_scalar(), blob1.get_max_scalar(), Thresh1);
			//Low Altitude Threshold
			inRange(labcs, blob2.get_min_scalar(), blob2.get_max_scalar(), Thresh2);

			///Combine the two Thresholds
			labThresh = Thresh1|Thresh2;

			/// Noise reduction
			Mat erodeElement1 = getStructuringElement(MORPH_RECT, Size(11, 11));
			Mat dilateElement1 = getStructuringElement(MORPH_RECT, Size(9, 9));
			Mat dilateElement2 = getStructuringElement(MORPH_RECT, Size(15, 15));
			
			dilate(labThresh, labThresh, dilateElement1);
			erode(labThresh, labThresh, erodeElement1);
			dilate(labThresh, labThresh, dilateElement1);

			//Blob detection and center point generation 
			vector< vector<Point> > contours;
			findContours(labThresh, contours, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
			vector<float> areas;
			vector<Point> contoursPoints;
			int lrgContour;
			Point center;
			Point delta;
			if(contours.size()!=0)
			{

				for(int i=0;i<contours.size();i++)
					areas.push_back(contourArea(contours[i], false));
				
				vector<size_t> sorted_areas = sort_indexes(areas);
				// vector<size_t> sorted_areas(areas.size(), 0);
				// lrgContour = distance(areas.begin(),max_element(areas.begin(),areas.end()));
				lrgContour = sorted_areas[sorted_areas.size()-1];
				// cout << "areas contains:";
				// for (vector<float>::iterator it=areas.begin(); it!=areas.end(); ++it)
				// 	cout << ' ' << *it;
				// cout << '\n';
				// cout << lrgContour << endl;
				float closestDistSq;
				int closestIndex;
				Point closestDelta;
				Point closestPoint;
				bool is_detected = false;

				for(int i = 0; i < 3; i++){
					int ind = sorted_areas.size()-(i+1);
					if(ind >= 0){
						if(areas[sorted_areas[ind]] < areaThresh){
							cout << "not big enough: ";
							cout << areas[sorted_areas[ind]] << endl;
							continue;
						}
						cout << areas[sorted_areas[ind]] << " ";
						is_detected = true;

						int xSum =0;
						int ySum =0;
						contoursPoints = contours[sorted_areas[ind]];
						for(int j =0;j<contoursPoints.size();j++){
							xSum += contoursPoints[j].x;
							ySum += contoursPoints[j].y; 
						}

						center = Point(xSum/contoursPoints.size(), ySum/contoursPoints.size());
						delta = center - Point(cam_width/2, cam_height/2);
						// cout << center << " " << getSquareDist(delta) << endl;
						cout << getSquareDist(delta) << endl;
						if(i == 0){
							closestDistSq = getSquareDist(delta);
							closestIndex = sorted_areas[ind];
							closestDelta = delta;
							closestPoint = center;
						}
						else if(getSquareDist(delta) < closestDistSq){
							closestDistSq = getSquareDist(delta);
							closestIndex = sorted_areas[ind];
							closestDelta = delta;
							closestPoint = center;
						}

					}
				}
				detected.data = is_detected;
				detectPub.publish(detected);
				cout << endl;
				
				// center = Point(xSum/contoursPoints.size(), ySum/contoursPoints.size());
				// Point delta = center - Point(cam_width/2, cam_height/2);
				std_msgs::Int16 msg;
				msg.data = closestDelta.x;
				pubx.publish(msg);
				msg.data = closestDelta.y;
				puby.publish(msg);

				if(DEBUG){
					//cout <<"< " <<center.x <<" , " <<center.y <<" >" <<endl;
					if(is_detected){
						drawContours(src, contours, closestIndex, Scalar(0, 255, 255), 3, 8, vector<Vec4i>(), 0, Point());
						circle(src, closestPoint, 5, Scalar(255, 255, 0), FILLED, LINE_8);
					}
					circle(src, Point(cam_width/2, cam_height/2), 5, Scalar(255, 255, 0), FILLED, LINE_8);
				}
			}
		}
		else{
			detected.data = false;
			detectPub.publish(detected);
		}
		if(DEBUG){
			if(!labThresh.empty()){
				imshow("Thresh",labThresh);
			}
			if(!src.empty()){
				imshow("Original",src);
			}
		}
		ros::spinOnce();
		waitKey(5);
	}
}
