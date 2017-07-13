#include "ros/ros.h"
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include "color.h"
#include <iostream>
#include "imageDecoder.h"
#include <string>

using namespace std;
using namespace cv;

string path = "/home/stoplime/catkin_ws/src/catkin-build/plate_detect/include";

bool sortFunc (int i, int j) { return (i>j); }

template <typename T>
vector<size_t> sort_indexes(const vector<T> &v) {

  // initialize original index locations
  vector<size_t> idx(v.size());
  iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  sort(idx.begin(), idx.end(),
       [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});

  return idx;
}

float getSquareDist(Point p){
	return p.x*p.x + p.y*p.y;
}

int main( int argc, char** argv ){
	int cam_width = 640;
	int cam_height = 480;
	bool DEBUG = true;

	ros::init(argc, argv,"roomba_pose");
	ros::NodeHandle nh;
	ros::Publisher pubx = nh.advertise<std_msgs::Int16>("/IARC/OrientationNet/pos/x", 1);
	ros::Publisher puby = nh.advertise<std_msgs::Int16>("/IARC/OrientationNet/pos/y", 1);
	ros::Publisher detectPub = nh.advertise<std_msgs::Bool>("/IARC/OrientationNet/detected", 1);

	imageDecoder cap("/sensor/forwardCam");
	Mat color, labcs, labThresh, Thresh1, Thresh2;
	//Define color of blobs to track
	Color blob1('r', path);
	Color blob2('g', path);
	std_msgs::Bool detected;
	while(ros::ok())
	{
		//Passing video footage
		color = cap.get_image();
		if(!color.empty()){
			// std::cout << "image found" << std::endl;
			//Define CIE Lab img and smooth it
			cvtColor(color, labcs, COLOR_BGR2Lab);
			medianBlur(labcs,labcs,11);
			//Std Altitude Threshold
			inRange(labcs, blob1.get_min_scalar(), blob1.get_max_scalar(), Thresh1);
			//Low Altitude Threshold
			inRange(labcs, blob2.get_min_scalar(), blob2.get_max_scalar(), Thresh2);
			
			/// Noise reduction
			// Mat erodeElement2 = getStructuringElement(MORPH_RECT, Size(21, 21));
			// Mat dilateElement2 = getStructuringElement(MORPH_RECT, Size(9, 9));
			// erode(Thresh2, Thresh2, erodeElement2);
			// dilate(Thresh2, Thresh2, dilateElement2);

			///Combine the two Thresholds
			labThresh = Thresh1|Thresh2;
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
				detected.data = true;
				detectPub.publish(detected);

				for(int i=0;i<contours.size();i++)
					areas.push_back(contourArea(contours[i], false));
				
				vector<size_t> sorted_areas = sort_indexes(areas);
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
				// for(int i = 0; i < sorted_areas.size(); i++){
				// 	cout << areas[sorted_areas[i]] << " ";
				// }
				// cout << endl;
				for(int i = 0; i < 3; i++){
					int ind = sorted_areas.size()-(i+1);
					cout << areas[sorted_areas[ind]] << " \t";
					if(ind >= 0){
						// cout << ind << " ";

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
				cout << endl;
				
				// center = Point(xSum/contoursPoints.size(), ySum/contoursPoints.size());
				// Point delta = center - Point(cam_width/2, cam_height/2);
				std_msgs::Int16 msg;
				msg.data = closestDelta.x;
				pubx.publish(msg);
				msg.data = closestDelta.y;
				puby.publish(msg);

				//cout <<"< " <<center.x <<" , " <<center.y <<" >" <<endl;
				drawContours(color, contours, closestIndex, Scalar(0, 255, 255), 3, 8, vector<Vec4i>(), 0, Point());
				circle(color, closestPoint, 5, Scalar(255, 255, 0), FILLED, LINE_8);
				circle(color, Point(cam_width/2, cam_height/2), 5, Scalar(255, 255, 0), FILLED, LINE_8);
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
			if(!color.empty()){
				imshow("Original",color);
			}
		}
		ros::spinOnce();
		waitKey(5);
	}
}
