#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <vector>
#include <geometry_msgs/Point.h>
#include "camera_model.h"

using namespace cv;
using namespace std;

class grid_tracker
{
public:
	grid_tracker();
	~grid_tracker();
	/// Post: runs the main loop for grid tracking
	void grid_algorithm();
	
	Mat src;
	Vec2f pos, velocity;
private:
	bool debug;
	
	vector<Vector3> *preIntersects, *curIntersects;
	
	/// Pre: lines are composed of rho and theta
	/// Post: draws lines on img
	void drawLine(Vec2f line, Mat &img, Scalar rgb = CV_RGB(0,0,255));
	/// Pre: previous vector of intersects,
	///		 the delta time taken from previous intersects
	/// Post: return a single velocity vector in the average distance traveled by all intersects 
	void distanceTraveled(vector<Vector3> *pre_intersects, vector<Vector3> *cur_intersects);
	/// Post: Find if a point x,y exist in a group. If yes, return group i
	bool inGroup(vector<vector<Vec2f*> > groups, const float x, const float y, int& i);
	/// Pre: takes in list of intersection points
	/// Post: returns the average points, where the the groups are within a threshold
	///			ignors the points near the edge of the screen
	vector<Vec2f> avaragePoint(vector<Vec2f> *intersects, int groupThresh, int boundryThresh, vector<Vector3> *averaged);
	/// Pre: lines of theta and rho
	/// Post: returns a list of intersects on x,y points
	void findIntersectLines(vector<Vec2f> *lines, int angle, vector<Vec2f> *intersects);
	/// pre: lines of theta ond rho
	/// post: merges the parallel lines together
	void mergeRelatedLines(vector<Vec2f> *lines, Mat &img);
};

