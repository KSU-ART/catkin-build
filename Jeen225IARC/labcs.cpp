#include "opencv2/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include "color.h"
#include <iostream>

using namespace std;
using namespace cv;


int main( int argc, char** argv ){
	VideoCapture cap(1);
	if (!cap.isOpened()){
		return -1;
	}
	Mat color,labcs,labThresh,Thresh1,Thresh2;
	//Std Threshold values used, the values are based on RGB from competions rules
	Color blob1('r');
	Color blob2('g');
	//int lMin =(31+128)-(31+128)*.20; 
	//int aMin =(-34+128)-(-34+128)*.20;	
	//int bMin =(33+128)-(33+128)*.20;	
	//int lMax =(31+128)+(31+128)*.20;
	//int aMax =(-34+128)+(-34+128)*.20;
	//int bMax =(33+128)+(33+128)*.20;
	while(1)
	{
		//Passing video footage
		cap>>color;
		//Define CIE Lab img and smooth it
		cvtColor(color, labcs, COLOR_BGR2Lab);
		medianBlur(labcs,labcs,11);
		//Std Altitude Threshold
		inRange(labcs, Scalar(blob1.lMin, blob1.aMin, blob1.bMin), Scalar(blob1.lMax, blob1.aMax, blob1.bMax), Thresh1);
		//Low Altitude Threshold
		inRange(labcs, Scalar(blob2.lMin, blob2.aMin, blob2.bMin), Scalar(blob2.lMax, blob2.aMax, blob2.bMax),Thresh2);
		////Noise reduction
		//Mat erodeElement2 = getStructuringElement(MORPH_RECT, Size(21, 21));
		//Mat dilateElement2 = getStructuringElement(MORPH_RECT, Size(9, 9));
		//erode(Thresh2, Thresh2, erodeElement2);
		//dilate(Thresh2, Thresh2, dilateElement2);
		////Combine the two Thresholds
		labThresh = Thresh1|Thresh2;
		//Blob detection and center point generation 
		vector<vector<Point> > contours;
		findContours(labThresh, contours, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
		vector<float> areas;
		vector<Point> contoursPoints;
		int lrgContour;
		Point center;
		if(contours.size()!=0)
		{
			for(int i=0;i<contours.size();i++)
				areas.push_back(contourArea(contours[i], false));
			int xSum =0;
			int ySum =0;
			lrgContour = distance(areas.begin(),max_element(areas.begin(),areas.end()));
			contoursPoints = contours[lrgContour];
			for(int i =0;i<contoursPoints.size();i++)
			{
				xSum += contoursPoints[i].x;
				ySum += contoursPoints[i].y; 
			}
			center = Point(xSum/contoursPoints.size(),ySum/contoursPoints.size());
			//cout <<"< " <<center.x <<" , " <<center.y <<" >" <<endl;
			drawContours(color, contours, lrgContour, Scalar(0, 255, 255), 3, 8, vector<Vec4i>(), 0, Point());
			circle(color, center, 5, Scalar(255, 255, 0), FILLED, LINE_8);
		}
		imshow("Thresh",labThresh);
		//imshow("Labcs",labcs);
		imshow("Original",color);
		waitKey(5);
	}
}
