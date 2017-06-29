#include "opencv2/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <algorithm> 
#include <math.h>
#include <string>
#include <fstream>

using namespace std;
using namespace cv;

Point points[2] = {Point(0,0),Point(1,1)};
int clicks = 0;

float percentage = 0.2;
string calibration_file = "/home/stoplime/catkin_ws/src/catkin-build/plate_detect/include/plate_detect";

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if  ( event == EVENT_LBUTTONDOWN )
    {
		clicks++;
		points[clicks]=(Point(x,y));
    }
}

int main(int argc, char** argv) {
	VideoCapture cap(1);
	if (!cap.isOpened()){
		return -1;
	}
	Mat img_rgb,img_lab;
	char color;
	namedWindow("Img_RGB", WINDOW_AUTOSIZE);
	setMouseCallback("Img_RGB", CallBackFunc, NULL);
	cout <<"Enter the first letter of the color you are looking for (r for RED g for Green): ";
	cin >> color;
	if(color == 'r'|| color == 'g')
	{
		while(1)
		{
			cap>>img_rgb;
			cvtColor(img_rgb, img_lab, COLOR_BGR2Lab);
			medianBlur(img_lab,img_lab,11);
			int lSum = 0;
			int aSum = 0;
			int bSum = 0;
			int count = 0;
			for(int x = min(points[1].x,points[2].x);x<=max(points[1].x,points[2].x);x++)
				{
					for(int y = min(points[1].y,points[2].y);y<=max(points[1].y,points[2].y);y++)
					{
						lSum += img_lab.at<Vec3b>(y, x).val[0];
						aSum += img_lab.at<Vec3b>(y, x).val[1];
						bSum += img_lab.at<Vec3b>(y, x).val[2]; 
						count++;
					}
				}
			int lMin = (lSum-lSum*percentage)/count; 
			int aMin = (aSum-aSum*percentage)/count;	
			int bMin = (bSum-bSum*percentage)/count;	
			int lMax = (lSum+lSum*percentage)/count;
			int aMax = (aSum+aSum*percentage)/count;
			int bMax = (bSum+bSum*percentage)/count;
			if(clicks>=2)
			{
				if(color == 'r')
				{
					ofstream myfile (calibration_file + "/red.txt");
					if (myfile.is_open())
					{
						myfile <<lMin <<" " <<aMin <<" " <<bMin <<" " <<lMax <<" " <<aMax <<" " <<bMax <<endl; 
						myfile.close();
					}
				}
				if(color == 'g')
				{
					ofstream myfile (calibration_file + "/green.txt");
					if (myfile.is_open())
					{
						myfile <<lMin <<" " <<aMin <<" " <<bMin <<" " <<lMax <<" " <<aMax <<" " <<bMax <<endl; 
						myfile.close();
					}
				}
				cout <<lMin <<" " <<aMin <<" " <<bMin <<" " <<lMax <<" " <<aMax <<" " <<bMax <<endl; 
				destroyAllWindows();
				break;
			
			}
			imshow("Img_RGB",img_rgb);
			waitKey(5);
		}
	}
	else
		cout <<"Invalid input. Calibration exiting...";
	
	return 0;
}
