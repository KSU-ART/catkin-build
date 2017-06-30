#include "color.h"

void Color::set_color(char inColor){
	switch(inColor){
	case 'g':
		ifstream green;
		green.open(calibration_file + "/green.txt");
		if (green.is_open()){
			green >> lMin;
			green >> aMin;
			green >> bMin;
			green >> lMax;
			green >> aMax;
			green >> bMax;
		}
		green.close();
		break;
		
	case 'r':
		ifstream red;
		red.open(calibration_file + "/red.txt");
		if (red.is_open()){
			red >> lMin;
			red >> aMin;
			red >> bMin;
			red >> lMax;
			red >> aMax;
			red >> bMax;
		}
		red.close();
		break;
		
	default:
		lMin = 0;
		aMin = 0;
		bMin = 0;
		lMax = 180;
		aMax = 255;
		bMax = 255;
	}
}

void Color::set_calibration_file(std::string file){
	if (file.compare("") == 0){
		calibration_file = "/home/stoplime/catkin_ws/src/catkin-build/plate_detect/include";
	}
	else{
		calibration_file = file;
	}
}


void Color::calibrate_colors(){
	VideoCapture cap(1);
	if (!cap.isOpened()){
		return -1;
	}
	Mat img_rgb,img_lab;
	char color;
	namedWindow("Img_RGB", WINDOW_AUTOSIZE);
	setMouseCallback("Img_RGB", calibration_click_cb, NULL);
	cout <<"Enter the first letter of the color you are looking for (r for RED g for Green): ";
	cin >> color;
	if(color == 'r'|| color == 'g'){
		while(1){
			cap>>img_rgb;
			cvtColor(img_rgb, img_lab, COLOR_BGR2Lab);
			medianBlur(img_lab,img_lab,11);
			int lSum = 0;
			int aSum = 0;
			int bSum = 0;
			int count = 0;
			for(int x = min(points[1].x,points[2].x);x<=max(points[1].x,points[2].x);x++){
					for(int y = min(points[1].y,points[2].y);y<=max(points[1].y,points[2].y);y++){
						lSum += img_lab.at<Vec3b>(y, x).val[0];
						aSum += img_lab.at<Vec3b>(y, x).val[1];
						bSum += img_lab.at<Vec3b>(y, x).val[2]; 
						count++;
					}
				}
			lMin = (lSum-lSum*percentage)/count;
			aMin = (aSum-aSum*percentage)/count;
			bMin = (bSum-bSum*percentage)/count;
			lMax = (lSum+lSum*percentage)/count;
			aMax = (aSum+aSum*percentage)/count;
			bMax = (bSum+bSum*percentage)/count;
			if(clicks>=2){
				if(color == 'r'){
					ofstream myfile (calibration_file + "/red.txt");
					if (myfile.is_open()){
						myfile <<lMin <<" " <<aMin <<" " <<bMin <<" " <<lMax <<" " <<aMax <<" " <<bMax <<endl; 
						myfile.close();
					}
				}
				if(color == 'g'){
					ofstream myfile (calibration_file + "/green.txt");
					if (myfile.is_open()){
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
}
