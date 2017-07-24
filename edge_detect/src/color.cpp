#include "color.h"

// static implementation
void Color::onMouse(int event, int x, int y, int flags, void* userdata){
	Color* set_color_obj = reinterpret_cast<Color*>(userdata);
    set_color_obj->onMouse(event, x, y);
}
// member implementation
void Color::onMouse(int event, int x, int y){
	if  ( event == cv::EVENT_LBUTTONDOWN ){
		clicks++;
		points[clicks]=(cv::Point(x,y));
	}
}

cv::Scalar Color::get_min_scalar(){
	return cv::Scalar(lMin, aMin, bMin);
}
cv::Scalar Color::get_max_scalar(){
	return cv::Scalar(lMax, aMax, bMax);
}

void Color::set_color(char inColor){
	color_type = inColor;
	switch(inColor){
		case 'g':{
			std::ifstream green;
			std::string green_file = calibration_file + "/green.txt";
			green.open(green_file.c_str());
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
		}
		case 'r':{
			std::ifstream red;
			std::string red_file = calibration_file + "/red.txt";
			red.open(red_file.c_str());
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
		}
		default:{
			lMin = 0;
			aMin = 0;
			bMin = 0;
			lMax = 180;
			aMax = 255;
			bMax = 255;
		}
	}
}

void Color::set_calibration_file(std::string file){
	clicks = 0;
	points[0] = cv::Point(0,0);
	points[1] = cv::Point(1,1);
	if (file.compare("") == 0){
<<<<<<< HEAD
		calibration_file = "/home/odroid/catkin_ws/src/catkin-build/plate_detect/include";
=======
		calibration_file = "/home/kyle/catkin_ws/src/catkin-build/edge_detect/include";
>>>>>>> 25dbe30114245c2b54dc133a11c7ba0584c3c8c0
	}
	else{
		calibration_file = file;
	}
}


void Color::calibrate_colors(int camera, float deviation_percentage){
<<<<<<< HEAD
	cv::VideoCapture cap(camera);
	if (!cap.isOpened()){
		return;
	}
	cv::Mat img_rgb, img_lab;
=======
	cv::VideoCapture cap(0);
	if (!cap.isOpened()){
		return;
	}
	cv::Mat img_rgb,img_lab;
>>>>>>> 25dbe30114245c2b54dc133a11c7ba0584c3c8c0
	cv::namedWindow("Img_RGB", cv::WINDOW_AUTOSIZE);
	cv::setMouseCallback("Img_RGB", onMouse, this);
	if(color_type == 'r'|| color_type == 'g'){
		while(1){
			cap>>img_rgb;
			cv::cvtColor(img_rgb, img_lab, cv::COLOR_BGR2Lab);
			cv::medianBlur(img_lab,img_lab,11);
			int lSum = 0;
			int aSum = 0;
			int bSum = 0;
			int count = 0;
			for(int x = std::min(points[1].x, points[2].x); x<=std::max(points[1].x, points[2].x); x++){
					for(int y = std::min(points[1].y, points[2].y); y<=std::max(points[1].y, points[2].y); y++){
						lSum += img_lab.at<cv::Vec3b>(y, x).val[0];
						aSum += img_lab.at<cv::Vec3b>(y, x).val[1];
						bSum += img_lab.at<cv::Vec3b>(y, x).val[2]; 
						count++;
					}
				}
			lMin = (lSum-lSum*deviation_percentage)/count;
			aMin = (aSum-aSum*deviation_percentage)/count;
			bMin = (bSum-bSum*deviation_percentage)/count;
			lMax = (lSum+lSum*deviation_percentage)/count;
			aMax = (aSum+aSum*deviation_percentage)/count;
			bMax = (bSum+bSum*deviation_percentage)/count;
			if(clicks>=2){
				if(color_type == 'r'){
					std::string red_file = calibration_file + "/red.txt";
					std::ofstream myfile (red_file.c_str());
					if (myfile.is_open()){
						myfile <<lMin <<" " <<aMin <<" " <<bMin <<" " <<lMax <<" " <<aMax <<" " <<bMax <<std::endl; 
						myfile.close();
					}
				}
				if(color_type == 'g'){
					std::string green_file = calibration_file + "/green.txt";
					std::ofstream myfile (green_file.c_str());
					if (myfile.is_open()){
						myfile <<lMin <<" " <<aMin <<" " <<bMin <<" " <<lMax <<" " <<aMax <<" " <<bMax <<std::endl; 
						myfile.close();
					}
				}
				std::cout <<lMin <<" " <<aMin <<" " <<bMin <<" " <<lMax <<" " <<aMax <<" " <<bMax <<std::endl; 
				cv::destroyAllWindows();
				break;
			
			}
			cv::imshow("Img_RGB",img_rgb);
			cv::waitKey(5);
		}
	}
	else
		std::cout <<"Unknown color setting. Please set color either r for red or g for green";
}
<<<<<<< HEAD
=======

int Color::getLMin(){return lMin;}
int Color::getLMax(){return lMax;}
int Color::getAMin(){return aMin;}
int Color::getAMax(){return aMax;}
int Color::getBMin(){return bMin;}
int Color::getBMax(){return bMax;}
>>>>>>> 25dbe30114245c2b54dc133a11c7ba0584c3c8c0
