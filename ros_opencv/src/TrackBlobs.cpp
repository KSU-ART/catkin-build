#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros_opencv/Diffmessage.h>
#include <ros_opencv/TrackingPoint.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
//#include "opencv2/nonfree/features2d.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/photo/photo.hpp>

using namespace  cv;
using namespace std;

namespace enc = sensor_msgs::image_encodings;

class ColorDetector
{
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
        ros::Publisher result_pub;
        ros_opencv::TrackingPoint trackingPoint;
        
        int sliderHMin;
        int sliderHMax;
		int sliderSMin;
        int sliderSMax;
        int sliderVMin;
        int sliderVMax;
        
    public:

        ColorDetector()
            : it_(nh_)
        {
            result_pub= nh_.advertise<ros_opencv::TrackingPoint>("image_point" , 1);
            image_sub_ = it_.subscribe("/image_raw", 1, &ColorDetector::imageCb, this);
			sliderHMin = 0;
			sliderHMax = 180;
			sliderSMin = 0;
			sliderSMax = 255;
			sliderVMin = 0;
			sliderVMax = 255;
			
			 //// Create Trackbar window
			 //namedWindow("Tracking tuning", 1);

			 ////Create trackbar to change H

			 //createTrackbar("Hue Min", "Tracking tuning", &sliderHMin, 180);
			 //createTrackbar("Hue Max", "Tracking tuning", &sliderHMax, 180);

			  ////Create trackbar to change S

			 //createTrackbar("Saturation Min", "Tracking tuning", &sliderSMin, 255);
			 //createTrackbar("Saturation Max", "Tracking tuning", &sliderSMax, 255);
			 
			 ////Create trackbar to change V
			 //createTrackbar("Value Min", "Tracking tuning", &sliderVMin, 255);
			 //createTrackbar("Value Max", "Tracking tuning", &sliderVMax, 255);
        }

        ~ColorDetector()
        {

        }

        Mat GetThresholdedImage(Mat img, _InputArray lowBound, _InputArray upBound)
        {

            Mat imgHSV(img.rows,img.cols, CV_8UC3, Scalar(0,0,0));

            cvtColor(img, imgHSV, CV_BGR2HSV);

            Mat imgThreshed(img.rows,img.cols, CV_8UC3, Scalar(0,0,0));

            inRange((_InputArray)imgHSV, lowBound, upBound, imgThreshed);

            imgHSV.release();

            return imgThreshed;
        }


        void imageCb(const sensor_msgs::ImageConstPtr& msg)
        {
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            Mat frame=cv_ptr->image;
            Mat frameSmall;
            Mat threshSmallRed;
            Mat threshSmallGreen;

            resize(frame,frameSmall,Size(640,480));
			
			//Use trackbar
			//Mat imgRedThresh = GetThresholdedImage(frameSmall, (_InputArray)cvScalar(180,255,255), (_InputArray)cvScalar(180,255,255));
			//Mat imgGreenThresh = GetThresholdedImage(frameSmall,(_InputArray)cvScalar(sliderHMin,sliderSMin,sliderVMin), (_InputArray)cvScalar(sliderHMax,sliderSMax,sliderVMax));

            Mat imgRedThresh = GetThresholdedImage(frameSmall, (_InputArray)cvScalar(140,127,111), (_InputArray)cvScalar(180,255,255));
            Mat imgGreenThresh = GetThresholdedImage(frameSmall,(_InputArray)cvScalar(40,42,100), (_InputArray)cvScalar(80,255,255));

            resize(imgRedThresh,threshSmallRed,Size(640,480));
            resize(imgGreenThresh,threshSmallGreen,Size(640,480));

            vector<Point> threshVector;
            Mat finalImage = imgRedThresh;
            for(int j=0; j<threshSmallRed.rows; j++) {
                for (int i=0; i<threshSmallRed.cols; i++) {
                    if(threshSmallRed.at<uchar>(j,i)==255 || threshSmallGreen.at<uchar>(j,i)==255) {
                        threshVector.push_back(Point(j,i));
                        finalImage.at<uchar>(j,i)=255;
                    }
                }
            }
            
            //Blur, erode, and dilate to filter the binary image
            GaussianBlur(finalImage.clone(), finalImage, cv::Size(5, 5), 2, 2);
            erode( finalImage.clone(), finalImage, Mat(), Point(-1, -1), 2, 1, 1);
            dilate( finalImage.clone(), finalImage, Mat(), Point(-1, -1), 2, 1, 1);

			vector<vector<Point> > contours;
			vector<Vec4i> hierarchy;
			Mat thresh = finalImage.clone();
			findContours(finalImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
				
			vector<vector<Point> > filteredContours;
				for (int i = 0; i< contours.size(); i++)
				{
					if(contourArea(contours[i]) > 3000){
					filteredContours.push_back(contours[i]);
					}
				}
				
			/// Draw contours
			Mat drawing = Mat::zeros(finalImage.size(), CV_8UC3);
			for (int i = 0; i< filteredContours.size(); i++)
			{
				Scalar color = Scalar(0, 255, 0);
				drawContours(drawing, filteredContours, i, color, 2, 8, hierarchy, 0, Point());
			}
			
			int posX = -1;
			int posY = -1;
			
			int cx = -1;
			int cy = -1;
			
			for (int i = 0; i< filteredContours.size(); i++){
				Moments m = moments(filteredContours[i]);
				cx = m.m10/m.m00;
				cy = m.m01/m.m00;
				circle(frameSmall, Point(cx,cy), 30, cvScalar(255,0,0), 5, 8, 0);
				if((posX == -1 && posY == -1) || (abs(cx - 320) + abs(cy - 240)) <  (abs(posX - 320) + abs(posY - 240))){
					posX = cx;
					posY = cy;
				}
			}
			
			//cout<<"Pos: "<<posX<<","<<posY<<endl;
			
            //imshow("ColorTrackerRGB", frameSmall);
            //imshow("ColorTrackerThresh", thresh);
			//imshow("Blob contours", drawing);
            //cv::waitKey(3);

            threshSmallRed.release();
            threshSmallGreen.release();
            frameSmall.release();

            trackingPoint.pointX=posX;
            trackingPoint.pointY=posY;
            result_pub.publish(trackingPoint);
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "track_irobot");
    ColorDetector ld;
    ros::spin();
    return 0;
}
