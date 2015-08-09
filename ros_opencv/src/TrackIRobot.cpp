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
        ros_opencv::TrackingPoint boundmsg;

    public:

        ColorDetector()
            : it_(nh_)
        {
            result_pub= nh_.advertise<ros_opencv::TrackingPoint>("image_point" , 1);
            image_sub_ = it_.subscribe("/image_raw", 1, &ColorDetector::imageCb, this);
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
            Mat threshSmallRed2;
            Mat threshSmallGreen;
            Mat threshSmallYellow;
            Mat threshSmallWhite;

            resize(frame,frameSmall,Size(640,480));
			//Scalar(40,20,235), Scalar(80,125,255)
			//Scalar(50,55,210), Scalar(85,110,255)
            Mat imgRedThresh = GetThresholdedImage(frameSmall, (_InputArray)cvScalar(0,254,254), (_InputArray)cvScalar(6,255,255));
            Mat imgRedThresh2 = GetThresholdedImage(frameSmall,(_InputArray)cvScalar(170,150,60), (_InputArray)cvScalar(180,255,255));
            Mat imgGreenThresh = GetThresholdedImage(frameSmall,(_InputArray)cvScalar(50,55,210), (_InputArray)cvScalar(85,110,255));

            resize(imgRedThresh,threshSmallRed,Size(640,480));
            resize(imgRedThresh2,threshSmallRed2,Size(640,480));
            resize(imgGreenThresh,threshSmallGreen,Size(640,480));

            vector<Point> threshVector;
            Mat finalImage = imgRedThresh;
            for(int j=0; j<threshSmallRed.rows; j++) {
                for (int i=0; i<threshSmallRed.cols; i++) {
                    if(threshSmallRed.at<uchar>(j,i)==255 || threshSmallRed2.at<uchar>(j,i)==255 || threshSmallGreen.at<uchar>(j,i)==255) {
                        threshVector.push_back(Point(j,i));
                        finalImage.at<uchar>(j,i)=255;
                    }
                }
            }

            IplImage iplimage =finalImage;

            CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
            cvMoments(&iplimage, moments, 1);

            // The actual moment values
            double moment10 = cvGetSpatialMoment(moments, 1, 0);
            double moment01 = cvGetSpatialMoment(moments, 0, 1);
            double area = cvGetCentralMoment(moments, 0, 0);

            int posX;
            int posY;

            if(threshVector.size()>300) {
                posX = moment10/area;
                posY = moment01/area;
            }
            else {
                posX=-1;
                posY=-1;
            }

            circle(frameSmall, Point(posX,posY), 30, cvScalar(255,0,0), 5, 8, 0);

            //imshow("ColorTrackerRGB", frameSmall);
            //imshow("ColorTrackerThresh", finalImage);
            //cv::waitKey(3);

            threshSmallRed.release();
            threshSmallYellow.release();
            threshSmallGreen.release();
            frameSmall.release();

            boundmsg.pointX=posX;
            boundmsg.pointY=posY;
            result_pub.publish(boundmsg);
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "track_irobot");
    ColorDetector ld;
    ros::spin();
    return 0;
}
