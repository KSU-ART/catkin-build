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
#include "opencv/cv.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/photo/photo.hpp"

using namespace std;

vector<cv::KeyPoint> histogramColorDetection(cv::Mat source_image, vector<cv::KeyPoint> keypoints)
{
    vector<cv::KeyPoint> detected_keypoints;
    cv::Mat denoised;
    cv::Mat img = source_image.clone();

    ///Blur source image to reduce noise
    cv::GaussianBlur(img, denoised, cv::Size(5,5), 2, 2);

    for(int i = 0; i < keypoints.size(); i++)
    {
        if(keypoints[i].pt.x > 25 || keypoints[i].pt.x < source_image.rows - 25 || keypoints[i].pt.y > 25 || keypoints[i].pt.y < source_image.cols - 25)
        {
            cv::Mat hsv, img_threshed, processed;
            cv::cvtColor(denoised, hsv, CV_BGR2HSV);
            //cv::rectangle(denoised, cv::Point(keypoints[i].pt.x - 5, keypoints[i].pt.y - 5), cv::Point(keypoints[i].pt.x + 5, keypoints[i].pt.y + 5), cv::Scalar(255, 0, 0));

            /// Set region of interest (ROI) to around the blob key point
            cv::Mat roi = hsv (cv::Rect(keypoints[i].pt.x - 5, keypoints[i].pt.y - 5, 10, 10));

            /// Split HSV into 3 channels
            std::vector<cv::Mat> hsvPlanes;
            cv::split(roi, hsvPlanes);

            /// Find mean and standard deviation of the Hue histogram of the ROI
            cv::Scalar hue_mean, hue_stddev, val_mean, val_stddev;
            cv::meanStdDev(hsvPlanes[0], hue_mean, hue_stddev);
            //cv::meanStdDev(hsvPlanes[2], val_mean, val_stddev);

            /// Get the overall Hue of the ROI (95% of the values in the center of the histogram)
            float min_hue = hue_mean[0] - hue_stddev[0]*3;
            float max_hue = hue_mean[0] + hue_stddev[0]*3;

            //float min_val = val_mean[0] - val_stddev[0]*3;
            //float max_val = val_mean[0] + val_stddev[0]*3;

            /// If overall Hue of the ROI fits what we are looking for, add to vector that we will return
            if(((max_hue < 10) || (min_hue > 135)) || ((max_hue < 115) && (min_hue > 50)))
            {
                // STEP 2: detection phase
                //cv::inRange(hsvPlanes[0], cv::Scalar(min_hue), cv::Scalar(max_hue), img_threshed);
                //imshow("thresholded", img_threshed);

                //cv::erode(img_threshed, processed, 5);  // minimizes noise
                //cv::dilate(processed, processed, 20);  // maximize left regions
                detected_keypoints.push_back(keypoints[i]);
            }
        }

    }

    //cv::imshow("Blur", denoised);
    return detected_keypoints;
}

int main(int argc, char** argv)
{
    cv::VideoCapture video("/home/hassaan/Videos/AUVSI_RAW_LINE_TRACKING_VIDEO.mp4");
    cv::Mat input_img;
    cv::Mat blobs;
    cv::Mat blur;
    vector<cv::KeyPoint> keypoints;
    vector<cv::KeyPoint> filtered_keypoints;
    cv::SimpleBlobDetector::Params params;

    while(cv::waitKey(30) < 0)
    {
        video >> input_img;

        cv::GaussianBlur(input_img.clone(), blur, cv::Size(5,5), 2, 2);

        /// Filter by Area.
        params.filterByArea = true;
        params.minArea = 200;

        /// Filter by Circularity
        params.filterByCircularity = false;
        params.minCircularity = 0.2;

        /// Filter by Convexity
        params.filterByConvexity = false;
        params.minConvexity = 0.15;

        cv::SimpleBlobDetector detector(params);
        detector.detect(blur, keypoints);
        filtered_keypoints = histogramColorDetection(input_img.clone(), keypoints);

        cv::drawKeypoints(input_img, filtered_keypoints, blobs, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        //cv::drawKeypoints(input_img, keypoints, blobs, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        cv::imshow("Blobs", blobs);

        cout << keypoints.size() << endl;
    }
}