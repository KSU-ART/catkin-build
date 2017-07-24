/**
compressed_image_repeater.cpp
Purpose: subscribes to a topic and publishes a compressed version of the topic.

@author shadySource
@version 0.0.1
*/

#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
// #include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


class ImageRepeater
{
private:
    // width and height of broadcast program
    const int IMAGE_HEIGHT_WIDTH;

    ros::NodeHandle _nh;
    // image_transport::ImageTransport _it;
    ros::Subscriber _imageSub;
    ros::Publisher _compressedPub;
    
    cv::Mat _image;
    std::vector<uchar> _data;
    std_msgs::UInt8MultiArray _compressed;

    void ImageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cvPtr;
        try
        {
            cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Resize image
        // cv::resize(cvPtr->image, _image, cv::Size(IMAGE_HEIGHT_WIDTH, IMAGE_HEIGHT_WIDTH));

        //Compress image
        cv::imencode(".jpg", cvPtr->image, _data);

        _compressed.data = _data;
        
        // Publish compressed image
        _compressedPub.publish(_compressed);
    }

public:

    ImageRepeater(std::string cameraTopic, int imageShape=416)
        : IMAGE_HEIGHT_WIDTH(imageShape)
    {
        // Set up communications
        _imageSub = _nh.subscribe(cameraTopic, 1, &ImageRepeater::ImageCallback, this);
        _compressedPub = _nh.advertise<std_msgs::UInt8MultiArray>(cameraTopic + "/compressed", 1000);

        // Set up message for compressed jpeg stream
        _compressed.layout.dim.push_back(std_msgs::MultiArrayDimension());
        _compressed.layout.dim[0].label = "img_data";
        _compressed.layout.dim[0].stride = 7;
        _compressed.layout.data_offset = 0;
    }
};

// int main(int argc, char ** argv)
// {
//     std::string topicName("/usb_cam/image_raw");

//     if (argc == 2) // If argument given.
//     {
//         topicName = argv[1];
//     }

//     ros::init(argc, argv, "talker");

//     auto ImageRepeater repeater(topicName);

//     ros::Rate rate(120);
//     while(ros::ok())
//     {
//         ros::spinOnce();
//         rate.sleep();
//     }

// }