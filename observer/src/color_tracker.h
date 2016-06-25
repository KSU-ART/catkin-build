/*******************************************************************
 * This program takes a camera feed and publishes:
 *  an image showing objects detected in the set color range, 
 *  binary images filtered for color,
 *  and arrays with xy coordinates of the center of detected objects.
 * There is also a calibrationMode for finding desired color range.
********************************************************************/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/highgui/highgui.hpp>
#include "LAB_Object.h"
#include <string.h>

class trackobjects{
private:
	ros::NodeHandle nh_;
    image_transport::ImageTransport it_;    
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub1_;
    image_transport::Publisher image_pub2_;
	int MAX_NUM_OBJECTS;
	int MIN_OBJECT_AREA;
	bool angler;
	vector <LAB_Object> objects;
	vector <Point> greenObjects;
	vector <Point> redObjects;
	
	//std_msgs::UInt32MultiArray greenArr;
	//std_msgs::UInt32MultiArray redArr;
	
	
public:
    trackobjects();
    
    trackobjects(std::string camID);

    ~trackobjects();
    
    ///Post: publishes binary to angler switch
    void broadcastAngles(bool angler);
    
    ///Post: runs tracking
    void track(const sensor_msgs::ImageConstPtr& original_image);
    
    ///Post: differentiates between grean and red objects
	void setLocArrs(vector<LAB_Object>& theObjects);

	///Post: returns solid objects from binary image (denoising)
	void morphOps(Mat& thresh);
	
	///Post: takes binary into 
	void trackFilteredObject(LAB_Object& theObject,Mat threshold);
 };
