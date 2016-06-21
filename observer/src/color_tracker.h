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
#include "LAB_Object.h" //uses namespaces cv and std
#include <string.h>

class trackobjects{
private:
	ros::NodeHandle nh_;
    image_transport::ImageTransport it_;    
    image_transport::Subscriber image_sub_;
	int MAX_NUM_OBJECTS;
	int MIN_OBJECT_AREA;
public:
    trackobjects();
    
    trackobjects(std::string camID);

    ~trackobjects();

    void track(const sensor_msgs::ImageConstPtr& original_image);
    
	void setLocArrs(vector<LAB_Object> theObjects);

	void morphOps(Mat &thresh);

	void trackFilteredObject(LAB_Object theObject,Mat threshold);
 };
