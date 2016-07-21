///cam_model_test.cpp
#include <iostream>
#include "camera_model.h"
#include <ros/ros.h>
int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_cammodel");
	ros::Time::init();
	projection_::cameraModel c1;
	c1.saveModel('t',2.2e-6, 2.2e-6, (int)480, (int)640, 284.040145d, 282.671480d, 318.016079d, 229.129939d, 0.2d, 0.0d, 0.0d, 0.0d, 0.0d, 0.0d, 0.0d);
	c1.printModel();
	double quat[]=  {0,0,0,0};
	cv::Point2f p1;
	double height;
	geometry_msgs::PoseStamped po1;
	while(ros::ok())
	{
		/*std::cout << "Input height:" <<std::endl;
		std::cin >> height;
		
		std::cout << "Input quat orientation (x,y,z,w)" <<std::endl;
		std::cin >> quat[0];
		std::cout << "\n";
		std::cin >> quat[1];
		std::cout << "\n";
		std::cin >> quat[2];
		std::cout << "\n";
		std::cin >> quat[3];
		std::cout << "\n";
		
		std::cout << "Input pixel(x,y)" << std::endl;
		std::cin >> p1.x;
		std::cout << "\n";
		std::cin >> p1.y;
		std::cout << "\n";
		
		po1.pose.position.z = height;
		po1.pose.orientation.x=quat[0];
		po1.pose.orientation.y=quat[1];
		po1.pose.orientation.z=quat[2];
		po1.pose.orientation.w=quat[3];
		
		std::cout << c1.getGroundFeatureWorldLocation(po1, p1) << std::endl;
		
		*/
		break;

	}
	return 0;
}
