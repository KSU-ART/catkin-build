///cartographer
#include <iostream>
#include "camera_model.h"

using namespace std;
using namespace cv;

class cartographer{
	cv::Point p1;
	geometry_msgs::Pose uavPose;
	Vector3 loc;
public:
	cartographer(){
		projection_space::cameraModel c1;
		while (true)
		{
			cout << "\n\n\n";
			c1.printModel();
			cout << "\n\n\n";
			cout << "\nX pixel: ";
			cin >> p1.x;
			cout <<  "\nY pixel: ";
			cin >> p1.y;
			cout << "\nqX: ";
			cin >> uavPose.orientation.x;
			cout << "\nqY: ";
			cin >> uavPose.orientation.y;
			cout << "\nqZ: ";
			cin >> uavPose.orientation.z;
			cout << "\nqW: ";
			cin >> uavPose.orientation.w;
			cout << "\nposition X: ";
			cin >> uavPose.position.x;
			cout << "\nposition Y: ";
			cin >> uavPose.position.y;
			cout << "\nposition Z: ";
			cin >> uavPose.position.z;
			loc = c1.getGroundFeatureWorldLocation(uavPose, p1); 
			cout << "\n\n " << 	loc << "\n\n";
		}
	}
};	
		
