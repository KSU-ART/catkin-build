///Header file for camera_model
#include <cmath>
#include <iostream>
#include <vector>
#include <cv.h>
#include <highgui.h>
#include "atlante.h"
#include <fstream>
#include <geometry_msgs/Pose.h>
#include <string.h>

namespace Projection_space
{
	class cameraModel
	{
	private:
		int image_width, image_height;
		double camera_focal_distance_x, camera_focal_distance_y, 
			camera_center_x_pixels, camera_center_y_pixels, 
			camera_pixel_width_meters, camera_pixel_height_meters, 
			roomba_height;
			
		HTMatrix4 camera_transform_from_drone;
		
	public:
		cameraModel(char camID);
		cameraModel(char camID, double pH, double pW, int h, int w, double fx, 
				double fy, double x0, double y0, double t1, double t2, double t3, 
				double qw, double qx, double qy, double qz);
		cameraModel();
		void makeNewCam();
		void saveModel(char camID, double pH, double pW, int h, int w, double fx, 
			double fy, double x0, double y0, double t1, double t2, double t3,
			double qw, double qx, double qy, double qz);
		
		void loadModel(char camID);
		
		void printModel();
		
		Vector3 findLinePlaneIntersection(const Vector3 &lineVector, const Vector3 &planeNormal, const Vector3 &planeCenter);
		
		Vector3 getPlateWorldLocation(geometry_msgs::Pose uavPose, cv::Point pixel);
		
		Vector3 getGroundFeatureWorldLocation(geometry_msgs::Pose uavPose, cv::Point pixel);
	};

	///I realized that library has this...
	class my_quaternion
	{
	private:
		std::vector< double > q;
	public:
		my_quaternion();
		my_quaternion(double qw, double qx, double qy, double qz);
		void set(double qw, double qx, double qy, double qz);
		std::vector< double > get();
		std::vector< double > change2euler();
	};
}


	
