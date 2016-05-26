///Header file for camera_model
#include <cmath>
#include <iostream>
#include <vector>
#include <cv.h>
#include <highgui.h>
#include <atlante.h>
#include <fstream>
#include <geometry_msgs/Pose.h>
#include <string.h>

class cameraModel{
private:
	int image_width, image_height;
	double camera_focal_distance_x, camera_focal_distance_y, 
		camera_center_x_pixels, camera_center_y_pixels, 
		camera_pixel_width_meters, camera_pixel_height_meters, 
		roomba_height;
		
	HTMatrix4 camera_transform_from_drone;
public:
	cameraModel(char camID){
		loadModel(camID);
	}
	cameraModel(char camID, double pH, double pW, int h, int w, double fx, 
			double fy, double x0, double y0, double t1, double t2, double t3, 
			double qw, double qx, double qy, double qz){
		saveModel(camID, pH, pW, h, w, fx, fy, x0, y0, t1, t2, t3, qw, qx, qy, qz);
	}
	cameraModel(){
		makeNewCam();
	}
	void makeNewCam(){
		using namespace std;
		char cameraNum, option = 'n';
		double fx, fy, x0, y0,t1, t2, t3, pHeight, pWidth, qw, qx, qy, qz;
		int width, height;
		do{
			cout << "input camera number (a character): ";
			cin >> cameraNum;
			cout << "\ninput pixel height (meters): ";
			cin >> pHeight;
			cout <<"\ninput pixel width (meters): ";
			cin >> pWidth;
			cout <<"\ninput fx ('k' camera matrix [0][0]): ";
			cin >> fx;
			cout << "\ninput fy ('k' camera matrix [1][1]): ";
			cin >> fy;
			cout << "\ninput x0(cam center x pixel, k[0][2]): ";
			cin >> x0;
			cout << "\ninput y0(cam cneter y pixel, k[1][2]): ";
			cin >> y0;
			cout << "\ninput camera width(pixels): ";
			cin >> width;
			cout << "\ninput camera height (pixels): ";
			cin >> height;
			cout << "\ninput camera translation to IMU\n(ex. (0,0,d) if d meters below imu):\n";
			cin >> t1; cout << endl;
			cin >> t2; cout << endl;
			cin >> t2;
			cout << "\ninput camera rotation quaternion relative to IMU\n"
			<<"(ex. (0,0,0,0) if no rotation relative to IMU and facing straight down):\n";
			cin >> qw; cout << endl;
			cin >> qx; cout << endl;
			cin >> qy;cout << endl;
			cin >> qz;
			cout << "\nis the information correct? Y or N: ";
			cin >> option;
		}while (option != 'y' || option !='Y');
		saveModel(cameraNum, pHeight, pWidth, height, width, fx, fy, x0, y0, t1, t2, t3, qw, qx, qy, qz);
	}
	void saveModel(char camID, double pH, double pW, int h, int w, double fx, 
		double fy, double x0, double y0, double t1, double t2, double t3,
		double qw, double qx, double qy, double qz){
		using namespace std;
		string s1 = "cameraModel"; s1+=camID; s1+=".dat";
		ofstream save;
		save.open(s1.c_str(), ofstream::binary);
		save.write( (char*)&pH , sizeof(double) );
		save.write( (char*)&pW , sizeof(double) );
		save.write( (char*)&h , sizeof(int) );
		save.write( (char*)&w , sizeof(int) );
		save.write( (char*)&fx , sizeof(double) );
		save.write( (char*)&fy , sizeof(double) );
		save.write( (char*)&x0 , sizeof(double) );
		save.write( (char*)&y0 , sizeof(double) );
		save.write( (char*)&t1 , sizeof(double) );
		save.write( (char*)&t2 , sizeof(double) );
		save.write( (char*)&t3 , sizeof(double) );
		save.write( (char*)&qw , sizeof(double) );
		save.write( (char*)&qx , sizeof(double) );
		save.write( (char*)&qy , sizeof(double) );
		save.write( (char*)&qz , sizeof(double) );
		save.close();
		loadModel(camID);
	}
	
	void loadModel(char camID){
		using namespace std;
		int h, w;
		double pH, pW, fx, fy, x0, y0, t1, t2, t3, qw, qx, qy, qz;
		string s1 = "cameraModel"; s1+=camID; s1+=".dat";
		ifstream load;
		load.open(s1.c_str(), ifstream::binary);
		load.read( (char*)&pH , sizeof(double) );
		load.read( (char*)&pW , sizeof(double) );
		load.read( (char*)&h , sizeof(int) );
		load.read( (char*)&w , sizeof(int) );
		load.read( (char*)&fx , sizeof(double) );
		load.read( (char*)&fy , sizeof(double) );
		load.read( (char*)&x0 , sizeof(double) );
		load.read( (char*)&y0 , sizeof(double) );
		load.read( (char*)&t1 , sizeof(double) );
		load.read( (char*)&t2 , sizeof(double) );
		load.read( (char*)&t3 , sizeof(double) );
		load.read( (char*)&qw , sizeof(double) );
		load.read( (char*)&qx , sizeof(double) );
		load.read( (char*)&qy , sizeof(double) );
		load.read( (char*)&qz , sizeof(double) );
		load.close();
		camera_focal_distance_x = fx;
		camera_focal_distance_y = fy;
		camera_center_x_pixels = x0;
		camera_center_y_pixels = y0;
		camera_pixel_width_meters = pW;
		camera_pixel_height_meters = pH;
		Quaternion cam_from_imu(qw, qx, qy, qz);
		camera_transform_from_drone = HTMatrix4(cam_from_imu.getRotMatrix(), Vector3(t1, t2, t3));
		
	}
	
	void printModel(){
		using namespace std;
		cout << "Pixel width (m): " << camera_pixel_width_meters << endl;
		cout << "Pixel height (m): " << camera_pixel_height_meters << endl;
		cout << "Image sensor width (pix): " << image_width << endl;
		cout << "Image sensor height (pix): " << image_height << endl;
		cout << "\nSensor location from IMU (m):\n" << camera_transform_from_drone << "\n";
		cout << "\nCamera Intrinsic Matrix:" << endl;
		cout << camera_focal_distance_x << "\t" << "0\t" << camera_center_x_pixels <<"\n";
		cout << "0\t" << camera_focal_distance_y  << "\t" << camera_center_y_pixels << "\n";
		cout << "0\t" << "0\t" << "1" << endl<<endl;
		
	}
			
							
		
	Vector3 findLinePlaneIntersection(const Vector3 &lineVector, const Vector3 &planeNormal, const Vector3 &planeCenter) {
			double lambda = (planeCenter * planeNormal) / (lineVector * planeNormal);
			return lambda * lineVector;
	}
	
	
	Vector3 getPlateWorldLocation(geometry_msgs::Pose uavPose, cv::Point pixel){
		 
		Quaternion q1(uavPose.orientation.x, uavPose.orientation.y, 
						uavPose.orientation.z, uavPose.orientation.w);
						
		RotMatrix3 rot = q1.getRotMatrix();
		
		HTMatrix4 droneFromWorld = HTMatrix4(rot, Vector3(0, 0, uavPose.position.y - roomba_height));
		HTMatrix4 camFromWorld = droneFromWorld * camera_transform_from_drone;
		HTMatrix4 worldFromCam = camFromWorld.inverse();
		Vector4 floorCenterFromCam4 = worldFromCam * Vector4(0, 0, 0, 1);
		Vector4 floorNormalFromCam4 = worldFromCam * Vector4(0, 0, 1, 1) - floorCenterFromCam4;
		
		Vector3 floorNormalFromCam = Vector3(floorNormalFromCam4.x, 
											floorNormalFromCam4.y, 
											floorNormalFromCam4.z);
						
		Vector3 floorCenterFromCam = Vector3(floorCenterFromCam4.x, 
											floorCenterFromCam4.y, 
											floorCenterFromCam4.z + roomba_height);
		
		
		//find 3D point of feature detected on camera:				
		Vector3 s = Vector3( (pixel.x - camera_center_x_pixels) * camera_pixel_width_meters,
								(pixel.y - camera_center_y_pixels) * camera_pixel_height_meters,
								camera_focal_distance_x * camera_pixel_width_meters );
								
		Vector4 pointOnFloorFromWorld4 = camFromWorld * findLinePlaneIntersection(s.normalize(), 
																				floorNormalFromCam, 
																				floorCenterFromCam);
								
		Vector3 pointOnFloorFromWorld = Vector3(pointOnFloorFromWorld4.x, 
												pointOnFloorFromWorld4.y, 
												pointOnFloorFromWorld4.z);//may want to add roomba height to this if using
		
		return pointOnFloorFromWorld;						
	}
	
	Vector3 getGroundFeatureWorldLocation(geometry_msgs::Pose uavPose, cv::Point pixel){
		 
		Quaternion q1(uavPose.orientation.x, uavPose.orientation.y, 
						uavPose.orientation.z, uavPose.orientation.w);
						
		RotMatrix3 rot = q1.getRotMatrix();
		
		HTMatrix4 droneFromWorld = HTMatrix4(rot, Vector3(0, 0, uavPose.position.y));
		HTMatrix4 camFromWorld = droneFromWorld * camera_transform_from_drone;
		HTMatrix4 worldFromCam = camFromWorld.inverse();
		Vector4 floorCenterFromCam4 = worldFromCam * Vector4(0, 0, 0, 1);
		Vector4 floorNormalFromCam4 = worldFromCam * Vector4(0, 0, 1, 1) - floorCenterFromCam4;
		
		Vector3 floorNormalFromCam = Vector3(floorNormalFromCam4.x, 
											floorNormalFromCam4.y, 
											floorNormalFromCam4.z);
						
		Vector3 floorCenterFromCam = Vector3(floorCenterFromCam4.x, 
											floorCenterFromCam4.y, 
											floorCenterFromCam4.z);
		
		
		//find 3D point of feature detected on camera:				
		Vector3 s = Vector3( (pixel.x - camera_center_x_pixels) * camera_pixel_width_meters,
								(pixel.y - camera_center_y_pixels) * camera_pixel_height_meters,
								camera_focal_distance_x * camera_pixel_width_meters );
								
		Vector4 pointOnFloorFromWorld4 = camFromWorld * findLinePlaneIntersection(s.normalize(), 
																				floorNormalFromCam, 
																				floorCenterFromCam);
								
		Vector3 pointOnFloorFromWorld = Vector3(pointOnFloorFromWorld4.x, 
												pointOnFloorFromWorld4.y, 
												pointOnFloorFromWorld4.z);//may want to add roomba height to this if using
		
		return pointOnFloorFromWorld;						
	}
	
};

///I realized that library has this...
class my_quaternion{
private:
	std::vector< double > q;
public:
	my_quaternion(){set(0.0,0.0,0.0,0.0);}
	my_quaternion(double qw, double qx, double qy, double qz){set(qw,qx,qy,qz);}
	void set(double qw, double qx, double qy, double qz){
		q.clear();
		q.push_back(qw);
		q.push_back(qx);
		q.push_back(qy);
		q.push_back(qz);
	}
	std::vector< double > get(){return q;}
	std::vector< double > change2euler(){
		double qw = q[0], qx = q[1], qy = q[2], qz = q[3],
				qw2=qw*qw, qx2=qx*qx, qy2=qy*qy, qz2=qz*qz, 
				bank, heading, attitude;
		const double pi = 3.1415926535897;
		double unit = qw2 + qx2 + qy2 + qz2;
		double test = qx*qy + qz*qw;
		if (test > 0.499){
			heading = 2 * atan2(qx, qw);
			attitude = pi/2;
			bank = 0;
		}
		else if (test < -0.499){
			heading = -2 * atan2(qx,qw); 
			attitude = -pi/2;
			bank = 0;
		}
		else {
			heading = atan2(2*qy*qw - 2*qx*qz , qx2 - qy2 - qz2 + qw2);
			attitude = asin(2*test/unit);
			bank = atan2(2*qx*qw - 2*qy*qz , -qx2 + qy2 - qz2 + qw2);
		}
		std::vector< double > euler_angs;
		euler_angs.push_back(heading*180/pi);//rotation about y
		euler_angs.push_back(attitude*180/pi);//rotation about z
		euler_angs.push_back(bank*180/pi);//rotation about x
		return euler_angs;
	}
};


	
