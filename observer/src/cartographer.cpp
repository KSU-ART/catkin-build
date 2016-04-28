/*****************************************************************
 * This node will take in data from several sources and construct
 *  a map of the UAV's surroundings. This map could later be used
 *  to assist navigaion.
 * **************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Float32.h"

int greenArr[100];
int redArr[100];
float plateAngle;

void greenArrayCallback(const std_msgs::Int16MultiArray::ConstPtr& array)
{
	for (int idx = 0; idx<100; idx++) 
		greenArr[idx] = 0;
	int i = 0;
	// print all the remaining numbers
	for(std::vector<short>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{
		greenArr[i] = *it;
		i++;
	}

	return;
}

void redArrayCallback(const std_msgs::Int16MultiArray::ConstPtr& array)
{
	for (int idx = 0; idx<100; idx++) 
		greenArr[idx] = 0;
	int i = 0;
	// print all the remaining numbers
	for(std::vector<short>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{
		redArr[i] = *it;
		i++;
	}

	return;
}

void plateAngleCallback(const std_msgs::Float32& msg)
{
	plateAngle = msg.data;
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "demoSubscriber");

	ros::NodeHandle n;	

	ros::Subscriber sub1 = n.subscribe("green_loc", 100, greenArrayCallback);
	
	ros::Subscriber sub2 = n.subscribe("red_loc", 100, redArrayCallback);
	
	ros::Subscriber sub3 = n.subscribe("plate_angle", 100, plateAngleCallback);
	
	ros::Rate loop_rate(10);
	  
	ros::spinOnce();

	while(ros::ok()){
		
		/*for(int j = 0; j < 100; j++)
		{
			if (greenArr[j] != 0){
				if (j == 0)
					printf("\nGreen: ");
				printf("%d, ", greenArr[j]);
			}
			
		}
		
		for(int j = 0; j < 100; j++)
		{
			if (redArr[j] != 0){
				if (j == 0);
					printf(" Red: ");
				printf("%d, ", greenArr[j]);
			}	
		}*/
		
		if (plateAngle != 0.0000000000)
			printf("\nPlate Angle: %f\n", plateAngle);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
