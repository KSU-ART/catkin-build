#pragma once
#include <string>
#include <cv.h>
#include <highgui.h>
using namespace std;
using namespace cv;

class LAB_Object
{
public:
	LAB_Object();
	~LAB_Object(void);

	LAB_Object(string name);

	int getXPos();
	void setXPos(int x);

	int getYPos();
	void setYPos(int y);

	Scalar getLABmin();
	Scalar getLABmax();

	void setLABmin(Scalar min);
	void setLABmax(Scalar max);

	string getType(){return type;}
	void setType(string t){type = t;}

	Scalar getColor(){ //get function for color of object
		return Color;
	}
	void setColor(Scalar c){ //the color of the object is purely cosmetic
		Color = c;
	}

private:

	int xPos, yPos; //position of the object for x and y
	string type; //name of the color
	Scalar LABmin, LABmax; //color value minimum and maximum for the object
	Scalar Color; //values of the color to draw
};

