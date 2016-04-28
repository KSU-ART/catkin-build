/*****************************************************************
 * Use color_finder to find the LAB color range for what you 
 *  want to detect. Then, store min and max values under the constructor 
 *  with if statements
 * ****************************************************************/

#include "LAB_Object.h"
//LAB -> standard range: ( Lightness (0 - 100), A (-128 - 128), B (-128 - 128) ) 
//OpenCV scales L, A , and B values to a range between 0 and 255.


LAB_Object::LAB_Object()
{
	//set values for default constructor
	setType("Object");
	setColor(Scalar(0,0,0));
}

LAB_Object::LAB_Object(string name){ //constructor with string 'name' as the parameter

	setType(name); //set the color name for the object
	
	
	
	if(name=="green"){

		setLABmin(Scalar(0,93,115));
		setLABmax(Scalar(255,121,150));

		//BGR value for Green:
		setColor(Scalar(0,255,0));
	}
		
	if(name=="red"){

		setLABmin(Scalar(0,228,195));
		setLABmax(Scalar(255,255,200));

		//BGR value for Red:
		setColor(Scalar(0,0,255));
	}
}

LAB_Object::~LAB_Object(void){}

int LAB_Object::getXPos(){

	return LAB_Object::xPos;
}

void LAB_Object::setXPos(int x){

	LAB_Object::xPos = x;
}

int LAB_Object::getYPos(){

	return LAB_Object::yPos;
}

void LAB_Object::setYPos(int y){

	LAB_Object::yPos = y;
}

Scalar LAB_Object::getLABmin(){

	return LAB_Object::LABmin;
}

Scalar LAB_Object::getLABmax(){

	return LAB_Object::LABmax;
}

void LAB_Object::setLABmin(Scalar min){

	LAB_Object::LABmin = min;
}


void LAB_Object::setLABmax(Scalar max){

	LAB_Object::LABmax = max;
}

