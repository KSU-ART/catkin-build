#include <math.h>
#include <iostream>
#include <string>
#include <fstream>

using namespace std;

class Color{
public:
	Color();
	Color(char inColor);
	int lMin; 
	int aMin;	
	int bMin;	
	int lMax;
	int aMax;
	int bMax;
};

Color::Color(){
}


Color::Color(char inColor){
	switch(inColor)
	{
		case 'g':
		{
			ifstream green;
			green.open("green.txt");
			if (green.is_open())
			{
				green >> lMin; 
				green >> aMin;	
				green >> bMin;	
				green >> lMax;
				green >> aMax;
				green >> bMax;
			}
			green.close();
		}
			break;
			
		case 'r':
		{
			ifstream red;
			red.open("red.txt");
			if (red.is_open())
			{
				red >> lMin; 
				red >> aMin;	
				red >> bMin;	
				red >> lMax;
				red >> aMax;
				red >> bMax;
			}
			red.close();
		}
			break;
			
		default:
			lMin = 0; 
			aMin = 0;	
			bMin = 0;	
			lMax = 180;
			aMax = 255;
			bMax = 255;
	}
}
