#include "line.h"
#include <cmath>
#ifndef Line
#define Line
void Line(Vec2f def){
    // def is a line in the form of (r, theta)
    this.r = def[0];
    this.theta = def[1];
    this.slope = -cot(this.theta);
    this.y_int = this.r * (sin(this.theta) + cot(this.theta) * cos(this.theta));
}

Vec2f areaBelow(int picHeight, int picWidth){
    // outputs the area below and above a line using calculus
    int start = 0;
    int end = 0;
    if(this.y_int >= 0 && this.slope >= 0){
        end = picWidth - 1;
    }
    else if(this.y_int >= 0 && this.slope < 0){
        end = this.r * (tan(this.theta)*sin(this.theta)+cos(this.theta));
    }
    else if(this.y_int < 0 && this.slope > 0){
        start = this.r * (tan(this.theta)*sin(this.theta)+cos(this.theta));
    }
    else {throw Exception e;}
    float areaAbove = (-cot(pow(end,2)/2) + this.r * (sin(this.theta)+cot(this.theta)*cos(this.theta))*end)
                -(-cot(pow(start,2)/2) + this.r * (sin(this.theta)+cot(this.theta)*cos(this.theta))*start);
    float areaBelow = picHeight * picWidth - areaAbove;
    return Vec2f(areaBelow,areaAbove);
}

static Vec2f areaBelow(int picHeight, int picWidth, float r, float theta){
    // same function as other areaBelow but designed to be used without an Line object
    float slope = -cot(theta);
    float y_int = r * (sin(theta) + cot(theta) * cos(theta));
    int start = 0;
    int end = 0;

    if(y_int >= 0 && slope >= 0){
        end = picWidth - 1;
    }
    else if(y_int >= 0 && slope < 0){
        end = r * (tan(theta)*sin(theta)+cos(theta));
    }
    else if(y_int < 0 && slope > 0){
        start = r * (tan(theta)*sin(theta)+cos(theta));
    }
    else {throw Exception e;}
    float areaAbove = (-cot(pow(end,2)/2) + r * (sin(theta)+cot(theta)*cos(theta))*end)
                -(-cot(pow(start,2)/2) + r * (sin(theta)+cot(theta)*cos(theta))*start);
    float areaBelow = picHeight * picWidth - areaAbove;
    return Vec2f(areaBelow,areaAbove);
}

#endif