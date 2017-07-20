#include "line.h"
#include <cmath>
#ifndef Line
#define Line
Line(Vec2f def, int picHeight, int picWidth){
    // def is a line in the form of (r, theta)
    this.r = def[0];
    this.theta = def[1];
    this.slope = -cot(this.theta);
    this.y_int = this.r * (sin(this.theta) + cot(this.theta) * cos(this.theta));
    this.pts = this.endPoints(picHeight, picWidth);
}

Line(Vec4f pts){
    // pts represents the two points at the edge of the frame
    // in the form (x1,y1,x2,y2)
    this.pts.x1 = pts[0];
    this.pts.y1 = pts[1];
    this.pts.x2 = pts[2];
    this.pts.y2 = pts[3];
    if(pts[0] - pts[2] == 0){
        this.slope = 9999;
    }
    else{
        this.slope = (pts[3]-pts[1]) / (pts[2]-pts[0]);
    }
    if(pts[0] <= pts[2]){
        this.y_int = pts[1];
    }
    else{
        this.y_int = pts[3];
    }
    this.r = (pts[2] * pts[1] - pts[3] * pts[0]) / (sqrt(pow(pts[3]-pts[1],2)+pow(pts[2]-pts[0])));
    if(this.y_int == 0){
        this.theta == NULL;
    }
    else if(this.slope == 0){
        this.theta == PI / 2;
    }
    else if(this.y_int > 0){
        if(this.slope > 0){
            this.theta = atan2(abs(pts[3]-pts[1]), abs(pts[2]-pts[0])) + PI/2;
        }
        else{
            this.theta = PI/2 - atan2(abs(pts[3]-pts[1]), abs(pts[2]-pts[0]))
        }
    }
    else{
        this.theta = atan2(abs(pts[3]-pts[1]), abs(pts[2]-pts[0])) + (3*PI)/4
    }
}

Vec2f areaBelow(int picHeight, int picWidth){
    // outputs the area below and above a line using calculus
    Vec4f endPts = this.endPts(picHeight, picWidth);
    float start = endPts[0];
    float end = endPts[2];
    float areaAbove = (-cot(pow(end,2)/2) + this.r * (sin(this.theta)+cot(this.theta)*cos(this.theta))*end)
                -(-cot(pow(start,2)/2) + this.r * (sin(this.theta)+cot(this.theta)*cos(this.theta))*start);
    if(endPts[3] == picHeight){
        areaAbove += (picWidth - endPts[2]) * picHeight;
    }
    else if(endPts[1] == picHeight){
        areaAbove += endPts[0] * picHeight;
    }
    float areaBelow = picHeight * picWidth - areaAbove;
    return Vec2f(areaBelow,areaAbove);
}
/*
static Vec2f areaBelow(int picHeight, int picWidth, float r, float theta){
    // same function as other areaBelow but designed to be used without an Line object
    // don't use doesn't work
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
*/

Points2_t endPts(int picHeight, int picWidth){
    float start_x = 0;
    float start_y = 0;
    float end_x = 0;
    float end_y = 0;
    if(slope == 0){
        start_y = this.y_int;
        end_y = this.y_int;
        end_x = picWidth;
    }
    else if(this.y_int >= 0 && this.slope > 0){
        if(this.slope * picWidth + this.y_int > picHeight){
            end_x = (picHeight - this.y_int) / this.slope;
            end_y = picHeight;
        }
        else{
            end_x = picWidth;
            end_y = this.slope * picWidth + this.y_int;
        }
        start_y = y_int;
    }
    else if(this.y_int >= 0 && this.slope < 0){
        if(this.r * (tan(this.theta)*sin(this.theta)+cos(this.theta)) > picWidth){
            end_x = picWidth;
            end_y = this.slope * picWidth + this.y_int;
        }
        else{
            end_x = this.r * (tan(this.theta)*sin(this.theta)+cos(this.theta));
        }
        if(this.y_int > picHeight){
            start_x = (picHeight - this.y_int) / this.slope;
            start_y = picHeight;
        }
        else{
            start_y = this.y_int;
        }
    }
    else if(this.y_int < 0 && this.slope > 0){
        if(this.slope * picWidth + this.y_int > picHeight){
            end_x = (picHeight - this.y_int) / this.slope;
            end_y = picHeight;
        }
        else{
            end_x = picWidth;
            end_y = this.slope * picWidth + this.y_int;
        }
        start_x = this.r * (tan(this.theta)*sin(this.theta)+cos(this.theta));
    }
    else {throw Exception e;}
    Points_t endpts;
    endpts.x1 = start_x;
    endpts.y1 = start_y;
    endpts.x2 = end_x;
    endpts.y2 = end_y;
    return endpts;
}

float * getR(){return &this.r;}
float * getTheta(){return &this.theta;}
float * getSlope(){return &this.slope;}
float * getYInt(){return &this.y_int;}
Points_t * getPts(){return &this.pts;}

#endif