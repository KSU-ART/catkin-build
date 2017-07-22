#include "line.h"
#include <math.h>
Line::Line(cv::Vec2f defin, int picHeight, int picWidth){
    // def is a line in the form of (r, theta)
    if(defin[0] < 1){
        this->r = -defin[0];
        this->theta = defin[1]+PI;
    }
    else{
        this->r = defin[0];
        this->theta = defin[1];
    }
    if(this->theta == 0 || this->theta == PI){
        this->slope = 9999;
        this->y_int = -9999;
    }
    else{
        this->slope = -1/tan(this->theta);
        this->y_int = this->r * (sin(this->theta) + ((1/tan(this->theta)) * cos(this->theta)));
    }
    this->pts = this->endPts(picHeight, picWidth);
}

Line::Line(cv::Vec4f pts){
    // pts represents the two points at the edge of the frame
    // in the form (x1,y1,x2,y2)
    this->pts.x1 = pts[0];
    this->pts.y1 = pts[1];
    this->pts.x2 = pts[2];
    this->pts.y2 = pts[3];
    if(this->pts.x1 - this->pts.x2 == 0){
        this->slope = 9999;
    }
    else{
        this->slope = (this->pts.y2-this->pts.y1) / (this->pts.x2-this->pts.x1);
    }
    if(this->pts.x1 <= this->pts.x2){
        if(this->pts.x1 == 0){
            this->y_int = this->pts.y1;
        }
        else{
            this->y_int = this->pts.y1-this->pts.x1*this->slope;
        }
    }
    else{
        if(this->pts.x2 == 0){
            this->y_int = this->pts.y2;
        }
        else{
            this->y_int = this->pts.y1-this->pts.x1*this->slope;
        }
    }
    this->r = abs(this->y_int) / sqrt(pow(this->slope,2) + 1);
    if(this->slope == 0){
        this->theta == PI / 2;
    }
    else if(this->y_int >= 0){
        if(this->slope > 0){
            this->theta = atan2(abs(this->pts.y2-this->pts.y1), abs(this->pts.x2-this->pts.x1)) + PI/2;
        }
        else{
            this->theta = PI/2 - atan2(abs(this->pts.y2-this->pts.y1), abs(this->pts.x2-this->pts.x1));
        }
    }
    else{
        this->theta = atan2(abs(this->pts.y2-this->pts.y1), abs(this->pts.x2-this->pts.x1)) + (3*PI)/2;
    }
}

cv::Vec2f Line::areaBelow(int picHeight, int picWidth){
    // outputs the area below and above a line using calculus
    Points endPts = this->pts;
    float start = endPts.x1;
    float end = endPts.x2;
    float areaAbove = ((-1/tan(pow(end,2)))/2 + this->r * (sin(this->theta)+(1/tan(this->theta))*cos(this->theta))*end)
                -((-1/tan(pow(start,2)))/2 + this->r * (sin(this->theta)+(1/tan(this->theta))*cos(this->theta))*start);
    if(endPts.y2 == picHeight){
        areaAbove += (picWidth - endPts.x2) * picHeight;
    }
    else if(endPts.y1 == picHeight){
        areaAbove += endPts.x1 * picHeight;
    }
    float areaBelow = picHeight * picWidth - areaAbove;
    return cv::Vec2f(areaBelow,areaAbove);
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

Points Line::endPts(int picHeight, int picWidth){
    float start_x = 0;
    float start_y = 0;
    float end_x = 0;
    float end_y = 0;
    if(slope == 0){
        start_y = this->y_int;
        end_y = this->y_int;
        end_x = picWidth;
    }
    else if(this->y_int >= 0 && this->slope > 0){
        if(this->slope * picWidth + this->y_int > picHeight){
            end_x = (picHeight - this->y_int) / this->slope;
            end_y = picHeight;
        }
        else{
            end_x = picWidth;
            end_y = this->slope * picWidth + this->y_int;
        }
        start_y = y_int;
    }
    else if(this->y_int >= 0 && this->slope < 0){
        if(-this->y_int/this->slope > picWidth){
            end_x = picWidth;
            end_y = this->slope * picWidth + this->y_int;
        }
        else{
            end_x = -this->y_int/this->slope;
        }
        if(this->y_int > picHeight){
            start_x = (picHeight - this->y_int) / this->slope;
            start_y = picHeight;
        }
        else{
            start_y = this->y_int;
        }
    }
    else if(this->y_int < 0 && this->slope > 0){
        if(this->slope * picWidth + this->y_int > picHeight){
            end_x = (picHeight - this->y_int) / this->slope;
            end_y = picHeight;
        }
        else{
            end_x = picWidth;
            end_y = this->slope * picWidth + this->y_int;
        }
        start_x = -this->y_int/this->slope;
    }
    Points endpts;
    endpts.x1 = start_x;
    endpts.y1 = start_y;
    endpts.x2 = end_x;
    endpts.y2 = end_y;
    return endpts;
}

float Line::getR(){
    if(this->theta > PI){
        return -this->r;
    }
    return this->r;
}
float Line::getTheta(){
    if(this->theta > PI){ 
        return this->theta - PI;
    }
    return this->theta;
}
float * Line::getSlope(){return &this->slope;}
float * Line::getYInt(){return &this->y_int;}
Points * Line::getPts(){return &this->pts;}
