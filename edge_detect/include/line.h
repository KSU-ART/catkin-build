#define PI 3.14159265

#ifndef Line_STUFF
#define Line_STUFF

#include <cv_bridge/cv_bridge.h>

//class by Kyle
struct Points{
    float x1;
    float y1;
    float x2;
    float y2;
};

class Line{
    
    private:
        float r;
        float theta;
        float slope;
        float y_int;
        Points pts;
        Points endPts(int picHeight, int picWidth);

    public:
        Line(cv::Vec2f defin, int picHeight, int picWidth);
        Line(cv::Vec4f pts);
        cv::Vec2f areaBelow(int picHeight, int picWidth);
        //static cv::Vec2f areaBelow(int picHeight, int picWidth, float r, float theta);
        float getR();
        float getTheta();
        float * getSlope();
        float * getYInt();
        Points * getPts();
};


#endif