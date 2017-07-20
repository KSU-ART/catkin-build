using namespace std;
using namespace cv;
//class by Kyle
// doesn't actually fit into the context of Stefan's algoithm, but I made it and thought it was cool
// could be useful
union Points_t{
    float x1;
    float y1;
    float x2;
    float y2;
};

class Line{
    
    private:
        Vec4f endPts(int picHeight, int picWidth);
        float r;
        float theta;
        float slope;
        float y_int;
        Points_t pts;

    public:
        Line(Vec2f def, int picHeight, int picWidth);
        Line(Vec4f pts);
        Vec2f areaBelow(int picHeight, int picWidth);
        static Vec2f areaBelow(int picHeight, int picWidth, float r, float theta);
        float * getR();
        float * getTheta();
        float * getSlope();
        float * getYInt();
        Points_t * getPts();
        
};
