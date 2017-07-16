using namespace std;
using namespace cv;
//class by Kyle
// doesn't actually fit into the context of Stefan's algoithm, but I made it and thought it was cool
// could be useful
class Line{
    float r;
    float theta;
    float slope;
    float y_int;
    public:
        void Line(Vec2f def);
        Vec2f areaBelow(int picHeight, int picWidth);
        static Vec2f areaBelow(int picHeight, int picWidth, float r, float theta);
}
