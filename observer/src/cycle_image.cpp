#include "opencv2/opencv.hpp"
using namespace cv;
int main(int argc, char** argv)
{
    int camera = 1;

    for(;;)
    {
    	VideoCapture cap(camera);
        for(;;)
        {
	    Mat frame;
            cap >> frame;
            if( frame.empty() ) break; // end of video stream
            imshow("this is you, smile! :)", frame);
            if( waitKey(1) == 27 )
            {
                cap.release();
                
                if(camera > 5)
                    camera = 1;
                else
                    camera++;

                continue;
            }
        } 
    }
    // the camera will be closed automatically upon exit
    // cap.close();
    return 0;
}
