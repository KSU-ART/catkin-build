#include "edge_detect.h"

void edgeDetector::drawLine(cv::Vec2f line, cv::Mat &img, cv::Scalar rgb, int thickness){
    if(line[1]!=0){
        float m = -1/tan(line[1]);
        float c = line[0]/sin(line[1]);
        cv::line(img, cv::Point(0, c), cv::Point(img.size().width, m*img.size().width+c), rgb, thickness);
    }
    else{
        cv::line(img, cv::Point(line[0], 0), cv::Point(line[0], img.size().height), rgb, thickness);
    }
}

void edgeDetector::findEdges(std::vector<cv::Vec2f> *lines, cv::Mat &img, std::vector<cv::Vec3f> *edges, int maxOverhangThresh, int minBufferArea, float lineOffset){
    std::vector<cv::Vec2f>::iterator current;
    for(current=lines->begin();current!=lines->end();current++){
        float p = (*current)[0];
        float theta = (*current)[1];

        float negArea = 0;
        float posArea = 0;

        float negGridArea = 0;
        float posGridArea = 0;

        for(int y=0; y<img.size().height; y++){
            uchar *row = img.ptr(y);
            for(int x=0; x<img.size().width; x++){
                // row[x] is the element value

                // find if the current coordinate is in the positive delta area
                if(x*cos(theta)+y*sin(theta) > p + lineOffset){
                    posArea++;
                    if(row[x] >= 128){
                        posGridArea++;
                    }
                }

                // find if the current coordinate is in the negative delta area
                if(x*cos(theta)+y*sin(theta) < p - lineOffset){
                    negArea++;
                    if(row[x] >= 128){
                        negGridArea++;
                    }
                }
            }
        }
        // check for minBufferArea
        if(negArea > minBufferArea && posArea > minBufferArea){
            if(posGridArea < maxOverhangThresh){
                edges->push_back(cv::Vec3f(p, theta, 1));
            }
            else if(negGridArea < maxOverhangThresh){
                edges->push_back(cv::Vec3f(p, theta, 0));
            }
        }
    }
}

void edgeDetector::mergeRelatedLines(std::vector<cv::Vec2f> *lines, cv::Mat &img){
    // double angleThresh = 10;
    double mergeThresh = 64;
    std::vector<cv::Vec2f>::iterator current;
    for(current = lines->begin();current != lines->end();current++){
        if((*current)[0]<-99 && (*current)[1]<-99)
            continue;
        float p1 = (*current)[0];
        float theta1 = (*current)[1];
        
        cv::Point pt1current, pt2current;
        if(theta1>CV_PI*45/180 && theta1<CV_PI*135/180){
            pt1current.x = 0;
            pt1current.y = p1/sin(theta1);

            pt2current.x = img.size().width;
            pt2current.y = -pt2current.x/tan(theta1) + p1/sin(theta1);
        }
        else{
            pt1current.y = 0;
            pt1current.x = p1/cos(theta1);

            pt2current.y = img.size().height;
            pt2current.x = -(pt2current.y-p1/sin(theta1))*tan(theta1);

        }
        
        std::vector<cv::Vec2f>::iterator pos;
        std::vector<cv::Vec4f> merges;
        merges.push_back(cv::Vec4f(pt1current.x, pt1current.y, pt2current.x, pt2current.y));
        for(pos=lines->begin();pos!=lines->end();pos++){
            if((*pos)[0]<-99 && (*pos)[1]<-99) 
                continue;
            if(*current==*pos) 
                continue;
            
            float p = (*pos)[0];
            float theta = (*pos)[1];
            
            cv::Point pt1, pt2;
            if((*pos)[1]>CV_PI*45/180 && (*pos)[1]<CV_PI*135/180){
                pt1.x = 0;
                pt1.y = p/sin(theta);

                pt2.x = img.size().width;
                pt2.y = -pt2.x/tan(theta) + p/sin(theta);
            }
            else{
                pt1.y = 0;
                pt1.x = p/cos(theta);

                pt2.y = img.size().height;
                pt2.x = -(pt2.y-p/sin(theta))*tan(theta);
            }
            if( ((double)(pt1.x-pt1current.x)*(pt1.x-pt1current.x) + (pt1.y-pt1current.y)*(pt1.y-pt1current.y)<mergeThresh*mergeThresh) &&
                ((double)(pt2.x-pt2current.x)*(pt2.x-pt2current.x) + (pt2.y-pt2current.y)*(pt2.y-pt2current.y)<mergeThresh*mergeThresh) )
            {
                // Merge the two
                merges.push_back(cv::Vec4f(pt1.x, pt1.y, pt2.x, pt2.y));

                (*pos)[0]=-100;
                (*pos)[1]=-100;
            }
        }
        // merge lines
        float count = 0;
        cv::Point average1(0,0);
        cv::Point average2(0,0);
        for(std::vector<cv::Vec4f>::iterator i=merges.begin(); i!=merges.end(); i++){
            average1.x += (*i)[0];
            average1.y += (*i)[1];

            average2.x += (*i)[2];
            average2.y += (*i)[3];
            
            count++;
        }
        average1.x /= count;
        average1.y /= count;
        average2.x /= count;
        average2.y /= count;

        float newP, newTheta;
        if(average2.y - average1.y < 0.000001){
            newTheta = PI/2;
        }
        else{
            newTheta = atan2((average1.x - average2.x), (average2.y - average1.y));
        }
        newP = average1.x*cos(newTheta) + average1.y*sin(newTheta);
        (*current)[0] = newP;
        (*current)[1] = newTheta;
    }
    // remove unwanted lines
    for(int i = lines->size()-1; i >= 0; i--){
        if((*lines)[i][0] < -99 && (*lines)[i][1] < -99){
            lines->erase(lines->begin()+i);
        }
    }
}

void edgeDetector::runGridProcOnce(){
    src = im.get_image();
    if(src.empty()){
        std::cout << "no image" << std::endl;
        return;
    }
    cv::cvtColor(src, src, CV_BGR2GRAY);

    // dst = cv::Mat::zeros( src.size(), CV_32FC1 );

    /// Detecting corners
    cv::GaussianBlur(src, dst, cv::Size(11,11), 0);
    cv::bitwise_not(dst, dst);
    cv::adaptiveThreshold(dst, dst, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, blockSize, 2);
    cv::bitwise_not(dst, dst);
    cv::Mat kernel = (cv::Mat_<uchar>(3,3) << 0,1,0,1,1,1,0,1,0);
    cv::dilate(dst, dst2, kernel);

    int max=-1;
    cv::Point maxPt;

    for(int y = 0; y < dst2.size().height; y++){
        uchar *row = dst2.ptr(y);
        for(int x = 0; x < dst2.size().width; x++){
            if(row[x] >= 128){
                int area = cv::floodFill(dst2, cv::Point(x, y), CV_RGB(16, 16, 64));

                if(area > max){
                    maxPt = cv::Point(x, y);
                    max = area;
                }
            }
        }
    }

    cv::floodFill(dst2, maxPt, CV_RGB(255, 255, 255));

    for(int y = 0; y < dst2.size().height; y++){
        uchar *row = dst2.ptr(y);
        for(int x = 0; x < dst2.size().width; x++){
            if(row[x] == 64 && x != maxPt.x && y != maxPt.y){
                int area = cv::floodFill(dst2, cv::Point(x,y), CV_RGB(0,0,0));
            }
        }
    }

    std::vector<cv::Vec2f> lines;
    cv::HoughLines(dst2, lines, 1, CV_PI/180, 210);
    mergeRelatedLines(&lines, dst2);

    std::vector<cv::Vec3f> edges;
    findEdges(&lines, dst2, &edges, 150, 32000, 20);

    if(DEBUG){
        // for lines
        for(int i=0;i<lines.size();i++){
            drawLine(lines[i], dst2, CV_RGB(0,0,128));
        }
        // for edges
        for(int i=0;i<edges.size();i++){
            drawLine(cv::Vec2f(edges[i][0], edges[i][1]), dst2, CV_RGB(0,0,128), 3);
        }
        std::cout << "display image" << std::endl;
        imshow( source_window, src );
        imshow( corners_window, dst2 );
    }
    cv::waitKey(5);
}

