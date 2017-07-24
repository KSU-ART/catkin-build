#include "edge_detect.h"
#include "line.h"

cv::Mat * isColor(cv::Mat pic, cv::Vec3f color){
    cv::Mat * newMat = new cv::Mat(pic.size().height, pic.size().width, 0.0);
    for(int y = 0; y < pic.size().height; y++){
        for(int x = 0; x < pic.size().width; x++){
            ///newMat->at<uchar>(y,x) = pic.at<uchar>(x,y,1);
            //float a = (pow((double)(color.val[0]-img.at<uchar>(y,x,0)),2)+((double)(color.val[1]-img.at<uchar>(y,x,1)))+pow((double)(color.val[2]-img.at<uchar>(y,x,2)),2));
            newMat->at<uchar>(y,x) = sqrt(pow((double)(color[0]-pic.at<cv::Vec3b>(y,x).val[0]),2)+ 
                                pow((double)(color[1]-pic.at<cv::Vec3b>(y,x).val[1]),2)+pow((double)(color[2]-pic.at<cv::Vec3b>(y,x).val[2]),2));
        }
    }
    return newMat;
}

double isColor(cv::Vec3f compare, cv::Vec3f color){
    // std::cout << pow(color[0]-compare[0],2) << " " << pow(color[1]-compare[1],2)<< " " << pow(color[2]-compare[2],2) << " : " << sqrt(pow(color[0]-compare[0],2)+pow(color[1]-compare[1],2)+pow(color[2]-compare[2],2));
    // std::cout << "\t" << color[0] << " " << color[1] << " " << color[2];
    return sqrt(pow(color[0]-compare[0],2)+pow(color[1]-compare[1],2)+pow(color[2]-compare[2],2));
}

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

void edgeDetector::findEdges(std::vector<cv::Vec2f> *lines, cv::Mat &img, std::vector<cv::Vec2f> *edges, int maxOverhangThresh, int minBufferArea, float lineOffset){
    std::vector<cv::Vec2f>::iterator current;
    //std::vector<cv::Vec2f> *lines = whittleLines(lines0, PI / 2);
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
                edges->push_back(cv::Vec2f(p, theta));
            }
            else if(negGridArea < maxOverhangThresh){
                edges->push_back(cv::Vec2f(-p, theta));
            }
        }
    }
}

void edgeDetector::mergeRelatedLines(std::vector<cv::Vec2f> *lines, cv::Mat &img){
    // double angleThresh = 10;
    double mergeThresh = 50;
    std::vector<cv::Vec2f>::iterator current;
    for(current = lines->begin();current != lines->end();current++){
        if((*current)[0]<-99 && (*current)[1]<-99)
            continue;
        Line line (*current, img.size().height, img.size().width);
        std::vector<cv::Vec2f>::iterator pos;
        std::vector<Line> merges;
        merges.push_back(line);
        for(pos=lines->begin();pos!=lines->end();pos++){
            Line compare_line (*pos, img.size().height, img.size().width);
            if((*pos)[0]<-99 && (*pos)[1]<-99) 
                continue;
                
            if(*current==*pos) 
                continue;
            if( ((double)pow(line.getPts()->x1-compare_line.getPts()->x1, 2) + pow(line.getPts()->y1-compare_line.getPts()->y1, 2))<mergeThresh*mergeThresh &&
                ((double)pow(line.getPts()->x2-compare_line.getPts()->x2, 2) + pow(line.getPts()->y2-compare_line.getPts()->y2, 2))<mergeThresh*mergeThresh )
            {
                // Merge the two
                merges.push_back(compare_line);

                (*pos)[0]=-100;
                (*pos)[1]=-100;
            }
        }
        // merge lines
        float count = 0;
        cv::Point average1(0,0);
        cv::Point average2(0,0);
        for(std::vector<Line>::iterator i=merges.begin(); i!=merges.end(); i++){
            average1.x += i->getPts()->x1;
            average1.y += i->getPts()->y1;

            average2.x += i->getPts()->x2;
            average2.y += i->getPts()->y2;
            
            count++;
        }
        average1.x /= count;
        average1.y /= count;
        average2.x /= count;
        average2.y /= count;

        Line newLine (cv::Vec4f(average1.x,average1.y,average2.x,average2.y));
        (*current)[0] = newLine.getR();
        (*current)[1] = newLine.getTheta();
    }
    // remove unwanted lines
    for(int i = lines->size()-1; i >= 0; i--){
        if((*lines)[i][0] < -99 && (*lines)[i][1] < -99){
            lines->erase(lines->begin()+i);
        }
    }
}

// returns a vector in x component and y component of the average edges
cv::Vec2f edgeDetector::averageEdge(std::vector<cv::Vec2f> *edges){
    std::vector<cv::Vec2f>::iterator current;
    cv::Vec2f average;
    average[0] = 0;
    average[1] = 0;
    float count = 0;
    for(current=edges->begin(); current!=edges->end(); current++){
        average[0] += (*current)[0]*cos((*current)[1]);
        average[1] += (*current)[0]*sin((*current)[1]);
        count++;
    }
    average[0] /= count;
    average[1] /= count;
    return average;
}

void edgeDetector::runGridProcOnce(){
    src = im.get_image();
    double labThreshold = 20.0;
    cv::Mat lab;
    if(src.empty()){
        std::cout << "no image" << std::endl;
        return;
    }
    //lab = src;
    cv::cvtColor(src, lab, CV_BGR2Lab);
    cv::cvtColor(src, src, CV_BGR2GRAY);
    /*labRed = *isColor(lab, cv::Vec3f((*red.getLMax()+*red.getLMin())/2,(*red.getAMax()+*red.getAMin())/2,(*red.getBMax()+*red.getBMin())/2));
    labGreen = *isColor(lab, cv::Vec3f((*green.getLMax()+*green.getLMin())/2,(*green.getAMax()+*green.getAMin())/2,(*green.getBMax()+*green.getBMin())/2));
    */
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

    // int count_ = 0;
    for(int y = 0; y < dst2.size().height; y++){
        uchar *row = dst2.ptr(y);
        for(int x = 0; x < dst2.size().width; x++){
            // std::cout << count_ << " : "<< x << ", " << y << " ";
            // isColor(lab.at<cv::Vec3b>(y,x), this->vec_red);
            // std::cout << std::endl;
            // count_++;
            // row[x] = 0;
            if(isColor(lab.at<cv::Vec3b>(y,x), this->vec_red) < labThreshold){
                row[x] = 255;
            }
            else if(isColor(lab.at<cv::Vec3b>(y,x), this->vec_green) < labThreshold){
                row[x] = 255;
            }
        }
    }
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
    std::cout << lab.at<cv::Vec3b>(0,0) << std::endl;
    std::cout << vec_red;
    // cv::Vec3f var = *this->vec_red;
    cv::floodFill(dst2, maxPt, CV_RGB(255, 255, 255));

    // std::cout << "labX" << lab.size().width << std::endl;
    // std::cout << "labY" << lab.size().height << std::endl;
    // std::cout << "dst2X" << dst2.size().width << std::endl;
    // std::cout << "dst2Y" << dst2.size().height << std::endl;

    for(int y = 0; y < dst2.size().height; y++){
        uchar *row = dst2.ptr(y);
        for(int x = 0; x < dst2.size().width; x++){
            if(row[x] == 64 && x != maxPt.x && y != maxPt.y){
                int area = cv::floodFill(dst2, cv::Point(x,y), CV_RGB(0,0,0));
            }
        }
    }
    // std::cout << "Didn't break\n";

    std::vector<cv::Vec2f> lines;
    cv::HoughLines(dst2, lines, 1, CV_PI/180, 210);
    mergeRelatedLines(&lines, dst2);

    std::vector<cv::Vec2f> edges;
    findEdges(&lines, dst2, &edges, 150, 32000, 20);

    std_msgs::Bool detected;
    if(edges.size() > 0){
        detected.data = true;
        detectPub.publish(detected);

        cv::Vec2f arenaVector;
        arenaVector = averageEdge(&edges);

        std_msgs::Float32 msg;
        msg.data = arenaVector[0];
        pubx.publish(msg);
        msg.data = arenaVector[1];
        puby.publish(msg);
    }
    else{
        detected.data = false;
        detectPub.publish(detected);
    }

    if(DEBUG){
        // for lines
        for(int i=0;i<lines.size();i++){
            drawLine(lines[i], dst2, CV_RGB(0,0,128));
        }
        // for edges
        for(int i=0;i<edges.size();i++){
            drawLine(edges[i], dst2, CV_RGB(0,0,128), 3);
        }
        std::cout << "display image" << std::endl;
        imshow( source_window, src );
        imshow( corners_window, dst2 );
    }
    cv::waitKey(5);
}

