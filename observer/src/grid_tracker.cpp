
#include "grid_tracker.h"


namespace enc = sensor_msgs::image_encodings;

grid_tracker gt;
image_transport::Publisher grid_intersects_pub;
image_transport::Publisher grid_lines_pub;
	
string source_window = "Source image";
string corners_window = "Corners detected";

/// *********************** Class functions ********************
grid_tracker::grid_tracker()
{
	debug = true;
	
	pos[0] = 0;
	pos[1] = 0;
	velocity[0] = 0;
	velocity[1] = 0;
	
	preIntersects = new vector<Vec2f>();
	curIntersects = new vector<Vec2f>();
}

grid_tracker::~grid_tracker()
{
	delete preIntersects;
	delete curIntersects;
}

void grid_tracker::drawLine(Vec2f line, Mat &img, Scalar rgb)
{
    if(line[1]!=0)
    {
        float m = -1/tan(line[1]);

        float c = line[0]/sin(line[1]);

        cv::line(img, Point(0, c), Point(img.size().width, m*img.size().width+c), rgb);
    }
    else
    {
        cv::line(img, Point(line[0], 0), Point(line[0], img.size().height), rgb);
    }

}

/// Pre: previous vector of intersects,
///		 the delta time taken from previous intersects
/// Post: return a single velocity vector in the average distance traveled by all intersects 
void grid_tracker::distanceTraveled(vector<Vec2f> *pre_intersects, vector<Vec2f> *cur_intersects)
{
	velocity[0] = 0;
	velocity[1] = 0;
	int num_of_points = 0;
	vector<Vec2f>::iterator previous;
	for(previous=pre_intersects->begin();previous!=pre_intersects->end();previous++)
	{
		vector<Vec2f>::iterator current;
		for(current=cur_intersects->begin();current!=cur_intersects->end();current++)
		{
			if (abs((*current)[0] - (*previous)[0]) < 100 &&
				abs((*current)[1] - (*previous)[1]) < 100)
			{
				num_of_points++;
				velocity[0] += (*current)[0] - (*previous)[0];
				velocity[1] += (*current)[1] - (*previous)[1];
				break;
			}
		}
	}
	
	if (num_of_points > 0)
	{
		velocity[0] *= (float)num_of_points;
		velocity[1] *= (float)num_of_points;
	}
	else
	{
		velocity[0] = 0;
		velocity[1] = 0;
	}
	pos[0] += velocity[0];
	pos[1] += velocity[1];
}

/// Post: Find if a point x,y exist in a group. If yes, return group i
bool grid_tracker::inGroup(vector<vector<Vec2f*> > groups, const float x, const float y, int& i)
{
    for (i = 0; i < groups.size(); i++)
	{
		for (int j = 0; j < groups[i].size(); j++)
		{
			if (abs((*groups[i][j])[0] - x) < 0.0002f &&
				abs((*groups[i][j])[1] - y) < 0.0002f)
			{
				return true;
			}
		}
	}
	return false;
}

/// Pre: takes in list of intersection points
/// Post: returns the average points, where the the groups are within a threshold
///			ignors the points near the edge of the screen
void grid_tracker::avaragePoint(vector<Vec2f> *intersects, int groupThresh, int boundryThresh, vector<Vec2f> *averaged)
{
	averaged->clear();
	vector<vector<Vec2f*> > groups;
	
	vector<Vec2f>::iterator current;
    for(current=intersects->begin();current!=intersects->end();current++)
    {
		float x1 = (*current)[0];
		float y1 = (*current)[1];
		int i;
		if (!inGroup(groups,x1,y1,i))
		{
			vector<Vec2f*> new_group;
			new_group.push_back(&(*current));
			groups.push_back(new_group);
			i = groups.size()-1;
		}
		
		vector<Vec2f>::iterator test_iter;
		for(test_iter=current;test_iter!=intersects->end();test_iter++)
		{
			float x2 = (*test_iter)[0];
			float y2 = (*test_iter)[1];
			if (abs(x2-x1) < groupThresh &&
				abs(y2-y1) < groupThresh)
			{
				groups[i].push_back(&(*test_iter));
			}
		}
	}
	
	for (int i = 0; i < groups.size(); i++)
	{
		Vec2f avarage_point;
		for (int j = 0; j < groups[i].size(); j++)
		{
			avarage_point[0] += (*groups[i][j])[0];
			avarage_point[1] += (*groups[i][j])[1];
		}
		avarage_point[0] /= groups[i].size();
		avarage_point[1] /= groups[i].size();
		averaged->push_back(avarage_point);
	}	
}

/// Pre: lines of theta and rho
/// Post: returns a list of intersects on x,y points
void grid_tracker::findIntersectLines(vector<Vec2f> *lines, int angle, vector<Vec2f> *intersects)
{
	intersects->clear();
	vector<Vec2f>::iterator current;
    for(current=lines->begin();current!=lines->end();current++)
    {
		float p1 = (*current)[0];
		float theta1 = (*current)[1];
		vector<Vec2f>::iterator test_iter;
		for(test_iter=current;test_iter!=lines->end();test_iter++)
		{
			float p2 = (*test_iter)[0];
			float theta2 = (*test_iter)[1];
			float denom = sin(theta1-theta2);
			// skip parallel lines
			if (denom == 0 || abs(theta1-theta2) < (90-angle)*CV_PI/180 || abs(theta1-theta2) > (90+angle)*CV_PI/180)
			{
				continue;
			}
			float x = (p2*sin(theta1)-p1*sin(theta2))/denom;
			float y = (p1*cos(theta2)-p2*cos(theta1))/denom;
			Vec2f cross(x,y);
			intersects->push_back(cross);
		}
	}
}

/// pre: lines of theta ond rho
/// post: merges the parallel lines together
void grid_tracker::mergeRelatedLines(vector<Vec2f> *lines, Mat &img)
{
	vector<Vec2f>::iterator current;
    for(current=lines->begin();current!=lines->end();current++)
    {
		if((*current)[0]==0 && (*current)[1]==-100) 
			continue;
		float p1 = (*current)[0];
        float theta1 = (*current)[1];
        
        Point pt1current, pt2current;
        if(theta1>CV_PI*45/180 && theta1<CV_PI*135/180)
        {
            pt1current.x=0;

            pt1current.y = p1/sin(theta1);

            pt2current.x=img.size().width;
            pt2current.y=-pt2current.x/tan(theta1) + p1/sin(theta1);
        }
        else
        {
            pt1current.y=0;

            pt1current.x=p1/cos(theta1);

            pt2current.y=img.size().height;
            pt2current.x=-pt2current.y/tan(theta1) + p1/cos(theta1);

        }
        
        vector<Vec2f>::iterator pos;
        for(pos=lines->begin();pos!=lines->end();pos++)
        {
            if(*current==*pos) 
				continue;
			if(fabs((*pos)[0]-(*current)[0])<20 && fabs((*pos)[1]-(*current)[1])<CV_PI*10/180)
            {
                float p = (*pos)[0];
                float theta = (*pos)[1];
                
                Point pt1, pt2;
                if((*pos)[1]>CV_PI*45/180 && (*pos)[1]<CV_PI*135/180)
                {
                    pt1.x=0;
                    pt1.y = p/sin(theta);
                    pt2.x=img.size().width;
                    pt2.y=-pt2.x/tan(theta) + p/sin(theta);
                }
                else
                {
                    pt1.y=0;
                    pt1.x=p/cos(theta);
                    pt2.y=img.size().height;
                    pt2.x=-pt2.y/tan(theta) + p/cos(theta);
                }
                if( ((double)(pt1.x-pt1current.x)*(pt1.x-pt1current.x) + (pt1.y-pt1current.y)*(pt1.y-pt1current.y)<64*64) &&
					((double)(pt2.x-pt2current.x)*(pt2.x-pt2current.x) + (pt2.y-pt2current.y)*(pt2.y-pt2current.y)<64*64) )
                {
                    // Merge the two
                    (*current)[0] = ((*current)[0]+(*pos)[0])/2;

                    (*current)[1] = ((*current)[1]+(*pos)[1])/2;

                    (*pos)[0]=0;
                    (*pos)[1]=-100;
                }
			}
		}
	}
}

/// Post: runs the main loop for grid tracking
void grid_tracker::grid_algorithm()
{
	/// cornerHarris_demo
	Mat dst, dst2;
	dst = Mat::zeros( src.size(), CV_32FC1 );
	// +- angle from 90 deg to count as a corner
	int intersectAngle = 20;

	/// Detector parameters
	int blockSize = 21;
	int apertureSize = 5;

	/// Detecting corners
	GaussianBlur(src, dst, Size(11,11), 0);
	bitwise_not(dst, dst);
	adaptiveThreshold(dst, dst, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, blockSize, 2);
	bitwise_not(dst, dst);
	Mat kernel = (Mat_<uchar>(3,3) << 0,1,0,1,1,1,0,1,0);
    dilate(dst, dst2, kernel);
    
    int count=0;
    int max=-1;

    Point maxPt;

    for(int y=0;y<dst2.size().height;y++)
    {
        uchar *row = dst2.ptr(y);
        for(int x=0;x<dst2.size().width;x++)
        {
            if(row[x]>=128)
            {

                 int area = floodFill(dst2, Point(x,y), CV_RGB(16,16,64));

                 if(area>max)
                 {
                     maxPt = Point(x,y);
                     max = area;
                 }
            }
        }
    }
    
    floodFill(dst2, maxPt, CV_RGB(255,255,255));
    
    for(int y=0;y<dst2.size().height;y++)
    {
        uchar *row = dst2.ptr(y);
        for(int x=0;x<dst2.size().width;x++)
        {
            if(row[x]==64 && x!=maxPt.x && y!=maxPt.y)
            {
                int area = floodFill(dst2, Point(x,y), CV_RGB(0,0,0));
            }
        }
	}
    
    vector<Vec2f> lines;
    HoughLines(dst2, lines, 1, CV_PI/180, 200);
	mergeRelatedLines(&lines, dst2);
	
	// count intersections
	vector<Vec2f> intersections;
	findIntersectLines(&lines, intersectAngle, &intersections);
	
	curIntersects->clear();
	avaragePoint(&intersections, 80, 0, curIntersects);
	
	distanceTraveled(preIntersects, curIntersects);
	//cout << "pos in pixels:\nX:" << pos[0] << "\nY:" << pos[1] << endl; 
	
    for(int i=0;i<lines.size();i++)
    {
        drawLine(lines[i], dst2, CV_RGB(0,0,128));
    }
    
    for (int i = 0; i < intersections.size(); i++)
	{
		int alpha = 128;
		circle(dst2, Point(intersections[i][0],intersections[i][1]), 10, Scalar(alpha, alpha, alpha));
	}
	
    for (int i = 0; i < curIntersects->size(); i++)
	{
		int alpha = 0;
		circle(src, Point((*curIntersects)[i][0],(*curIntersects)[i][1]), 3, Scalar(alpha, alpha, alpha), 5);
		circle(src, Point((*curIntersects)[i][0],(*curIntersects)[i][1]), 100, Scalar(alpha, alpha, alpha));
	}
	
	/// Post Operations
	preIntersects->clear();
	for (int i = 0; i < curIntersects->size(); i++)
	{
		preIntersects->push_back((*curIntersects)[i]);
	}
	
	/// Showing the result
	if (debug)
	{
		sensor_msgs::ImagePtr srcPtr, dstPtr;
		srcPtr = cv_bridge::CvImage(std_msgs::Header(), "mono8", src).toImageMsg();
		dstPtr = cv_bridge::CvImage(std_msgs::Header(), "mono8", dst2).toImageMsg();
		grid_intersects_pub.publish(srcPtr);
		grid_lines_pub.publish(dstPtr);
	}
	
	waitKey(10);
}

void downCamCB(const sensor_msgs::ImageConstPtr& msg)
{
	/// Load source image and convert it to gray
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	gt.src = cv_ptr->image;
	
	Rect croping(18, 36, gt.src.size().width- 58, gt.src.size().height- 85);
	gt.src = gt.src(croping);
	
	cvtColor( gt.src, gt.src, CV_BGR2GRAY );
	gt.grid_algorithm();

	waitKey(10);
}


/** @function main */
int main( int argc, char** argv )
{
	ros::init(argc, argv, "grid_detect_node");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/usb_cam/image_rect_color", 1, downCamCB);
	
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	grid_intersects_pub = it.advertise("/observer/debug/grid_tracking/intersects", 1);
	grid_lines_pub = it.advertise("/observer/debug/grid_tracking/lines", 1);

	ros::MultiThreadedSpinner spinner(1);
	spinner.spin();

	return(0);
}

