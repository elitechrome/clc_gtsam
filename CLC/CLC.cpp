#include "CLC.h"
bool _compare_min_x(Point2f const &p1, Point2f const &p2) { return p1.x < p2.x; }
bool _compare_min_y(Point2f const &p1, Point2f const &p2) { return p1.y < p2.y; }
CLC::CLC(double _fx, double _fy, double _cx, double _cy)
{
    fx = _fx;
    fy = _fy;
    cx = _cx;
    cy = _cy;
    um =Point2d(_cx, _cy);
}
bool CLC::SortPoints(vector<Point2f> &points)
{
    //    Point2d min_x = *std::min_element(points.begin(), points.end(), &_compare_min_x);
    //    Point2d min_y = *std::min_element(points.begin(), points.end(), &_compare_min_y);
    //    Point2d max_x = *std::max_element(points.begin(), points.end(), &_compare_min_x);
    //    Point2d max_y = *std::max_element(points.begin(), points.end(), &_compare_min_y);
    if(points.size()!=4){
        std::cout<<"The number of Points is must be 4"<<std::endl;
        return false;
    }
    Point2d center;
    Point2d bottom_l, bottom_r, top_r, top_l;
    bool isFoundBl=false, isFoundBr=false, isFoundTr=false, isFoundTl=false;
    
    center.x = (points[0].x+points[1].x+points[2].x+points[3].x)/4;
    center.y = (points[0].y+points[1].y+points[2].y+points[3].y)/4;
    
    for(int i = 0; i < points.size();i++){
        if(((points[i].x-center.x)<0)&&((points[i].y-center.y)>0)&&(!isFoundBl)){
            bottom_l = points[i];
            isFoundBl = true;
            continue;
        }
        else if(((points[i].x-center.x)>0)&&((points[i].y-center.y)>0)&&(!isFoundBr)){
            bottom_r = points[i];
            isFoundBr = true;
            continue;
        }
        else if(((points[i].x-center.x)>0)&&((points[i].y-center.y)<0)&&(!isFoundTr)){
            top_r = points[i];
            isFoundTr = true;
            continue;
        }
        else if(((points[i].x-center.x)<0)&&((points[i].y-center.y)<0)&&(!isFoundTl)){
            top_l = points[i];
            isFoundTl = true;
            continue;
        }
        else{
            return false;
        }
    }
    points.clear();
    points.push_back(bottom_l);
    points.push_back(bottom_r);
    points.push_back(top_r);
    points.push_back(top_l);
    return true;
}
Point2f CLC::GetIntersectPoint(const Point2f &p1, const Point2f &p2, const Point2f &p3, const Point2f &p4)
{
    // Store the values for fast access and easy
    // equations-to-code conversion
    double x1 = p1.x, x2 = p2.x, x3 = p3.x, x4 = p4.x;
    double y1 = p1.y, y2 = p2.y, y3 = p3.y, y4 = p4.y;
    
    double d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    // If d is zero, there is no intersection
    if (d == 0) return Point2f();
    
    // Get the x and y
    double pre = (x1*y2 - y1*x2), post = (x3*y4 - y3*x4);
    double x = (pre * (x3 - x4) - (x1 - x2) * post) / d;
    double y = (pre * (y3 - y4) - (y1 - y2) * post) / d;
    
    // Check if the x and y coordinates are within both lines
    //if (x < min(x1, x2) || x > max(x1, x2) ||
    //x < min(x3, x4) || x > max(x3, x4)) return Point2f();
    //if (y < min(y1, y2) || y > max(y1, y2) ||
    //y < min(y3, y4) || y > max(y3, y4)) return Point2f();
    
    // Return the point of intersection
    Point2f ret;
    ret.x = x;
    ret.y = y;
    return ret;
    
}
inline double CLC::GetDistance(const Point2f &p1, const Point2f &p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}
bool CLC::SetOffCenteredQuad(vector<Point2f> &points)
{
    if (points.size() == 4 ) {
        //push pack points with initializing param.
        om = Point2f(0,0);
        
        quadOffCentered.imageQuad.clear();
        quadCentered.imageQuad.clear();
        quadOffCentered.imageQuad.push_back(points[0]);
        quadOffCentered.imageQuad.push_back(points[1]);
        quadOffCentered.imageQuad.push_back(points[2]);
        quadOffCentered.imageQuad.push_back(points[3]);
        if(!cv::isContourConvex(quadOffCentered.imageQuad))
            return false;
        SortPoints(quadOffCentered.imageQuad);
        return true;
    }
    
    return false;
}


bool CLC::FindProxyQuadrilateral()
{
    om = GetIntersectPoint(quadOffCentered.imageQuad[0], quadOffCentered.imageQuad[2], quadOffCentered.imageQuad[1], quadOffCentered.imageQuad[3]);
    
    w0 = GetIntersectPoint(quadOffCentered.imageQuad[0], quadOffCentered.imageQuad[1], quadOffCentered.imageQuad[2], quadOffCentered.imageQuad[3]);
    w1 = GetIntersectPoint(quadOffCentered.imageQuad[0], quadOffCentered.imageQuad[3], quadOffCentered.imageQuad[1], quadOffCentered.imageQuad[2]);
    
    wd0 = GetIntersectPoint(w0, w1, quadOffCentered.imageQuad[0], quadOffCentered.imageQuad[2]);
    wd1 = GetIntersectPoint(w0, w1, quadOffCentered.imageQuad[1], quadOffCentered.imageQuad[3]);
    
    wm = GetIntersectPoint(w0, w1, um, om);
    
    u0 = GetIntersectPoint(quadOffCentered.imageQuad[0], wm, um, wd0);
    u2 = GetIntersectPoint(quadOffCentered.imageQuad[2], wm, um, wd0);
    u1 = GetIntersectPoint(quadOffCentered.imageQuad[1], wm, um, wd1);
    u3 = GetIntersectPoint(quadOffCentered.imageQuad[3], wm, um, wd1);
    quadCentered.imageQuad.push_back(u0); quadCentered.imageQuad.push_back(u1);
    quadCentered.imageQuad.push_back(u2); quadCentered.imageQuad.push_back(u3);
    return true;
}
bool CLC::CalcCLC(Vector3d &trans, Quaternion<double> &q, double distance, RectFeature &feature, cv::Mat &img)
{
    //determinate that the projective quadrilateral is rectangle in real world
    double l0 = sqrt(pow((quadCentered.imageQuad[0].x - um.x), 2) + pow((quadCentered.imageQuad[0].y - um.y), 2));
    double l2 = sqrt(pow((quadCentered.imageQuad[2].x - um.x), 2) + pow((quadCentered.imageQuad[2].y - um.y), 2));
    double l1 = sqrt(pow((quadCentered.imageQuad[1].x - um.x), 2) + pow((quadCentered.imageQuad[1].y - um.y), 2));
    double l3 = sqrt(pow((quadCentered.imageQuad[3].x - um.x), 2) + pow((quadCentered.imageQuad[3].y - um.y), 2));
    
    double alpha0 = (l0 - l2) / (l0 + l2);
    double alpha1 = (l1 - l3) / (l1 + l3);
    
    double beta = l1 / l0;
    
    bool D = ( (beta >= ( (1-alpha0)/(1+alpha1) )) && (1 >= fabs(alpha1/alpha0) )) || ( (beta <= ( (1-alpha0)/(1+alpha1) )) && (1 <= fabs(alpha1/alpha0) ));
    //cout<<"determinant: "<<D<<endl;
        if(!D){
            //std::cout<<"Couldn't pass the determinant."<<std::endl;
            return false;
        }
    
    double d = sqrt((pow((1 - alpha1)*beta, 2) - pow(1 - alpha0, 2)) / (pow((1 - alpha1)*alpha0*beta, 2) - pow((1 - alpha0)*alpha1, 2)));
    
    double theta0 = acos(d*alpha0);
    double theta1 = acos(d*alpha1);
    
    double x1 = (quadCentered.imageQuad[0].x - quadCentered.imageQuad[2].x);
    double x2 = (quadCentered.imageQuad[1].x - quadCentered.imageQuad[3].x);
    double y1 = (quadCentered.imageQuad[0].y - quadCentered.imageQuad[2].y);
    double y2 = (quadCentered.imageQuad[1].y - quadCentered.imageQuad[3].y);
    double rho = acos((x1*x2 +y1*y2) / (sqrt(x1*x1 + y1*y1)*sqrt(x2*x2+y1*y2)));
    
    if (rho < 0)
        rho = M_PI * 2 + rho;
    double phi = acos(cos(theta0)*cos(theta1) + sin(theta0)*sin(theta1)*cos(rho));
    if(std::isnan(phi))
    {
        //std::cerr << "Crossing Angle is NaN" << std::endl;
        return false;
    }
    //cout<<"Crossing Angle : "<<phi<<endl;
    double psi1, psi2;
    psi1 = atan2(sqrt(pow(cx-(quadOffCentered.imageQuad[0].x+quadOffCentered.imageQuad[2].x)*0.5,2)+pow(cy-(quadOffCentered.imageQuad[0].y+quadOffCentered.imageQuad[2].y)*0.5,2)),(fx+fy)*0.5);
    psi2 = M_PI*0.5 - acos(sin(theta0)*sin(theta1)*sin(rho) / sin(phi));
    d=distance*sin(psi1+psi2)/sin(psi2);

    Point3d pc(d *cos(theta0)*sin(phi) / sin(phi), -d *cos(theta0)*cos(phi) + cos(theta1) / sin(phi), d *sin(theta0)*sin(theta1)*sin(rho) / sin(phi));
    //cout << "Principle point :\n" << pc << endl;
    
#if 1
    Point2f inputQuad[4];
    // Output Quadilateral or World plane coordinates
    Point2f outputQuad[4];
    
    // Lambda Matrix
    Mat H;
    
    // The 4 points that select quadilateral on the input , from top-left in clockwise order
    // These four pts are the sides of the rect box used as input
    double m = d*l0/((fx+fy)*0.5*sin(theta0)+l0*cos(theta0)) /*, phi = 0.61546*2*/;
    
    inputQuad[0] = Point2f( m, 0 );
    inputQuad[1] = Point2f( m*cos(phi), m*sin(phi) );
    inputQuad[2] = Point2f( -m, 0 );
    inputQuad[3] = Point2f( -m*cos(phi), -m*sin(phi) );
    
    outputQuad[0] = Point2f( quadOffCentered.imageQuad[0].x, quadOffCentered.imageQuad[0].y );
    outputQuad[1] = Point2f( quadOffCentered.imageQuad[1].x, quadOffCentered.imageQuad[1].y );
    outputQuad[2] = Point2f( quadOffCentered.imageQuad[2].x, quadOffCentered.imageQuad[2].y );
    outputQuad[3] = Point2f( quadOffCentered.imageQuad[3].x, quadOffCentered.imageQuad[3].y );
    
    // Get the Perspective Transform Matrix i.e. lambda
    H = getPerspectiveTransform( inputQuad, outputQuad );
    
    // Intrinsic
    cv::Mat K = (cv::Mat_<double>(3, 3) <<
                 fx,  0, cx,
                 0, fy, cy,
                 0,  0,  1
                 );
    H = K.inv()*H;
    cv::Mat pose;
    pose = Mat::eye(3, 4, CV_32FC1);      // 3x4 matrix, the camera pose
    float norm1 = (float)norm(H.col(0));
    float norm2 = (float)norm(H.col(1));
    float tnorm = (norm1 + norm2) / 2.0f; // Normalization value
    
    
    cv::normalize(H.col(0), pose.col(0));   // Normalize the rotation, and copies the column to pose
    
    cv::normalize(H.col(1),  pose.col(1));   // Normalize the rotation and copies the column to pose
    
    
    Mat p3 = pose.col(0).cross(pose.col(1));   // Computes the cross-product of p1 and p2
    p3.copyTo(pose.col(2));       // Third column is the crossproduct of columns one and two
    
    H.col(2).copyTo(pose.col(3)) /*/ tnorm*/;  //vector t [R|t] is the last column of pose
    pose.col(3) = pose.col(3)/tnorm;
    
    // Map the OpenCV matrix with Eigen:
    
    Eigen::Matrix3d rot;
    rot<< pose.at<float>(0,0), pose.at<float>(0,1), pose.at<float>(0,2),
    pose.at<float>(1,0), pose.at<float>(1,1), pose.at<float>(1,2),
    pose.at<float>(2,0), pose.at<float>(2,1), pose.at<float>(2,2);
    q=rot;
    //Eigen::Quaternion<double> tmp_q = rot;
    //////////////////////////////////////////////
    ////need to be confirmed
    if(phi > 0 && phi < M_PI/2){
        q.coeffs()[1] *= -1.;
        q.coeffs()[2] *= -1.;
    }
    else {
        q.coeffs()[0] *= -1.;
        q.coeffs()[3] *= -1.;
    }
    //////////////////////////////////////////////
    trans << pose.at<float>(0,3), pose.at<float>(1,3), pose.at<float>(2,3) ;
#endif
#if 0
    ///Perspective-to-Euclidean transformation
    Point2f vecTranslate;
    double t0, t1, s0, s1;
    Point2f us0, us1;
    us0 = GetIntersectPoint(u0, om, um, wd1);
    us1 = GetIntersectPoint(u1, om, um, wd0);
    
    double m=0.1818175;
    
    s0 = GetDistance(us0, um) / l0;
    s1 = GetDistance(us1, um) / l1;
    
    t0 = s0*m*(l0 + l2) / (s0*m*l0 + ((1 - s0)*m+m)*l2);
    t1 = s1*m*(l1 + l3) / (s1*m*l1 + ((1 - s1)*m+m)*l3);
    
    vecTranslate.x = t0 + t1*cos(phi);
    vecTranslate.y = (t1)*sin(phi);
    
    trans[0]=pc.x;
    trans[1]=pc.y;
    trans[2]=pc.z;
    
    Vector3d N1(-pc.x, -pc.y, -pc.z);
    Vector3d N2(0, 0, 1);
    N1.normalize(), N2.normalize();
    Vector3d u = N1.cross(N2);
    u.normalize();
    double alpha = acos(N1.dot(N2)/(N1.norm()*N2.norm()));
    q = AngleAxis<double>(cos(0.5*alpha), sin(0.5*alpha)*u);
    
    //q = Quaternion<double>::FromTwoVectors(N1, N2);
    //(angle, axis.x, axis.y, axis.z);
    q.normalize();
    rot = q.matrix();
    //std::cout << "q: " << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << std::endl;
    std::cout << q.matrix()<<std::endl;
    std::cout << "vector translate:\n" << vecTranslate.x << ", " << vecTranslate.y << std::endl;
    //std::cout << lambda <<std::endl;
#endif

    feature.aspectRatio = phi;
    feature.diag_lenght = m;
    feature.rectCenter = trans;
    feature.rectOrientation = q;
    cv::Mat warpRect, transformMatrix;
    Point2f src[4], dst[4];
    dst[3] = Point2f( 0, 0 );
    dst[0] = Point2f( 0, 128 );
    dst[1] = Point2f( 128, 128 );
    dst[2] = Point2f( 128, 0 );

    src[0] = Point2f( quadOffCentered.imageQuad[0].x, quadOffCentered.imageQuad[0].y );
    src[1] = Point2f( quadOffCentered.imageQuad[1].x, quadOffCentered.imageQuad[1].y );
    src[2] = Point2f( quadOffCentered.imageQuad[2].x, quadOffCentered.imageQuad[2].y );
    src[3] = Point2f( quadOffCentered.imageQuad[3].x, quadOffCentered.imageQuad[3].y );
    transformMatrix = getPerspectiveTransform(src, dst);
    warpPerspective(img, warpRect, transformMatrix, Size(128, 128));
    warpRect.copyTo(feature.imagePatch);
    //imshow("test_patch", warpRect);
    //cv::waitKey(0);
    return true;
}

void CLC::Visualization(cv::Mat &out)
{
    line(out, quadOffCentered.imageQuad[0], quadOffCentered.imageQuad[1], Scalar(0,255,0));
    line(out, quadOffCentered.imageQuad[1], quadOffCentered.imageQuad[2], Scalar(0,255,0));
    line(out, quadOffCentered.imageQuad[2], quadOffCentered.imageQuad[3], Scalar(0,255,0));
    line(out, quadOffCentered.imageQuad[3], quadOffCentered.imageQuad[0], Scalar(0,255,0));
    line(out, quadOffCentered.imageQuad[0], quadOffCentered.imageQuad[2], Scalar(0,255,0));
    line(out, quadOffCentered.imageQuad[1], quadOffCentered.imageQuad[3], Scalar(0,255,0));
    circle(out, quadOffCentered.imageQuad[0], 5, cv::Scalar(0, 255, 0), 4, 5);
    circle(out, quadOffCentered.imageQuad[1], 5, cv::Scalar(0, 255, 0), 4, 5);
    circle(out, quadOffCentered.imageQuad[2], 5, cv::Scalar(0, 255, 0), 4, 5);
    circle(out, quadOffCentered.imageQuad[3], 5, cv::Scalar(0, 255, 0), 4, 5);
    circle(out, um, 5, cv::Scalar(255, 0, 255), 4, 5);
    
    line(out, quadCentered.imageQuad[0], quadCentered.imageQuad[1], Scalar(0,255,255));
    line(out, quadCentered.imageQuad[1], quadCentered.imageQuad[2], Scalar(0,255,255));
    line(out, quadCentered.imageQuad[2], quadCentered.imageQuad[3], Scalar(0,255,255));
    line(out, quadCentered.imageQuad[3], quadCentered.imageQuad[0], Scalar(0,255,255));
    line(out, quadCentered.imageQuad[0], quadCentered.imageQuad[2], Scalar(0,255,255));
    line(out, quadCentered.imageQuad[1], quadCentered.imageQuad[3], Scalar(0,255,255));
    circle(out, quadCentered.imageQuad[0], 5, cv::Scalar(0, 255, 255), 4, 5);
    circle(out, quadCentered.imageQuad[1], 5, cv::Scalar(0, 255, 255), 4, 5);
    circle(out, quadCentered.imageQuad[2], 5, cv::Scalar(0, 255, 255), 4, 5);
    circle(out, quadCentered.imageQuad[3], 5, cv::Scalar(0, 255, 255), 4, 5);
    circle(out, om, 5, cv::Scalar(255, 255, 0), 4, 5);
    
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define DEG2RAD 0.017453293f
int thresh = 50, N = 1;
const char* wndname = "Square Detection Demo";
double CLC::angle( Point2f pt1, Point2f pt2, Point2f pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

void CLC::findSquares( const Mat& image, vector<vector<Point2f> >& squares )
{
    thresh =50, N=11;
    squares.clear();

    Mat pyr, timg, gray0(image.size(), CV_8U), gray;

    // down-scale and upscale the image to filter out the noise
    pyrDown(image, pyr, Size(image.cols/2, image.rows/2));
    pyrUp(pyr, timg, image.size());
    vector<vector<Point> > contours;

    // find squares in every color plane of the image
    for( int c = 0; c < 3; c++ )
    {
        int ch[] = {c, 0};
        mixChannels(&timg, 1, &gray0, 1, ch, 1);

        // try several threshold levels
        for( int l = 0; l < N; l++ )
        {
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            if( l == 0 )
            {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                Canny(gray0, gray, 0, thresh, 5);
                // dilate canny output to remove potential
                // holes between edge segments
                dilate(gray, gray, Mat(), Point(-1,-1));
            }
            else
            {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                gray = gray0 >= (l+1)*255/N;
            }

            // find contours and store them all as a list
            findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

            vector<Point2f> approx;

            // test each contour
            for( size_t i = 0; i < contours.size(); i++ )
            {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if( approx.size() == 4 &&
                    fabs(contourArea(Mat(approx))) > 1000 &&
                    fabs(contourArea(Mat(approx))) < gray.rows*gray.cols*0.6 &&
                    isContourConvex(Mat(approx)) )
                {
                    double maxCosine = 0;

                    for( int j = 2; j < 5; j++ )
                    {
                        // find the maximum cosine of the angle between joint edges
                        double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }

                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                    if( maxCosine < 0.7 ){
                        if(SortPoints(approx))
                            squares.push_back(approx);
                    }
                }
            }
        }
    }
}


void CLC::drawSquares( Mat& image, const vector<vector<Point2f> >& squares )
{
    for( size_t i = 0; i < squares.size(); i++ )
    {
        line(image, squares[i][0], squares[i][1], Scalar(0,0,255));
        line(image, squares[i][1], squares[i][2], Scalar(0,0,255));
        line(image, squares[i][2], squares[i][3], Scalar(0,0,255));
        line(image, squares[i][3], squares[i][0], Scalar(0,0,255));
        
        circle(image, squares[i][0], 5, cv::Scalar(0, 255, 0), 4, 5);
        circle(image, squares[i][1], 5, cv::Scalar(0, 255, 0), 4, 5);
        circle(image, squares[i][2], 5, cv::Scalar(0, 255, 0), 4, 5);
        circle(image, squares[i][3], 5, cv::Scalar(0, 255, 0), 4, 5);
    }
}
