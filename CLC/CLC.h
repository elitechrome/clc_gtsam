#include <opencv2/opencv.hpp>
#include <vector>
#include <Eigen/Eigen>
using namespace std;
using namespace cv;
using namespace Eigen;

class RectFeature{
public:
    vector<Point2f> imageQuad;
    Eigen::Vector3d rectCenter;
    Eigen::Quaternion<double> rectOrientation;
    double diag_lenght;
    double aspectRatio;
    Mat imagePatch;
    //
};
class CLC{
public:
    CLC(double, double, double, double);
    
private:
    Point2f GetIntersectPoint(const Point2f &p1, const Point2f &p2, const Point2f &p3, const Point2f &p4);
    inline double GetDistance(const Point2f &p1, const Point2f &p2);
    double angle( Point2f pt1, Point2f pt2, Point2f pt0 );

    
public:
    void findSquares( const Mat& image, vector<vector<Point2f> >& squares );
    void drawSquares( Mat& image, const vector<vector<Point2f> >& squares );
    
public:
    bool SetOffCenteredQuad(vector<Point2f> &points);
    bool FindProxyQuadrilateral();
    bool CalcCLC(Vector3d &trans, Quaternion<double> &q, double distance, RectFeature &feature);
    void Visualization(Mat &out);
    bool SortPoints(vector<Point2f> &points);
private:
    int thresh, N;
    const char* wndname;
    RectFeature quadOffCentered;
    RectFeature quadCentered;
    Point2d um, om;
    Point2d w0, w1;
    Point2d u0, u1, u2, u3;
    Point2d wd0, wd1, wm;
    double fx, fy, cx, cy;
};
