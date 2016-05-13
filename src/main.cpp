#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

// iSAM2 requires as input a set set of new factors to be added stored in a factor graph,
// and initial guesses for any new variables used in the added factors
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>


#include <iostream>
#include <cmath>
#include <cstring>
#include <opencv2/opencv.hpp>

#include "CLC.h"
#include <vector>

using namespace cv;
using namespace std;
using namespace Eigen;
using namespace gtsam;
#include <gtsam/nonlinear/NonlinearFactor.h>
/*
class CLCBinaryFactor: public NoiseModelFactor2<Pose3, Pose3> {
  // The factor will hold a measurement consisting of an (X,Y) location
  // We could this with a Point2 but here we just use two doubles
  Pose3 mx_, my_;
public:
  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<CLCBinaryFactor> shared_ptr;
  // The constructor requires the variable key, the (X, Y) measurement value, and the noise model
  CLCBinaryFactor(Key i, Key j, Pose3 x, Pose3 y, const SharedNoiseModel& model):
    NoiseModelFacto2<Pose3, Pose3>(model, j), mx_(x), my_(y) {}
  virtual ~CLCBinaryFactor() {}
  // Using the NoiseModelFactor1 base class there are two functions that must be overridden.
  // The first is the 'evaluateError' function. This function implements the desired measurement
  // function, returning a vector of errors when evaluated at the provided variable value. It
  // must also calculate the Jacobians for this measurement function, if requested.
  // h(x)-z -> between(z,h(x)) for Rot manifold
  Vector evaluateError(const Pose& pose, const Point& point,
      boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
      //H is Jacobian of error function
      // The measurement function for a GPS-like measurement is simple:
      // error_x = pose.x - measurement.x
      // error_y = pose.y - measurement.y
      // Consequently, the Jacobians are:
      // [ derror_x/dx  derror_x/dy  derror_wqwwx/dtheta ] = [1 0 0]
      // [ derror_y/dx  derror_y/dy  derror_y/dtheta ] = [0 1 0]
      //if (H) (*H) = (Matrix(2,3) << 1.0,0.0,0.0, 0.0,1.0,0.0);
      //return (Vector(2) << q.x() - mx_, q.y() - my_);
    Matrix H11, H21, H12, H22;
    boost::optional<Matrix&> H11_ = H1 ? boost::optional<Matrix&>(H11) : boost::optional<Matrix&>();
    boost::optional<Matrix&> H21_ = H1 ? boost::optional<Matrix&>(H21) : boost::optional<Matrix&>();
    boost::optional<Matrix&> H12_ = H2 ? boost::optional<Matrix&>(H12) : boost::optional<Matrix&>();
    boost::optional<Matrix&> H22_ = H2 ? boost::optional<Matrix&>(H22) : boost::optional<Matrix&>();
    Rot y1 = pose.bearing(point, H11_, H12_);
    Vector e1 = Rot::Logmap(measuredBearing_.between(y1));
    double y2 = pose.range(point, H21_, H22_);
    Vector e2 = (Vector(1) << y2 - measuredRange_);
    if (H1) *H1 = gtsam::stack(2, &H11, &H21);
    if (H2) *H2 = gtsam::stack(2, &H12, &H22);
    return concatVectors(2, &e1, &e2);
  }
  // The second is a 'clone' function that allows the factor to be copied. Under most
  // circumstances, the following code that employs the default copy constructor should
  // work fine.
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new CLCBinaryFactor(*this))); }
  // Additionally, we encourage you the use of unit testing your custom factors,
  // (as all GTSAM factors are), in which you would need an equals and print, to satisfy the
  // GTSAM_CONCEPT_TESTABLE_INST(T) defined in Testable.h, but these are not needed below.
}; // UnaryFactor
*/
/* ************************************************************************* */

bool SortPoints(vector<Point2f> &points)
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
            std::cout<<"Point sorting error : it's not quadrilateral."<<std::endl;
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
std::vector<cv::Point2f> keypoints1;
int i = 0;

//callback function
void mouseEvent1(int evt, int x, int y, int flags, void* param){
    cv::Mat *src1 = (cv::Mat*)param;
    cv::Point pot;
    //cv::imshow("src1",*src1);

    if (evt == CV_EVENT_LBUTTONDOWN && i<4){
        //keypoints1[i].pt.x = x;
        //keypoints1[i].pt.y = y;
        pot = cv::Point(x, y);
        cv::circle(*src1, pot, 5, cv::Scalar(0, 255, 0), 4, 5);

        keypoints1.push_back(cv::Point(x, y));
        printf("사각형의 %d번째 꼭지점의 좌표(%d, %d)\n", i + 1, x, y);
        cv::imshow("Image", *src1);
        i++;
    }
}

int main(int argc, char** argv)
{
    // Define the camera observation noise model
    noiseModel::Isotropic::shared_ptr measurementNoise = noiseModel::Isotropic::Sigma(3, 1.0);

    //vector<Pose3> poses = createPoses();

    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    ISAM2 isam(parameters);

    // Create a Factor Graph and Values to hold the new data
    NonlinearFactorGraph graph;
    Values initialEstimate;

    //input parameters of CLC : (fx, fy, cx, cy)
    CLC clc(300,300,512,384);
    std::vector<vector<Point2f> > squares;
    cv::Mat image;
    argv[1] ="/Users/jaemin/Dropbox/EigenEKFSLAM/EigenEKFSLAM/IMG_2658.JPG";
    argv[2] ="/Users/jaemin/Dropbox/EigenEKFSLAM/EigenEKFSLAM/IMG_2660.JPG";
    argv[3] ="/Users/jaemin/Dropbox/EigenEKFSLAM/EigenEKFSLAM/IMG_2664.JPG";
    argv[4] ="/Users/jaemin/Dropbox/EigenEKFSLAM/EigenEKFSLAM/IMG_2658.JPG";
    argv[5] =0;

    vector<Pose3> vecPoses, clcPoses;
    Pose3 currentPose(Rot3::rodriguez(0, 0, 0), Point3(0, 0, 0));
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(0.3),gtsam::Vector3::Constant(0.1))); // 20cm std on x,y, 0.1 rad on theta
    graph.add(PriorFactor<Pose3>(Symbol('x', 0), currentPose, priorNoise)); // add directly to graph


    for( int index = 1; argv[index] != 0; index++ )
    {
        cv::Mat original_image = cv::imread(argv[index], 1);
        if( original_image.empty() ){
            std::cout << "Couldn't load " << argv[index] << std::endl;
            return -1;
        }
        cv::resize(original_image, original_image, Size(1024, 768));
        noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(0.3),gtsam::Vector3::Constant(0.1))); // 20cm std on x,y, 0.1 rad on theta
        Pose3 odometry(Rot3::rodriguez(0, 0, 0), Point3(0, 0, 0));
        currentPose.compose(odometry);
        vecPoses.push_back(currentPose);
        if(index > 1){
        graph.add(BetweenFactor<Pose3>(Symbol('x', index-2), Symbol('x', index-1), odometry, odometryNoise));
        }
        initialEstimate.insert(Symbol('x', index-1), vecPoses[index-1].compose(Pose3(Rot3::rodriguez(0, 0, 0), Point3(0, 0, 0))));

        char inputFlag;
        do{
            keypoints1.clear();
            i = 0;
            original_image.copyTo(image);
            cv::namedWindow("Image", CV_WINDOW_AUTOSIZE);
            cv::imshow("Image", image);
                cv::setMouseCallback("Image", mouseEvent1, &image);

            inputFlag = cv::waitKey();
            if (keypoints1.empty()){
                std::cout << "error, no points are selected.\n";
                i=0;
                continue;
            }

            if (inputFlag == 'd'){
                continue;
            } else{
                //3. CLC Pose Calculation for each rectangle
                int ID_rect=0;
                cv::imshow("Image", original_image);
                std::cout << "Input ID of this rectangle : ";
                //std::cin >> ID_rect;
                ID_rect = 0;
                SortPoints(keypoints1);
                clc.SetOffCenteredQuad(keypoints1);
                clc.FindProxyQuadrilateral();
                Vector3d trans; Eigen::Quaternion<double> q;
                if(!clc.CalcCLC(trans, q))
                {
                    std::cerr << "CLC is NaN" << std::endl;
                    continue;
                }

                Pose3 clcPose(Rot3::quaternion(q.w(),q.x(),q.y(),q.z()), Point3(trans[0], trans[1], trans[2])); // create a measurement for both factors (the same in this case)
                noiseModel::Diagonal::shared_ptr clcPoseNoise = noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(0.3),gtsam::Vector3::Constant(0.1))); // 20cm std on x,y, 0.1 rad on theta
                graph.add(BetweenFactor<Pose3>(Symbol('x', index-1), Symbol('l', ID_rect), clcPose, clcPoseNoise));
                clcPoses.push_back(clcPose);
                if(index==1){
                    //Todo : check landmark symbol is in the graph
                    initialEstimate.insert(Symbol('l', ID_rect), currentPose.compose(clcPose));
                }
                clc.Visualization(image);
                cv::imshow("Image", image);
                cv::waitKey(1);
                inputFlag = cv::waitKey();
            }
        }while(inputFlag != 'f');
        inputFlag = 0;

    }

    // Print
    initialEstimate.print("Initial Estimate:\n");

    // Optimize using Levenberg-Marquardt optimization. The optimizer
    // accepts an optional set of configuration parameters, controlling
    // things like convergence criteria, the type of linear system solver
    // to use, and the amount of information displayed during optimization.
    // Here we will use the default set of parameters.  See the
    // documentation for the full set of parameters.
    LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
    Values result = optimizer.optimize();
    result.print("Final Result:\n");

    // Calculate and print marginal covariances for all variables
    Marginals marginals(graph, result);
    print(marginals.marginalCovariance(Symbol('x',1)), "x1 covariance");
    print(marginals.marginalCovariance(Symbol('x',2)), "x2 covariance");
    print(marginals.marginalCovariance(Symbol('x',3)), "x3 covariance");


    return 0;
}
