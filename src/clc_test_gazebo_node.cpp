#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

#include <iostream>
#include <cmath>
#include <cstring>
#include <vector>

#include "CLC.h"

using namespace cv;
using namespace std;
using namespace Eigen;
using namespace gtsam;

std::vector<Quadrilateral> vecQuad;
vector<Pose3> vecPoses;
int num =0;
std::vector<cv::Point2f> keypoints1;
int i = 0;
char inputFlag;

noiseModel::Isotropic::shared_ptr measurementNoise = noiseModel::Isotropic::Sigma(3, 1.0);

NonlinearFactorGraph graph;
Values initialEstimate;

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

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //Get TF
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try{
        ros::Time now = ros::Time(0);
//        listener.waitForTransform("/camera_frame", "/rect", now, ros::Duration(1));
//        listener.lookupTransform("/camera_frame", "/rect", now, transform);
        listener.waitForTransform("/map", "/zed_optical_frame", now, ros::Duration(1));
        listener.lookupTransform("/map", "/zed_optical_frame", now, transform);
        tf::Matrix3x3 rot = transform.getBasis();
        tf::Vector3 trans = transform.getOrigin();
        cv::Mat extrinsic = (cv::Mat_<double>(3, 4) <<
                              1,  0,  0, 0,
                              0,  1,  0, 0,
                              0,  0,  1, 0);


        cv::Mat R = (cv::Mat_<double>(3, 3)    <<
        rot[0][0], rot[0][1], rot[0][2],
        rot[1][0], rot[1][1], rot[1][2],
        rot[2][0], rot[2][1], rot[2][2]
        );

        cv::Mat tvec = (cv::Mat_<double>(3, 1) << trans[0],trans[1],trans[2]);
        R.copyTo(extrinsic.rowRange(0, 3).colRange(0, 3));
        tvec.copyTo(extrinsic.rowRange(0, 3).col(3));

        std::cout << "extrinsic(tf):\n" << extrinsic  <<std::endl;
        tf::Quaternion rot_qt = transform.getRotation();
        std::cout << "q(tf):\n" << rot_qt.getW() << "," << rot_qt.getX() << "," << rot_qt.getY() << "," << rot_qt.getZ() <<std::endl;
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    try
    {
        num++;
        std::vector<vector<Point2f> > squares;
        cv::Mat original_image = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Mat image;
        original_image.copyTo(image);
        //resize(image, image, Size(640, 480), 0, 0, INTER_LINEAR);
        CLC clc(682.668212890625, 682.668212890625, 628.8949584960938, 385.1582336425781);
        clc.findSquares(image, squares);

        vector<Pose3> clcPoses;

        if( original_image.empty() ){
            ROS_ERROR("No Image");
            return;
        }
        noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(0.3),gtsam::Vector3::Constant(0.1))); // 20cm std on x,y, 0.1 rad on theta
        Pose3 odometry(Rot3::rodriguez(0, 0, 0), Point3(0, 0, 0));
        Pose3 currentPose(Rot3::rodriguez(0, 0, 0), Point3(0, 0, 0));

        currentPose.compose(vecPoses.back());
        currentPose.compose(odometry);
        vecPoses.push_back(currentPose);
        graph.add(BetweenFactor<Pose3>(Symbol('x', num-1), Symbol('x', num), odometry, odometryNoise));
        initialEstimate.insert(Symbol('x', num), vecPoses[num].compose(Pose3(Rot3::rodriguez(0, 0, 0), Point3(0, 0, 0))));

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
            }
            else if(inputFlag == 'q'){
                return;
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
                Eigen::Vector3d trans; Eigen::Quaternion<double> q;
                if(!clc.CalcCLC(trans, q, 1.0))
                {
                    std::cerr << "CLC is NaN" << std::endl;
                    continue;
                }
                // create a measurement for both factors (the same in this case)
                //?????why do I use negetive sign for w, x in quaternion???
                Pose3 clcPose(Rot3::quaternion(q.w(),q.x(),q.y(),q.z()), Point3(trans[0], trans[1], trans[2]));
                // 20cm std on x,y, 0.1 rad on theta
                print(clcPose);
                std::cout <<q.w()<<","<<q.x()<<","<<q.y()<<","<<q.z()<<std::endl;
                noiseModel::Diagonal::shared_ptr clcPoseNoise
                        = noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(0.3),gtsam::Vector3::Constant(0.1)));
                graph.add(BetweenFactor<Pose3>(Symbol('x', num), Symbol('l', ID_rect), clcPose, clcPoseNoise));
                clcPoses.push_back(clcPose);
                if(num==1){
                    //Todo : check landmark symbol is in the graph
                    initialEstimate.insert(Symbol('l', ID_rect), currentPose.compose(clcPose));
                }
                clc.Visualization(image);
                cv::imshow("Image", image);
                inputFlag = cv::waitKey();
                break;
            }
        }while(inputFlag != 'f' || inputFlag != 'q');
        if(inputFlag == 'q')
            return;
        inputFlag = 0;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "clc_test_gazebo");
    ros::NodeHandle nh;
    cv::namedWindow("Image");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/multisense_sl/camera/left/image_raw", 1, imageCallback);

    Pose3 currentPose(Rot3::rodriguez(0, 0, 0), Point3(0, 0, 0));
    vecPoses.push_back(currentPose);

    // 20cm std on x,y, 0.1 rad on theta
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(0.3),gtsam::Vector3::Constant(0.1)));
    graph.add(PriorFactor<Pose3>(Symbol('x', 0), currentPose, priorNoise)); // add directly to graph
    initialEstimate.insert(Symbol('x', 0), Pose3(Rot3::rodriguez(0, 0, 0), Point3(0, 0, 0)));


    while(inputFlag != 'q'){
        ros::spinOnce();
    }
    initialEstimate.print("Initial Estimate:\n");

    LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
    Values result = optimizer.optimize();
    result.print("Final Result:\n");

    // Calculate and print marginal covariances for all variables
//    Marginals marginals(graph, result);
//    print(marginals.marginalCovariance(Symbol('x',1)), "x1 covariance");
//    print(marginals.marginalCovariance(Symbol('x',2)), "x2 covariance");
//    print(marginals.marginalCovariance(Symbol('x',3)), "x3 covariance");
    cv::destroyAllWindows();
    return 0;
}
