#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define USE_IMAGE_TRANSPORT_SUBSCRIBER_FILTER 1

#if USE_IMAGE_TRANSPORT_SUBSCRIBER_FILTER
#  include <image_transport/subscriber_filter.h>
#else
#  include <sensor_msgs/Image.h>
#  include <message_filters/subscriber.h>
#endif

# include "opencv2/core/core.hpp"
# include "opencv2/features2d/features2d.hpp"
# include "opencv2/highgui/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
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


class CLCTestZedNode {
public:
    std::vector<RectFeature> vecQuad;
    vector<Pose3> vecPoses;
    int num;
    unsigned int numLandmark;
    std::vector<cv::Point2f> keypoints1;
    char inputFlag;

    noiseModel::Isotropic::shared_ptr measurementNoise;

    NonlinearFactorGraph graph;
    Values initialEstimate;

public:
    CLCTestZedNode() :
    it_(nh_),
    #if USE_IMAGE_TRANSPORT_SUBSCRIBER_FILTER
    left_image_sub_( it_, "camera/left/image_rect_color", 1 ),
    right_image_sub_( it_, "camera/right/image_rect_color", 1 ),
    #else
    orig_image_sub_( nh_,  "camera/left/image_rect_color", 1 ),
    warp_image_sub_( nh_, "camera/right/image_rect_color", 1 ),
    #endif
    sync( MySyncPolicy( 10 ), left_image_sub_, right_image_sub_ )
    {
    sync.registerCallback( boost::bind( &CLCTestZedNode::callback, this, _1, _2 ) );

    num =0;numLandmark=0;
    measurementNoise = noiseModel::Isotropic::Sigma(3, 1.0);


    Pose3 currentPose(Rot3::rodriguez(0, 0, 0), Point3(0, 0, 0));
    vecPoses.push_back(currentPose);

    // 20cm std on x,y, 0.1 rad on theta
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(0.3),gtsam::Vector3::Constant(0.1)));
    graph.add(PriorFactor<Pose3>(Symbol('x', 0), currentPose, priorNoise)); // add directly to graph
    initialEstimate.insert(Symbol('x', 0), Pose3(Rot3::rodriguez(0, 0, 0), Point3(0, 0, 0)));


    }

    void callback(
    const sensor_msgs::ImageConstPtr& left_msg,
    const sensor_msgs::ImageConstPtr& right_msg
    ){
        ROS_INFO("in callback");
        //Get TF
        tf::TransformListener listener;
        tf::StampedTransform transform;
//        try{
//            ros::Time now = ros::Time(0);
//    //        listener.waitForTransform("/camera_frame", "/rect", now, ros::Duration(1));
//    //        listener.lookupTransform("/camera_frame", "/rect", now, transform);
//            listener.waitForTransform("/map", "/zed_frame", now, ros::Duration(1));
//            listener.lookupTransform("/map", "/zed_frame", now, transform);
//            tf::Matrix3x3 rot = transform.getBasis();
//            tf::Vector3 trans = transform.getOrigin();
//            cv::Mat extrinsic = (cv::Mat_<double>(3, 4) <<
//                                  1,  0,  0, 0,
//                                  0,  1,  0, 0,
//                                  0,  0,  1, 0);


//            cv::Mat R = (cv::Mat_<double>(3, 3)    <<
//            rot[0][0], rot[0][1], rot[0][2],
//            rot[1][0], rot[1][1], rot[1][2],
//            rot[2][0], rot[2][1], rot[2][2]
//            );

//            cv::Mat tvec = (cv::Mat_<double>(3, 1) << trans[0],trans[1],trans[2]);
//            R.copyTo(extrinsic.rowRange(0, 3).colRange(0, 3));
//            tvec.copyTo(extrinsic.rowRange(0, 3).col(3));

//            std::cout << "extrinsic(tf):\n" << extrinsic  <<std::endl;
//            tf::Quaternion rot_qt = transform.getRotation();
//            std::cout << "q(tf):\n" << rot_qt.getW() << "," << rot_qt.getX() << "," << rot_qt.getY() << "," << rot_qt.getZ() <<std::endl;
//        }
//        catch (tf::TransformException ex){
//            ROS_ERROR("%s",ex.what());
//            ros::Duration(1.0).sleep();
//        }
        RNG rng(12345);

        try
        {
            num++;
            std::vector<vector<Point2f> > left_squares, left_matched_squares;
            std::vector<double>left_matched_squares_distance;
            cv::Mat left_image = cv_bridge::toCvShare(left_msg, "bgr8")->image;
            cv::Mat right_image = cv_bridge::toCvShare(right_msg, "bgr8")->image;

            cv::Mat left_image_copied;
            left_image.copyTo(left_image_copied);
            //resize(image, image, Size(640, 480), 0, 0, INTER_LINEAR);
            CLC clc(682.668212890625, 682.668212890625, 628.8949584960938, 385.1582336425781);

            vector<Pose3> clcPoses;

            if( left_image.empty() ){
                ROS_ERROR("No Image");
                return;
            }
            noiseModel::Diagonal::shared_ptr odometryNoise
                    = noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(1),gtsam::Vector3::Constant(3)));
            Pose3 odometry(Rot3::rodriguez(0, 0, 0), Point3(0, 0, 0));
            Pose3 currentPose(Rot3::rodriguez(0, 0, 0), Point3(0, 0, 0));

            currentPose.compose(vecPoses.back());
            currentPose.compose(odometry);
            vecPoses.push_back(currentPose);
            graph.add(BetweenFactor<Pose3>(Symbol('x', num-1), Symbol('x', num), odometry, odometryNoise));
            initialEstimate.insert(Symbol('x', num), vecPoses[num].compose(Pose3(Rot3::rodriguez(0, 0, 0), Point3(0, 0, 0))));

            //3. CLC Pose Calculation for each rectangle
            int ID_rect=0;
            //cv::imshow("Image", left_image);
            clc.findSquares(left_image, left_squares);
            clc.drawSquares(left_image_copied, left_squares);
            //Todo. Square matching

            cv::Ptr<Feature2D> f2d = xfeatures2d::SIFT::create();
            //cv::Ptr<Feature2D> f2d = xfeatures2d::SURF::create();
            //cv::Ptr<Feature2D> f2d = ORB::create();
            // you get the picture, i hope..

            cv::Mat leftImageGray, rightImageGray;
            cv::cvtColor(left_image, leftImageGray,CV_BGR2GRAY);
            cv::cvtColor(right_image, rightImageGray,CV_BGR2GRAY);
            //-- Step 1: Detect the keypoints:
            std::vector<KeyPoint> keypoints_l1, keypoints_l2, keypoints_l3, keypoints_l4, keypoints_lc;
            std::vector<KeyPoint> keypoints_r1, keypoints_r2, keypoints_r3, keypoints_r4, keypoints_rc;
            //f2d->detect( left_image, keypoints_1 );
            //f2d->detect( right_image, keypoints_2 );
            cv::Mat merge_image;
            cv::vconcat(left_image_copied, right_image, merge_image);
            for(int i = 0; i < left_squares.size();i++)
            {
                keypoints_l1.clear();keypoints_l2.clear();keypoints_l3.clear();keypoints_l4.clear();keypoints_lc.clear();
                keypoints_r1.clear();keypoints_r2.clear();keypoints_r3.clear();keypoints_r4.clear();keypoints_rc.clear();
                keypoints_l1.push_back(KeyPoint(left_squares[i][0], 1));
                keypoints_l2.push_back(KeyPoint(left_squares[i][1], 1));
                keypoints_l3.push_back(KeyPoint(left_squares[i][2], 1));
                keypoints_l4.push_back(KeyPoint(left_squares[i][3], 1));
                for(int j=0; j < left_image.cols; j++){
                    keypoints_r1.push_back(KeyPoint(j,left_squares[i][0].y, 1));
                    keypoints_r2.push_back(KeyPoint(j,left_squares[i][1].y, 1));
                    keypoints_r3.push_back(KeyPoint(j,left_squares[i][2].y, 1));
                    keypoints_r4.push_back(KeyPoint(j,left_squares[i][3].y, 1));
                }
                //-- Step 2: Calculate descriptors (feature vectors)
                Mat descriptors_l1, descriptors_l2, descriptors_l3, descriptors_l4;
                Mat descriptors_r1, descriptors_r2, descriptors_r3, descriptors_r4;
                f2d->compute( leftImageGray, keypoints_l1, descriptors_l1 );
                f2d->compute( leftImageGray, keypoints_l2, descriptors_l2 );
                f2d->compute( leftImageGray, keypoints_l3, descriptors_l3 );
                f2d->compute( leftImageGray, keypoints_l4, descriptors_l4 );
                f2d->compute( rightImageGray, keypoints_r1, descriptors_r1 );
                f2d->compute( rightImageGray, keypoints_r2, descriptors_r2 );
                f2d->compute( rightImageGray, keypoints_r3, descriptors_r3 );
                f2d->compute( rightImageGray, keypoints_r4, descriptors_r4 );

                //-- Step 3: Matching descriptor vectors using BFMatcher :
                std::vector< DMatch > matches_l1, matches_l2, matches_l3, matches_l4;
                Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");
                matcher->match(descriptors_l1, descriptors_r1, matches_l1);
                matcher->match(descriptors_l2, descriptors_r2, matches_l2);
                matcher->match(descriptors_l3, descriptors_r3, matches_l3);
                matcher->match(descriptors_l4, descriptors_r4, matches_l4);

                double max_dist_r1 = 0,max_dist_r2 = 0,max_dist_r3 = 0,max_dist_r4 = 0;
                int best_r1=-1,best_r2=-1,best_r3=-1,best_r4=-1;

                for( int i = 0; i < descriptors_l1.rows; i++ )
                { double dist = matches_l1[i].distance;
                    if( dist > max_dist_r1 ){
                        max_dist_r1 = dist;
                        best_r1 = i;
                    }
                    dist = matches_l2[i].distance;
                    if( dist > max_dist_r2 ){
                        max_dist_r2 = dist;
                        best_r2 = i;
                    }
                    dist = matches_l3[i].distance;
                    if( dist > max_dist_r3 ){
                        max_dist_r3 = dist;
                        best_r3 = i;
                    }
                    dist = matches_l4[i].distance;
                    if( dist > max_dist_r4 ){
                        max_dist_r4 = dist;
                        best_r4 = i;
                    }
                }
//                max_dist_r1 = matches_l1[0].distance;
//                max_dist_r2 = matches_l2[0].distance;
//                max_dist_r3 = matches_l3[0].distance;
//                max_dist_r4 = matches_l4[0].distance;

                if(max_dist_r1>200 || max_dist_r2 >200)
                    continue;
                else if(max_dist_r3>200 || max_dist_r4 >200)
                    continue;
                else{
                    Point2f leftCenter(
                        (keypoints_l1[matches_l1[best_r1].queryIdx].pt.x+
                        keypoints_l2[matches_l2[best_r2].queryIdx].pt.x+
                        keypoints_l3[matches_l3[best_r3].queryIdx].pt.x+
                        keypoints_l4[matches_l4[best_r4].queryIdx].pt.x)/4.,
                        (keypoints_l1[matches_l1[best_r1].queryIdx].pt.y+
                        keypoints_l2[matches_l2[best_r2].queryIdx].pt.y+
                        keypoints_l3[matches_l3[best_r3].queryIdx].pt.y+
                        keypoints_l4[matches_l4[best_r4].queryIdx].pt.y)/4.);
                    Point2f rightCenter(
                        (keypoints_r1[matches_l1[best_r1].trainIdx].pt.x+
                        keypoints_r2[matches_l2[best_r2].trainIdx].pt.x+
                        keypoints_r3[matches_l3[best_r3].trainIdx].pt.x+
                        keypoints_r4[matches_l4[best_r4].trainIdx].pt.x)/4.,
                        (keypoints_r1[matches_l1[best_r1].trainIdx].pt.y+
                        keypoints_r2[matches_l2[best_r2].trainIdx].pt.y+
                        keypoints_r3[matches_l3[best_r3].trainIdx].pt.y+
                        keypoints_r4[matches_l4[best_r4].trainIdx].pt.y)/4.);

                    keypoints_lc.push_back(KeyPoint(leftCenter, 1));
                    keypoints_rc.push_back(KeyPoint(rightCenter, 1));
                    Mat descriptors_lc, descriptors_rc;
                    std::vector< DMatch > matches_lc;

                    f2d->compute( leftImageGray, keypoints_lc, descriptors_lc );
                    f2d->compute( rightImageGray, keypoints_rc, descriptors_rc );
                    matcher->match(descriptors_lc, descriptors_rc, matches_lc);
                    if(matches_lc[0].distance > 600){
                        continue;
                    }
                    std::vector<cv::Point2f> tmp_square;
                    tmp_square.push_back(keypoints_l1[matches_l1[best_r1].queryIdx].pt);
                    tmp_square.push_back(keypoints_l2[matches_l2[best_r2].queryIdx].pt);
                    tmp_square.push_back(keypoints_l3[matches_l3[best_r3].queryIdx].pt);
                    tmp_square.push_back(keypoints_l4[matches_l4[best_r4].queryIdx].pt);


                    double tmp_dist =0; double focal_lenght=682.668212890625*0.001; double baseline = 0.12; double centerZ = 0;
                    centerZ = baseline*focal_lenght/fabs(leftCenter.x-rightCenter.x);
                    tmp_dist = sqrt(pow(leftCenter.x*centerZ/focal_lenght,2)+pow(leftCenter.y*centerZ/focal_lenght,2)+centerZ);
                    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
                    circle(merge_image, leftCenter, 5, color, 4, 5);
                    circle(merge_image, Point2f(rightCenter.x,rightCenter.y+left_image.rows), 5, color, 4, 5);
                    cv::line(merge_image,leftCenter,Point2f(rightCenter.x,rightCenter.y+left_image.rows),color);

                    imshow("merge_image", merge_image);
                    left_matched_squares.push_back(tmp_square);
                    left_matched_squares_distance.push_back(tmp_dist);
                    ROS_INFO("matched");
                }

            }



//            left_matched_squares = left_squares;

            //double distance=1;

            for(int i = 0; i < left_matched_squares.size();i++){
                clc.SetOffCenteredQuad(left_matched_squares[i]);
                clc.FindProxyQuadrilateral();
                Eigen::Vector3d trans; Eigen::Quaternion<double> q;
                RectFeature tmp_feature;


                if(!clc.CalcCLC(trans, q, left_matched_squares_distance[i], tmp_feature, left_image))
                {
                   ROS_ERROR("CLC couldn't pass determinant or the result is nan.");
                    continue;
                }
                if(std::isnan(trans[0])){continue;}
                //Todo:Data Association
                vecQuad.push_back(tmp_feature);

                // create a measurement for both factors (the same in this case)
                Pose3 clcPose(Rot3::quaternion(q.w(),q.x(),q.y(),q.z()), Point3(trans[0], trans[1], trans[2]));
                print(clcPose);
                noiseModel::Diagonal::shared_ptr clcPoseNoise
                        = noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(0.3),gtsam::Vector3::Constant(0.1)));
                graph.add(BetweenFactor<Pose3>(Symbol('x', num), Symbol('l', numLandmark), clcPose, clcPoseNoise));
                clcPoses.push_back(clcPose);

                //Todo : check landmark symbol is in the graph
                initialEstimate.insert(Symbol('l', numLandmark++), currentPose.compose(clcPose));

                //clc.Visualization(image);
                //cv::imshow("Image", image);

                inputFlag = cv::waitKey(1);
            }
            cv::Mat merge_patch;
            cv::Mat tmp(1,128,CV_8UC3);

            for(int i = 0;i<vecQuad.size();i++){
                if(i==0){
                    cv::vconcat(tmp, vecQuad[i].imagePatch, merge_patch);
                }
                else{
                    cv::vconcat(merge_patch, vecQuad[i].imagePatch, merge_patch);
                }
            }
            if(!merge_patch.empty()){
                imshow("merge_patch", merge_patch);
                inputFlag = cv::waitKey(1);
            }

        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert from msg to 'bgr8'.");
        }
    }


private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    #if USE_IMAGE_TRANSPORT_SUBSCRIBER_FILTER
    typedef image_transport::SubscriberFilter ImageSubscriber;
    #else
    typedef message_filters::Subscriber< sensor_msgs::Image > ImageSubscriber;
    #endif

    ImageSubscriber left_image_sub_;
    ImageSubscriber right_image_sub_;

    typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, sensor_msgs::Image
    > MySyncPolicy;

    message_filters::Synchronizer< MySyncPolicy > sync;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "clc_test_zed");
    ros::NodeHandle nh;

    CLCTestZedNode mc;


    while(mc.inputFlag != 'q'&&!ros::isShuttingDown()){
        ros::spinOnce();
        boost::this_thread::sleep(boost::posix_time::millisec(10));
    }
    mc.initialEstimate.print("Initial Estimate:\n");

    LevenbergMarquardtOptimizer optimizer(mc.graph, mc.initialEstimate);
    Values result = optimizer.optimize();
    result.print("Final Result:\n");

    // Calculate and print marginal covariances for all variables
//    Marginals marginals(graph, result);
//    print(marginals.marginalCovariance(Symbol('x',1)), "x1 covariance");
//    print(marginals.marginalCovariance(Symbol('x',2)), "x2 covariance");
//    print(marginals.marginalCovariance(Symbol('x',3)), "x3 covariance");
    return 0;
}



