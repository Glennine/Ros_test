//
// Created by g on 15/5/2022.
//
#ifndef TEST_ODOMETRY_CALCULATION_H
#define TEST_ODOMETRY_CALCULATION_H
#include "estimate_pose.h"
using namespace cv;
using namespace std;
estimate_pose estimate_pose;
class odometry_calculation{
public:
        odometry_calculation(){}
void init(const cv::Mat &K,
                          std::vector<KeyPoint>& keypoints_1,
                          std::vector<KeyPoint>& keypoints_2,
                          std::vector<cv::DMatch>& matches,
                          Mat &R, Mat &t,
                          vector<Point3d>& points_3d_new);
    
void calculate_match_pose(const cv::Mat &K,
                          std::vector<KeyPoint>& keypoints_1,
                          std::vector<KeyPoint>& keypoints_2,
                          std::vector<DMatch>& matches,
                          Mat& R, Mat& t,
                          vector<Point3d>& points_3d,
                          vector<Point3d>& points_3d_new);

void calculate_optial_pose(const cv::Mat &K,
                           std::vector<Point2f>& pt1,
                           std::vector<Point2f>& pt2,
                           std::vector<uchar> &status,
                           Mat& R, Mat& t,
                           vector<Point3d>& points_3d,
                          vector<Point3d>& points_3d_new);
void calculate_3d_2d_pose(const cv::Mat &K,
                          std::vector<KeyPoint> keypoints_1,
                          std::vector<KeyPoint> keypoints_2,
                          std::vector<DMatch> matches,
                          Mat &R_pre, Mat &t_pre,
                          Mat& R, Mat& t);


};
#endif //TEST_ODOMETRY_CALCULATION_H