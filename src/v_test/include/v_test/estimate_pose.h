//
// Created by g on 16/4/2022.
//
#ifndef TEST_ESTIMATE_POSE_H
#define TEST_ESTIMATE_POSE_H
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <se3.hpp>

using namespace cv;
using namespace std;
class estimate_pose{
public:
        estimate_pose(){}
void find_feature_matches(const cv::Mat& img_1, const cv::Mat& img_2,
                          std::vector<cv::KeyPoint>& keypoints_1,
                          std::vector<cv::KeyPoint>& keypoints_2,
                          std::vector<cv::DMatch>& matches);
void pose_estimation_2d2d(const cv::Mat &K,
                          std::vector<cv::KeyPoint> keypoints_1,
                          std::vector<cv::KeyPoint> keypoints_2,
                          std::vector<cv::DMatch> matches, cv::Mat& R, cv::Mat& t);

void triangulation(//must have translation vector
        const vector<KeyPoint> &keypoint_1,
        const vector<KeyPoint> &keypoint_2,
        const std::vector<DMatch> &matches,
        const Mat &R, const Mat &t,
        vector<Point3d> &points);

Point2d pixel2camera ( const Point2d& p, const Mat& K );

//3d-2d pose estimate
//create BA (slef define the edge type) 不光考虑了单应矩阵的计算误差，也考虑了图像点的测量误差，所以其精度会更高
typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> Vector2dVector;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> Vector3dVector;
void BA_g2o(const Vector3dVector& points_3d, const Vector2dVector& points_2d,const Mat &K, Sophus::SE3d &pose);
void BA_GaussNewton(const Vector3dVector& points_3d, const Vector2dVector& points_2d,
                    const Mat &K, Sophus::SE3d &pose);
cv::Mat visualizeMatches(const Mat &img_1, const Mat &img_2, 
                          std::vector<cv::KeyPoint> keypoints_1,
                          std::vector<cv::KeyPoint> keypoints_2,
                          std::vector<cv::DMatch> matches);
};
#endif //TEST_ESTIMATE_POSE_H
