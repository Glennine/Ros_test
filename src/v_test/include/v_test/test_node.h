#include<ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include<fstream>
#include <string>
#include <boost/format.hpp>
#include "v_test/estimate_pose.h"
#include "v_test/odometry_calculation.h"
#include "v_test/camera_node.h"
odometry_calculation odometry_calculation;
Camera_node Camera_node;
std::vector<cv::Mat> images;
std::vector<cv::KeyPoint> keypoints_pre,keypoints_cur,keypoints;
std::vector<cv::Mat> descriptors;
std::vector<cv::DMatch> matches;
std::vector<uchar> status;
std::vector<cv::Point2f> keypoints2f_pre,keypoints2f_cur,keypoints_2f;
ofstream fout;
cv::Ptr<cv::CLAHE> clahe;
cv::Mat P; // get the trajectory information
vector<vector<double>> mutil(vector<vector<double>> m1, vector<vector<double>> m2);