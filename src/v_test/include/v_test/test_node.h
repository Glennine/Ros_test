#include<ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <string>
#include <boost/format.hpp>
#include "v_test/estimate_pose.h"
estimate_pose estimate_pose;
std::vector<cv::Mat> images;
std::vector<cv::KeyPoint> keypoints_pre,keypoints_cur,keypoints;
std::vector<cv::Mat> descriptors;
std::vector<cv::DMatch> matches;
std::vector<cv::Point2f> keypoints_2f;