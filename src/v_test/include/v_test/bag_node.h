#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include<iostream>
#include <fstream>
#include<string>
#include <sstream>
#include <stdio.h>
#include "v_test/estimate_pose.h"
#include "v_test/odometry_calculation.h"
vector<double> image_times;
vector<cv::Mat> images;
vector<double> imu_times;
vector<double> a_x;
vector<double> a_y;
vector<double> a_z;
vector<double> w_x;
vector<double> w_y;
vector<double> w_z;

string yaml;
string IMAGE_TOPIC;
string TIME_PATH;
string IMAGE_PATH;