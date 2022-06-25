#include<ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include<fstream>
#include <string>
#include <stdio.h>
#include <boost/format.hpp>
#include "yaml-cpp/yaml.h"
#include <fstream>

class Camera_node
{
public:
        Camera_node(){}
        int init(int argc,char **argv,cv::VideoCapture cap);
        };