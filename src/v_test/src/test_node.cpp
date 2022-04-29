//
// Created by g on 19/4/2022.
//

#include<ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <string>
#include <boost/format.hpp>
using namespace std;
string DATA_PATH = "/home/g/Downloads/2011_09_26/2011_09_26_drive_0005_sync/";
boost::format fmt_file("/home/g/Downloads/2011_09_26/2011_09_26_drive_0005_sync/image_02/data/%010d.png");  
int main(int argc,char **argv){
    int frame = 0; // frame number
    ros::init(argc,argv,"ros_image_test"); //初始化节点
    ros::NodeHandle nh; //创建节点句柄
    image_transport::ImageTransport it(nh); //创建图像传输句柄
    image_transport::Publisher pub = it.advertise("camera/image", 1); //创建发布者
    
    ros::Rate loop_rate(10); //10HZ
    while (nh.ok()) { // 循环发布图像
        //string file_name = DATA_PATH + "image_02/data/" + ("%010d"%frame).str() + ".png";
        string file_name = (fmt_file%frame).str();
        cv::Mat image = cv::imread(file_name, 1); //读取图像
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",image).toImageMsg();
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        frame++;
        frame%=154;
        ROS_INFO_STREAM("publish image " << frame);
    }
    //ros publish image to topic


    ROS_INFO_STREAM("publish the image done!");

}
