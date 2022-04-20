//
// Created by g on 19/4/2022.
//

#include<ros/ros.h>

int main(int argc,char **argv){

    ros::init(argc,argv,"hello_ros");

    ros::NodeHandle nh;

    ROS_INFO_STREAM("hello,ROS!");

}