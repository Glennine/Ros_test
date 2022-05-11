//
// Created by g on 19/4/2022.
//

#include "v_test/test_node.h"
using namespace std;
//string DATA_PATH = "/home/g/Downloads/2011_09_26/2011_09_26_drive_0005_sync/";
boost::format fmt_file("/home/g/Downloads/2011_09_26/2011_09_26_drive_0005_sync/image_02/data/%010d.png");  
int main(int argc,char **argv){
    int frame_i = 0; // frame number
    ros::init(argc,argv,"ros_image_test"); //初始化节点
    ros::NodeHandle nh; //创建节点句柄
    image_transport::ImageTransport it(nh); //创建图像传输句柄
    image_transport::Publisher pub = it.advertise("camera/image", 1); //创建发布者
    image_transport::Publisher pub_matches = it.advertise("camera/image_match", 10); //创建订阅者
    //image_transport::Publisher pub_opticalflow = it.advertise("camera/image_opticalflow", 10); //创建订阅者
    cv::Mat image_pre,image_cur;
    ros::Rate loop_rate(10); //10HZ
    while (nh.ok()) { // 循环发布图像
        //string file_name = DATA_PATH + "image_02/data/" + ("%010d"%frame).str() + ".png";
        string file_name = (fmt_file%frame_i).str();
        cv::Mat image = cv::imread(file_name, 1); //读取图像
        if (frame_i <2){image_pre = image;}
        image_cur = image;
        //original image publish
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",image).toImageMsg();
        pub.publish(msg);
        //match image publish
        estimate_pose.find_feature_matches(image_pre,image_cur,keypoints_pre,keypoints_cur,matches);
        cv::Mat match_image = 
        estimate_pose.visualizeMatches(image_pre, image_cur, keypoints_pre,keypoints_cur, matches);
        cv_bridge::CvImage matches_viz_cvbridge = cv_bridge::CvImage(std_msgs::Header(), "bgr8", match_image);
        pub_matches.publish(matches_viz_cvbridge.toImageMsg());
        //estimate_pose.find_feature_flow(image_pre,image_cur,keypoints_pre,keypoints_cur,status);
        // cv::Mat optical_flow_match_image = estimate_pose.visualizeOpticalFlow(image_cur, keypoints2f_pre,keypoints2f_cur,status);
        // cv_bridge::CvImage optical_flow_viz_cvbridge = cv_bridge::CvImage(std_msgs::Header(), "bgr8", optical_flow_match_image);
        // pub_opticalflow.publish(optical_flow_viz_cvbridge.toImageMsg());
        ros::spinOnce();
        loop_rate.sleep();
        frame_i++;
        frame_i%=154;
        image_pre=image_cur;
        ROS_INFO_STREAM("publish image " << frame_i);
    }
    //ros publish image to topic


    ROS_INFO_STREAM("publish the image done!");

}
