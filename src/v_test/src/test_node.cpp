//
// Created by g on 19/4/2022.
//
#include "v_test/test_node.h"
using namespace std;
//boost::format fmt_file("/home/g/Downloads/2011_09_26/2011_09_26_drive_0005_sync/image_02/data/%010d.png");  
boost::format fmt_file("/home/g/Downloads/07/image_0/%06d.png");
string file="/home/g/Downloads/test_data/image/";
//Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1); //相机内参
Mat K = (Mat_<double>(3, 3) << 707.0912, 0, 601.8873, 0, 707.0912, 183.1104, 0, 0, 1); //相机内参
String pose_file = "/home/g/CLionProjects/Ros_test/src/v_test/results/VO_result.txt";
int pose_flag = 1,camera=0;
bool normal_file = true;
string file_name;
int main(int argc,char **argv){
    if(camera==1){
    cv::VideoCapture cap(0);
    if(cap.isOpened())
    {
        cout<<"camera open success"<<endl;
        Camera_node.init(argc,argv,cap);
    }
    else cout<<"camera open failed"<<endl;}
    if (pose_flag==1){
    fout.open(pose_file);
    int frame_i = 0; // frame number
    ros::init(argc,argv,"ros_image_test"); //初始化节点
    ros::NodeHandle nh; //创建节点句柄
    image_transport::ImageTransport it(nh); //创建图像传输句柄
    image_transport::Publisher pub = it.advertise("camera/image", 1); //创建发布者
    image_transport::Publisher pub_matches = it.advertise("camera/image_match", 10); //创建订阅者
    image_transport::Publisher pub_opticalflow = it.advertise("camera/image_opticalflow", 10); //创建订阅者
    cv::Mat image_pre,image_cur;
    cv::Mat R,t,R_pre,t_pre;
    vector<Point3d> points_3d,points_3d_new;
    ros::Rate loop_rate(10); //10HZ
    P = (Mat_<double>(4,4)<<1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1);
    while (nh.ok()) { // 循环发布图像
        //string file_name = DATA_PATH + "image_02/data/" + ("%010d"%frame).str() + ".png";
        if(normal_file)
        {file_name = file + std::to_string(frame_i) + ".png";}
        else{
        file_name = (fmt_file%frame_i).str();}
        cv::Mat image = cv::imread(file_name,0); //读取图像 gray 0 color 1
        if (frame_i==0){
            //init R,t
            image_cur = image;
            R = (Mat_<double>(3, 3) << 1, 0, 0, 0, 0, 1, 0, 0, 0);
            t = (Mat_<double>(3, 1) << 0, 0, 0);
            ROS_INFO("init R,t");}
        else if (frame_i==1) 
        {   
            image_cur = image;
            estimate_pose.find_feature_matches(image_pre,image_cur,keypoints_pre,keypoints_cur,matches);
            odometry_calculation.init(K,keypoints_pre,keypoints_cur,matches,R_pre,t_pre,R,t,points_3d_new);
            //need change to match count over 50 to continue.
            ROS_INFO_STREAM("R: " << R);
            ROS_INFO_STREAM("t: " << t);
        }
        else{
        image_cur = image;
        ROS_INFO_STREAM("frame_i: " << frame_i);
        //original image publish
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8",image).toImageMsg();
        pub.publish(msg);
        ROS_INFO_STREAM("image publish");
        // clahe = cv::createCLAHE(2.0);
        // clahe->apply(image,image); //对图像进行直方图均衡化
        //match image publish
        //calculate the pose of the camera
        if (pose_flag == 0){
            estimate_pose.find_feature_matches(image_pre,image_cur,keypoints_pre,keypoints_cur,matches);
            cv::Mat match_image = 
            estimate_pose.visualizeMatches(image_pre, image_cur, keypoints_pre,keypoints_cur, matches);
            cv_bridge::CvImage matches_viz_cvbridge = cv_bridge::CvImage(std_msgs::Header(), "bgr8", match_image);
            pub_matches.publish(matches_viz_cvbridge.toImageMsg());
            ROS_INFO_STREAM("use match to calculate the pose");
            odometry_calculation.calculate_match_pose(K,keypoints_pre,keypoints_cur,matches,R_pre,t_pre,R,t,points_3d,points_3d_new);
        }else if (pose_flag == 1){
            estimate_pose.find_feature_flow(image_pre,image_cur,keypoints2f_pre,keypoints2f_cur,status);
            cv::Mat optical_flow_match_image = estimate_pose.visualizeOpticalFlow(image_cur, keypoints2f_pre,keypoints2f_cur,status);
            cv_bridge::CvImage optical_flow_viz_cvbridge = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8,optical_flow_match_image);
            pub_opticalflow.publish(optical_flow_viz_cvbridge.toImageMsg());
            ROS_INFO_STREAM("use optical flow to calculate the pose");
            odometry_calculation.calculate_optial_pose(K,keypoints2f_pre,keypoints2f_cur,status,R_pre,t_pre,R,t,points_3d,points_3d_new);
        }  
        ROS_INFO_STREAM("R: " << R);
        ROS_INFO_STREAM("t: " << t);
        
        vector<vector<double>> temp_R{{R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2)},\
        {R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2)},{R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2)}};
        vector<vector<double>> temp_t{{t.at<double>(0,0)},{t.at<double>(1,0)},{t.at<double>(2,0)}};
        for (int k = 0; k < temp_R.size() -1; k++)
        {
            for (int j = k + 1; j < temp_R.size(); j++)
            {
                swap(temp_R[k][j], temp_R[j][k]);
            }
        }
        vector<vector<double>> temp_ = mutil(temp_R,temp_t);
        cv::Mat P_new = (Mat_<double>(4,4)<<temp_R[0][0],temp_R[0][1],temp_R[0][2],temp_t[0][0],\
        temp_R[1][0],temp_R[1][1],temp_R[1][2],temp_[1][0],temp_R[2][0],temp_R[2][1],temp_R[2][2],\
        temp_[2][0],0,0,0,1);
        //P_new[0:3][0:3] = R.t();
        //P_new[0:3][3] = (-R.t())*t;
        P = P*P_new;
        //change P to R
        fout << P.at<double>(0,0)<<" "<<P.at<double>(0,1)<<" "<<P.at<double>(0,2)<<" "<<P.at<double>(0,3)<<" "<<P.at<double>(1,0)<<" "<<P.at<double>(1,1)<<" "<<P.at<double>(1,2)\
        <<" "<<P.at<double>(1,3)<<" "<<P.at<double>(2,0)<<" "<<P.at<double>(2,1)<<" "<<P.at<double>(2,2)<<" "<<" "<<P.at<double>(2,3);
        // fout<<P.at<double>(0,3)<<" "<<P.at<double>(1,3)<<" "<<P.at<double>(2,3);
        //fout<<P;
        ROS_INFO_STREAM("each pose:"<< P);}
        // fout <<t.at<double>(0,0)<<" "<<t.at<double>(1,0)<<" "<<t.at<double>(2,0);
        fout << endl;
        ros::spinOnce();
        loop_rate.sleep();
        frame_i++;
        //frame_i%=154;
        R_pre = R;
        t_pre = t;
        points_3d = points_3d_new;
        image_pre=image_cur;
        ROS_INFO_STREAM("publish image " << frame_i);
    }
    //ros publish image to topic
    fout << flush; fout.close();

    ROS_INFO_STREAM("publish the image done!");}

}

vector<vector<double>> mutil(vector<vector<double>> m1, vector<vector<double>> m2) {
    //两矩阵相乘(but in this problem, we need -R to calculate)
    int m = m1.size();
    int n = m1[0].size();
    int p = m2[0].size();
    vector<vector<double>> array;
    vector<double> temparay;
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < p; j++) {
            double sum = 0;
            for (int k = 0; k < n; k++) {
                sum -= m1[i][k] * m2[k][j];
            }
            temparay.push_back(sum);
        }
        array.push_back(temparay);
        temparay.erase(temparay.begin(), temparay.end());
    }
    return array;
}