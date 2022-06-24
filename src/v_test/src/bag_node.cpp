#include "v_test/bag_node.h"

using namespace std;
void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    config_file = yaml;
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }


    fsSettings["image_topic"] >> IMAGE_TOPIC;
    fsSettings["time_path"] >> TIME_PATH;  
    fsSettings["image_path"] >> IMAGE_PATH;
    fsSettings.release();

}
int i=0;
void imageCallback(const sensor_msgs::ImageConstPtr &img_msg)
{

	cv_bridge::CvImageConstPtr ptr;
   ptr = cv_bridge::toCvCopy(img_msg, img_msg->encoding);
   cv::Mat show_img = ptr->image;

string str1,image_path;
stringstream ss1;
ss1 << i;
ss1 >> str1;
i++;
image_path=IMAGE_PATH + str1 + ".png";
cv::imwrite(image_path,show_img);
}




int main(int argc, char **argv)
{
    //if ( argc != 2 )
   // {
   //     cout<<"usage: no path of  .yaml "<<endl;
   //     return 1;
   // }
   yaml=argv[1];
// 初始化ROS节点
    ros::init(argc, argv, "image_subscriber");

    // 创建节点句柄
    ros::NodeHandle n;
    
	readParameters(n);

cout<<"IMAGE_PATH   "<< yaml<<endl;

    // 创建一个Subscriber，订阅名为/turtle1/pose的topic，注册回调函数poseCallback
    ros::Subscriber image_sub = n.subscribe(IMAGE_TOPIC, 5000, imageCallback);

    // 循环等待回调函数

    ros::spin();


    return 0;
}