#include "v_test/camera_node.h"
 
int Camera_node::init(int argc,char **argv)
{
  ros::init(argc, argv, "camera_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("velodyne/image", 1);
 
  cv::VideoCapture cap(0);
  std::string img_name;
  if(!cap.isOpened())
  {
    ROS_INFO("Can not open camera!");
    return -1;
  }
  else
  {
    ROS_INFO("Camera opened success...");
  }
  cap.set(cv::VideoCaptureProperties::CAP_PROP_FRAME_WIDTH ,640);
  cap.set(cv::VideoCaptureProperties::CAP_PROP_FRAME_HEIGHT,480);
  cv::Mat frame;
  ros::Rate loop_rate(30);
  
  while(nh.ok()&&cap.isOpened())
  {
    {
      cap>>frame;
      if(frame.empty())
      {
        ROS_INFO("frame is empty!");
        return -1;
      }
      else
      {
      cv::imshow("camera frame",frame);
      if(cv::waitKey(10)==27)
      {
        ROS_INFO("ImagePro ESC...");
        return -1;
      }
      //发布图像消息
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();  //通过cv_bridge转换成sensor_msgs::ImagePtr格式
      pub.publish(msg);
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
