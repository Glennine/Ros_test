#include "v_test/camera_node.h"
 
int Camera_node::init(int argc,char **argv,cv::VideoCapture cap)
{
  ros::init(argc, argv, "camera_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("velodyne/image", 1);
  YAML::Node config;
    try{
         config = YAML::LoadFile("/home/g/CLionProjects/Ros_test/src/v_test/src/config/path.yaml");
    }catch(YAML::BadFile &e){
        std::cout<<"read error!"<<std::endl;
        return -1;
    }

  //cv::VideoCapture cap(0);
  std::string img_name_path = config["image_path"].as<std::string>();
  int img_num = 0;
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
  ros::Rate loop_rate(10);
  
  while(nh.ok()&&cap.isOpened()&&img_num<100)
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
      cv::imwrite(img_name_path+std::to_string(img_num)+".png",frame);
      if(cv::waitKey(10)==27)
      {
        ROS_INFO("ImagePro ESC...");
        return -1;
      }
      //发布图像消息
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();  //通过cv_bridge转换成sensor_msgs::ImagePtr格式
      pub.publish(msg);
      }
      img_num++;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO_STREAM("Camera collection finished!");
}
