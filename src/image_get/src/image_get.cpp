#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <sensor_msgs/CameraInfo.h>//传感器信息
#include<cmath>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <unistd.h>
#define stoptimelong 500  //Sleep函数以毫秒为单位，Sleep(500);表示停半秒
#define _NODE_NAME_ "image_get" 
using namespace cv;
using namespace std;

class PubAndSub
{
private:
	ros::Publisher image_get_pub;
	ros::NodeHandle nh;
	std::string pub_topic;
	ros::Timer timer_;
		
public:
	bool init();
	void image_get(const ros::TimerEvent&);	
};


bool PubAndSub::init()
{
    //1、 初始化节点
    ros::NodeHandle nh, nh_private("~"); 
    nh_private.param<std::string>("image_topic", pub_topic, "");
    //2、 初始化订阅发布
    image_get_pub = nh.advertise<sensor_msgs::Image>(pub_topic,1);
    timer_ = nh.createTimer(ros::Duration(0.1), &PubAndSub::image_get, this);
			
}

void PubAndSub::image_get(const ros::TimerEvent&)
{	
	std::string folder_path = "/home/cxl/ros_yolov5/src/datasets/test/*.*"; 
    //path of folder, you can replace "*.*" by "*.jpg" or "*.png"
    std::vector<cv::String> file_names;  
    cv::glob(folder_path, file_names);   //get file names
    cv::Mat img;   //try show it when file is a picture
 	ros::Rate rate_20hz(10); // 20Hz

    for (int i = 0; i < file_names.size(); i++)
    {
      //std::cout << file_names[i] << std::endl;
      img = cv::imread(file_names[i]);

      //cout<<i<<endl;
      //sleep(1);
      if (!img.data)
      {
          continue;
      }
     //cout<<"save"<<endl;
     //cv::Mat image = cv::imread("/home/cxl/ros_yolov5/src/datasets/img20211128150730904/img_w/rgb_20211128150731286_5000.png");
     sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",img).toImageMsg();
     image_get_pub.publish(msg); 
     ros::spinOnce();
     rate_20hz.sleep();
     }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, _NODE_NAME_);
    while (ros::ok())
    {
        PubAndSub image_get;
        cout<<"start"<<endl;
        image_get.init();//调用PubAndSub类的init成员函数
        ros::spin();//循环
    }
    return 0;
}
