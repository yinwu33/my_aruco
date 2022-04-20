#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>

#include <string>

std::string type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch (depth) {
  case CV_8U:  r = "8U"; break;
  case CV_8S:  r = "8S"; break;
  case CV_16U: r = "16U"; break;
  case CV_16S: r = "16S"; break;
  case CV_32S: r = "32S"; break;
  case CV_32F: r = "32F"; break;
  case CV_64F: r = "64F"; break;
  default:     r = "User"; break;
  }

  r += "C";
  r += (chans + '0');

  return r;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "publisher");
  ros::NodeHandle nh, nh_private("~");

  // parameters
  std::string image_topic = nh_private.param<std::string>("image_topic", "image_raw");
  int queue_size = nh_private.param<int>("queue_size", 100);
  int fps = nh_private.param<int>("fps", 30);
  int video = nh_private.param<int>("video", 0);
  int width = nh_private.param<int>("width", 640);
  int height = nh_private.param<int>("height", 480);

  ROS_INFO("Parameters: \nfps: %i\nvideo: %i\nwidth: %i\nheight: %i", fps, video, width, height);

  ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>(image_topic, queue_size);

  ros::Rate rate(24);
  std::cout << "fps " << fps << std::endl;

  cv::VideoCapture cap;
  if (!cap.open(video)) throw std::runtime_error("can't find camera");

  cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);

  cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  while (ros::ok()) {
    cap >> frame;

    msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
    msg->header.stamp = ros::Time::now();
    image_pub.publish(msg);
  }

  return 0;
}