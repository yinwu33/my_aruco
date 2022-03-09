#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv4/opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>

#include <string>

std::string type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
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
  r += (chans+'0');

  return r;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "publisher");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<sensor_msgs::Image>("camera_image", 100);
    ros::Publisher camera_info_pub = n.advertise<sensor_msgs::CameraInfo>("camera_info", 100);

    // sensor_msgs::CameraInfo info;

    // camera_info_manager::CameraInfoManager manager(n, "camera", "file:///tmp/calibrationdata/ost.yaml");
    // info = manager.getCameraInfo();

    ros::Rate rate(20);

    cv::VideoCapture cap;
    if (!cap.open(0)) throw std::runtime_error("can't find camera");
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;

    while (ros::ok()) {
        cap >> frame;

        msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
        msg->header.stamp = ros::Time::now();
        // cv::waitKey(1);
        pub.publish(msg);
        // camera_info_pub.publish(info);

        rate.sleep();
    }

    return 0;
}