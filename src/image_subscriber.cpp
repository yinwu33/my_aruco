#include <my_aruco/image_subscriber.h>

namespace my_aruco {

ImageSubscriber::ImageSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buffer_size)
  : nh_(nh), buffer_size_(buffer_size) {
    p_Image_sub_ = std::make_shared<ros::Subscriber>(nh_.subscribe(topic_name, buffer_size_, &ImageSubscriber::MsgCallback, this));
}

void ImageSubscriber::MsgCallback(const sensor_msgs::ImageConstPtr& msg) {

  m_.lock();
  std::shared_ptr<cv::Mat> pImage = std::make_shared<cv::Mat>(cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image);

  while (dq_image_buffer_.size() > buffer_size_) {
    dq_image_buffer_.pop_front();
  }

  dq_image_buffer_.push_back(pImage);
  m_.unlock();
}

void ImageSubscriber::ParseData(std::deque<std::shared_ptr<cv::Mat>>& dq_buffer) {
  m_.lock();

  if (dq_image_buffer_.size() > 0) {
    // std::cout << "debug " << dq_image_buffer_.size() << std::endl; // ! debug
    dq_buffer.insert(dq_buffer.end(), dq_image_buffer_.begin(), dq_image_buffer_.end());
    dq_image_buffer_.clear();
  }

  m_.unlock();
}
}