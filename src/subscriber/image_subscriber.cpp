#include <my_aruco/subscriber/image_subscriber.h>

namespace my_aruco {

ImageSubscriber::ImageSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buffer_size)
  : nh_(nh), buffer_size_(buffer_size) {
    imageSub_ = ros::Subscriber(nh_.subscribe(topic_name, buffer_size_, &ImageSubscriber::MsgCallback, this));
}

void ImageSubscriber::MsgCallback(const sensor_msgs::ImageConstPtr& msg) {

  m_.lock();
  std::shared_ptr<cv::Mat> pImage = std::make_shared<cv::Mat>(cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image);

  ImageStamped::Ptr pImageStamped = std::make_shared<ImageStamped>(msg->header.stamp.toSec(), pImage);

  while (imageBuffer_.size() >= buffer_size_) {
    imageBuffer_.pop_front();
  }

  imageBuffer_.emplace_back(pImageStamped);
  m_.unlock();
}

void ImageSubscriber::ParseData(std::deque<ImageStamped::Ptr>& buffer) {
  m_.lock();

  if (imageBuffer_.size() > 0) {
    buffer.insert(buffer.end(), imageBuffer_.begin(), imageBuffer_.end());
    imageBuffer_.clear();
  }

  m_.unlock();
}
}