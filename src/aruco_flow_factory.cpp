#include "my_aruco/aruco_flow_factory.h"

namespace my_aruco
{
std::shared_ptr<ArucoFlowFactory> ArucoFlowFactory::pInstance_ = nullptr;


ArucoFlowFactory::ArucoFlowFactory(const std::string& configFile) {
  cv::FileStorage fs(configFile, cv::FileStorage::READ);
  p_ = Parameters(fs);
  p_.Logging();

  imageSub_ = nh_.subscribe(p_.topicImageRaw, 100, &ArucoFlowFactory::ImageCallback, this);
  arucoImagePub_ = nh_.advertise<sensor_msgs::Image>(p_.topicImageAruco, 100);
  measurementPub_ = nh_.advertise<my_aruco::AngleStamped>("measurement", 100);
  estimationPub_ = nh_.advertise<my_aruco::AngleStamped>("estimation", 100);

  pDetector_ = my_aruco::detector::create(p_);
  pOptimizer_ = my_aruco::optim::create(p_);

  interval_ = 1 / p_.fps;
}

ArucoFlowFactory::Ptr ArucoFlowFactory::Create(const std::string& configFile) {
  if (pInstance_ == nullptr) {
    std::cout << "haha" << std::endl;
    pInstance_ = std::shared_ptr<ArucoFlowFactory>(new ArucoFlowFactory(configFile));
  }

  return pInstance_;
}


void ArucoFlowFactory::Run() {
  ros::Rate rate(p_.fps);

  while (ros::ok()) {
    ros::spinOnce();
    Detect();

    if (doPubImage) {
      PublishImage();
    }

    rate.sleep();
  }
}

bool ArucoFlowFactory::Detect() {
  if (!markers_.hasImage) {
    return false;
  }

  // return if no marker detected
  if (!pDetector_->Detect(markers_))
    return false;

  markers_.pImageStamped->timestamp = markers_.pImageStamped->timestamp;

  pDetector_->PoseEstimate(markers_);
  pDetector_->GetYaw(markers_);

  // publish measurement angle
  measurementMsg_.header.stamp = markers_.pImageStamped->timestamp + ros::Duration(interval_ * count_);
  measurementMsg_.radian = markers_.yaw;
  measurementMsg_.degree = measurementMsg_.radian * 180 / M_PI;
  measurementPub_.publish(measurementMsg_);

  pOptimizer_->Update(markers_);

  // publish estimation angle
  estimationMsg_.header.stamp = markers_.pImageStamped->timestamp + ros::Duration(interval_ * count_);
  estimationMsg_.radian = markers_.optimizedYaw;
  estimationMsg_.degree = estimationMsg_.radian * 180 / M_PI;
  estimationPub_.publish(estimationMsg_);

  // markers_.Clear();
  ++count_;
  return true;
}

void ArucoFlowFactory::PublishImage() {
  // draw
  if (!markers_.hasImage)
    return;

  cv::Mat image = markers_.pImageStamped->image.clone();
  if (!markers_.IsEmpty())
    markers_.Draw(p_, image);

  pArucoImage_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  arucoImagePub_.publish(pArucoImage_);
}

void ArucoFlowFactory::ImageCallback(const sensor_msgs::ImagePtr& msg) {
  count_ = 0.0;
  markers_.Clear();
  my_aruco::ImageStamped::Ptr pTempImageStamped = std::make_shared<my_aruco::ImageStamped>(msg->header.stamp, cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image);

  markers_.SetImageStamped(pTempImageStamped);
}

} // namespace my_aruco
