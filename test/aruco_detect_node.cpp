#include "my_aruco/utils/parameters.h"
#include "my_aruco/detector/aruco_detector_factory.hpp"
#include "my_aruco/optim/aruco_optimizer_factory.hpp"
#include "my_aruco/types/markers.h"
#include "my_aruco/AngleStamped.h"

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>


class ArucoFlow {
public:
  ArucoFlow() = delete;
  ArucoFlow(const my_aruco::Parameters& p) : p_(p) {
    Init();
  }
  ArucoFlow(const my_aruco::Parameters& p, const ros::NodeHandle& nh) : p_(p), nh_(nh) {
    Init();
  }

  void Init() {
    imageSub_ = nh_.subscribe(p_.topicImageRaw, 100, &ArucoFlow::ImageCallback, this);
    arucoImagePub_ = nh_.advertise<sensor_msgs::Image>(p_.topicImageAruco, 100);
    measurementPub_ = nh_.advertise<my_aruco::AngleStamped>("measurement", 100);
    estimationPub_ = nh_.advertise<my_aruco::AngleStamped>("estimation", 100);


    pDetector_ = my_aruco::detector::create(p_);
    pOptimizer_ = my_aruco::optim::create(p_);

    interval_ = 1 / p_.fps;
  }

  bool Run() {
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

  void PublishImage() {
    // draw
    if (!markers_.hasImage)
      return;

    cv::Mat image = markers_.pImageStamped->image.clone();
    if (!markers_.IsEmpty())
      markers_.Draw(p_, image);

    pImageWithDraw = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    arucoImagePub_.publish(pImageWithDraw);
  }

  void ImageCallback(const sensor_msgs::ImagePtr& msg) {
    count_ = 0.0;
    markers_.Clear();
    my_aruco::ImageStamped::Ptr pTempImageStamped = std::make_shared<my_aruco::ImageStamped>(msg->header.stamp, cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image);

    markers_.SetImageStamped(pTempImageStamped);
    // // draw
    // cv::Mat image = pTempImageStamped->image;
    // if (!markers_.IsEmpty())
    //   markers_.Draw(p_, image);

    // pImageWithDraw = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    // arucoImagePub_.publish(pImageWithDraw);


  }


private:
  my_aruco::Parameters p_;

  ros::NodeHandle nh_;
  ros::Subscriber imageSub_;
  ros::Publisher arucoImagePub_;
  ros::Publisher measurementPub_;
  ros::Publisher estimationPub_;

  sensor_msgs::Image::Ptr pImageWithDraw;

  my_aruco::AngleStamped measurementMsg_;
  my_aruco::AngleStamped estimationMsg_;

  my_aruco::detector::ArucoDetector::Ptr pDetector_;
  my_aruco::optim::ArucoOptimizer::Ptr pOptimizer_;

  my_aruco::Markers markers_;

  size_t count_ = 0.0;
  double interval_ = 0.0;

};


int main(int argc, char** argv) {
  ros::init(argc, argv, "aruco_detect_node");

  ros::NodeHandle nh, nh_private("~");

  std::string configFile = nh_private.param<std::string>("config_file", "");

  if (configFile == "") {
    std::cerr << "Please give a config file path";
    return EXIT_FAILURE;
  }

  cv::FileStorage fs(configFile, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cerr << "Please check the config file path";
    return EXIT_FAILURE;
  }

  // create a image subscriber
  // todo: use my image sub
  // load parameters
  my_aruco::Parameters param(fs);
  param.Logging();

  ArucoFlow flow(param, nh);

  ros::Rate rate((int)fs["fps"]);

  while (ros::ok()) {
    ros::spinOnce();

    flow.Run();

    // flow.PublishImage();

    rate.sleep();
  }
}