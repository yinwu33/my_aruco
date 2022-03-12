#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

#include <string>

#include <Eigen/Dense>

static Eigen::Quaterniond AddOffset(const Eigen::Quaterniond& input) {
  // Eigen::Matrix4d offsetMatrix = Eigen::Matrix4d::Identity();
  // Eigen::Matrix3d offsetMatrix = Eigen::AngleAxisd(M_PI, Eigen::Vector3d(1, 0, 0)).toRotationMatrix(); // real
  Eigen::Matrix3d offsetMatrix = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(0, 1, 0)).toRotationMatrix(); // sim
  return Eigen::Quaterniond(input.matrix() * offsetMatrix);
}

static double calculateYaw(const Eigen::Quaterniond& q) {
  Eigen::Vector3d vector = q.matrix() * Eigen::Vector3d(0, 0, 1); // real
  Eigen::Vector3d vector = q.matrix() * Eigen::Vector3d(0, 1, 0); // sim
  // std::cout << vector.transpose() << std::endl;
  // return atan2(vector[0], vector[2]); // real
  return atan2(vector[2], vector[1]); // simulation
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "gt_pose_node");
  ros::NodeHandle nh, nh_private("~");
  ros::Publisher gt_pub = nh.advertise<geometry_msgs::PointStamped>("groundtruth", 100);

  std::string frame_id = nh_private.param<std::string>("frame_id", "panda_link0");
  std::string child_frame_id = nh_private.param<std::string>("child_frame_id", "panda_link7");

  // logging
  ROS_INFO("frame_id: %s\nchild_frame_id: %s", frame_id.c_str(), child_frame_id.c_str());

  tf::TransformListener listener;

  geometry_msgs::PointStamped pose;
  pose.header.frame_id = frame_id;

  tf::StampedTransform transform;

  ros::Rate rate(30);

  while (ros::ok()) {
    ros::Time now = ros::Time();
    try {
      listener.lookupTransform(frame_id, child_frame_id, now, transform);


    }
    catch (tf::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(0.5).sleep();
      continue;
    }
      tf::Quaternion q = transform.getRotation();
      Eigen::Quaterniond rotationMatrix = AddOffset(Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()));
      double yaw = calculateYaw(rotationMatrix);

      pose.header.stamp = transform.stamp_;
      // std::cout << transform.stamp_.toSec() << std::endl;
      pose.point.x = yaw;
      pose.point.y = yaw * 180 / M_PI;  
      gt_pub.publish(pose);
      // std::cout << rotationMatrix.matrix() << std::endl;
      rate.sleep();
  }

  return EXIT_SUCCESS;
}