#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

#include <string>

#include <Eigen/Dense>

int main (int argc, char** argv) {
  ros::init(argc, argv, "gt_pose_node");
  ros::NodeHandle nh, nh_private("~");

  std::string frame_id = nh_private.param<std::string>("frame_id", "panda_link0");
  std::string child_frame_id = nh_private.param<std::string>("child_frame_id", "panda_link7");

  tf::TransformListener listener;

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = frame_id;

  tf::StampedTransform transform;

  while (ros::ok()) {
    try {
    listener.lookupTransform(frame_id, child_frame_id, ros::Time(0), transform);

    tf::Quaternion q = transform.getRotation();
    Eigen::Matrix3d rotationMatrix(Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()));
    Eigen::Vector3d vector = rotationMatrix * Eigen::Vector3d(0, 0, 1);
    double yaw = atan2(vector[0], vector[2]);

    std::cout << yaw << std::endl;
    }
    catch (tf::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  return EXIT_SUCCESS;
}