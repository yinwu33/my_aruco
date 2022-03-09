#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

#include <string>

#include <Eigen/Dense>

static Eigen::Quaterniond AddOffset(const Eigen::Quaterniond& input) {
  // Eigen::Matrix4d offsetMatrix = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d offsetMatrix = Eigen::AngleAxisd(M_PI, Eigen::Vector3d(1, 0, 0)).toRotationMatrix();
  return Eigen::Quaterniond(input.matrix() * offsetMatrix);
}

static double calculateYaw(const Eigen::Quaterniond& q) {
  Eigen::Vector3d vector = q.matrix() * Eigen::Vector3d(0, 0, 1);
  return atan2(vector[0], vector[2]);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "gt_pose_node");
  ros::NodeHandle nh, nh_private("~");
  ros::Publisher gt_pub = nh.advertise<geometry_msgs::PointStamped>("gt_yaw", 100);

  std::string frame_id = nh_private.param<std::string>("frame_id", "panda_link0");
  std::string child_frame_id = nh_private.param<std::string>("child_frame_id", "panda_link7");

  tf::TransformListener listener;

  geometry_msgs::PointStamped pose;
  pose.header.frame_id = frame_id;

  tf::StampedTransform transform;

  ros::Rate rate(30);

  while (ros::ok()) {
    try {
      listener.lookupTransform(frame_id, child_frame_id, ros::Time(0), transform);


    }
    catch (tf::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
      // ros::Duration(1.0).sleep();
      continue;
    }
      tf::Quaternion q = transform.getRotation();
      Eigen::Quaterniond rotationMatrix = AddOffset(Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()));
      double yaw = calculateYaw(rotationMatrix);

      pose.header.stamp = ros::Time(0);
      pose.point.x = yaw;
      gt_pub.publish(pose);
      std::cout << pose.point.x << std::endl;
      rate.sleep();
  }

  return EXIT_SUCCESS;
}