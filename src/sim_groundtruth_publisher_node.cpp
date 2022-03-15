#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64.h>

#include <string>

#include <Eigen/Dense>

// ! to be deleted
// static Eigen::Quaterniond AddOffset(const Eigen::Quaterniond& input) {
//   return input;
// }

static double calculateYaw(const Eigen::Quaterniond& q) {
  // base on franka robot
  Eigen::Vector3d vector = q.matrix() * Eigen::Vector3d(1, 0, 0);
  return - atan2(vector[1], vector[0]);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "sim_groundtruth_publisher_node");
  ros::NodeHandle nh, nh_private("~");
  ros::Publisher gt_pub = nh.advertise<std_msgs::Float64>("groundtruth", 100);

  std::string frame_id = nh_private.param<std::string>("frame_id", "link_base");
  std::string child_frame_id = nh_private.param<std::string>("child_frame_id", "link_trailer");
  int fps = nh_private.param<int>("fps", 30);
  // bool use_degree = nh.param<bool>("use_degree", false); // ! to be deleted

  // logging
  ROS_INFO("frame_id: %s\nchild_frame_id: %s", frame_id.c_str(), child_frame_id.c_str());

  tf::TransformListener listener;

  std_msgs::Float64 groundtruth;

  tf::StampedTransform transform;

  ros::Rate rate(fps);

  while (ros::ok()) {
    ros::Time now = ros::Time();
    try {
      listener.lookupTransform(frame_id, child_frame_id, now, transform);
    }
    catch (tf::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(0.1).sleep();
      continue;
    }

    tf::Quaternion q = transform.getRotation();
    double yaw = calculateYaw(Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()));

    // if (use_degree) // ! to be deleted
    //   yaw = yaw * 180 / M_PI;

    groundtruth.data = yaw;
    gt_pub.publish(groundtruth);

    rate.sleep();
  }

  return EXIT_SUCCESS;
}