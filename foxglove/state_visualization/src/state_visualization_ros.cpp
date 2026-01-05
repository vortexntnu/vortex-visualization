#include "state_visualization/ros/state_visualization_ros.hpp"
#include "state_visualization/ros/state_visualization_utils.hpp"
#include <eigen3/Eigen/Dense>
#include <foxglove_msgs/msg/color.hpp>
#include <foxglove_msgs/msg/cylinder_primitive.hpp>

namespace vortex::visualization {

StateVisualizationNode::StateVisualizationNode() : Node("state_visualization") {

  std::string pose_sub_topic =
      this->declare_parameter<std::string>("pose_sub_topic");

  std::string state_visualization_pub_topic =
      this->declare_parameter<std::string>("state_visualization_pub_topic");

  std::string odom_sub_topic =
      this->declare_parameter<std::string>("odom_sub_topic");

  auto qos_profile{rclcpp::QoS(5)};
  qos_profile.best_effort();
  qos_profile.durability_volatile();

  scene_pub_ = this->create_publisher<foxglove_msgs::msg::SceneUpdate>(
      state_visualization_pub_topic, qos_profile);
  pose_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          pose_sub_topic, qos_profile,
          std::bind(&StateVisualizationNode::pose_callback, this,
                    std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_sub_topic, qos_profile,
      std::bind(&StateVisualizationNode::odom_callback, this,
                std::placeholders::_1));
}

void StateVisualizationNode::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  foxglove_msgs::msg::SceneUpdate scene_update;

  const double px = msg->pose.pose.position.x;
  const double py = msg->pose.pose.position.y;
  const double pz = msg->pose.pose.position.z;

  auto entity = make_scene_entity_base(msg, 0);

  // Position covariance
  Eigen::Matrix3d cov3d = extract_position_covariance(msg);
  EllipsoidData ell = compute_covariance_ellipsoid(cov3d);
  entity.spheres.push_back(make_covariance_sphere(ell, px, py, pz));

  // Orientation covariance
  Eigen::Quaterniond q(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  q.normalize();

  Eigen::Vector3d ori_std = extract_orientation_stddevs(msg);

  add_orientation_cone(entity, q * Eigen::Vector3d::UnitX(), ori_std.x(), px,
                       py, pz, 1.0f, 0.0f, 0.0f);

  add_orientation_cone(entity, q * Eigen::Vector3d::UnitY(), ori_std.y(), px,
                       py, pz, 0.0f, 1.0f, 0.0f);

  add_orientation_cone(entity, q * Eigen::Vector3d::UnitZ(), ori_std.z(), px,
                       py, pz, 0.0f, 0.0f, 1.0f);

  scene_update.entities.push_back(entity);
  scene_pub_->publish(scene_update);
}

void StateVisualizationNode::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  auto pose_msg =
      std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

  pose_msg->header = msg->header;
  pose_msg->pose = msg->pose;

  pose_callback(pose_msg);
}

} // namespace vortex::visualization
