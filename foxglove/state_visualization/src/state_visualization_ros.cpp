#include "state_visualization/ros/state_visualization_ros.hpp"
#include "state_visualization/ros/state_visualization_utils.hpp"
#include <eigen3/Eigen/Dense>
#include <foxglove_msgs/msg/color.hpp>
#include <foxglove_msgs/msg/cylinder_primitive.hpp>

namespace vortex::visualization {

StateVisualizationNode::StateVisualizationNode() : Node("state_visualization") {

  pos_mahalanobis_threshold_ =
      this->declare_parameter<double>("pos_mahalanobis_threshold");

  std::string pose_sub_topic =
      this->declare_parameter<std::string>("pose_sub_topic");

  std::string pos_visualization_pub_topic =
      this->declare_parameter<std::string>("pos_visualization_pub_topic");

  std::string orient_visualization_pub_topic =
      this->declare_parameter<std::string>("orient_visualization_pub_topic");

  std::string odom_sub_topic =
      this->declare_parameter<std::string>("odom_sub_topic");

  std::string landmark_array_sub_topic =
      this->declare_parameter<std::string>("landmark_array_sub_topic");

  std::string pose_array_pub_topic =
      this->declare_parameter<std::string>("pose_array_pub_topic");

  auto qos_profile{rclcpp::QoS(5)};
  qos_profile.best_effort();
  qos_profile.durability_volatile();

  scene_pub_pos_ = this->create_publisher<foxglove_msgs::msg::SceneUpdate>(
      pos_visualization_pub_topic, qos_profile);
  scene_pub_orient_ = this->create_publisher<foxglove_msgs::msg::SceneUpdate>(
      orient_visualization_pub_topic, qos_profile);
  pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      pose_array_pub_topic, qos_profile);
  pose_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          pose_sub_topic, qos_profile,
          std::bind(&StateVisualizationNode::pose_callback, this,
                    std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_sub_topic, qos_profile,
      std::bind(&StateVisualizationNode::odom_callback, this,
                std::placeholders::_1));
  landmark_array_sub_ =
      this->create_subscription<vortex_msgs::msg::LandmarkArray>(
          landmark_array_sub_topic, qos_profile,
          std::bind(&StateVisualizationNode::landmark_array_callback, this,
                    std::placeholders::_1));
}

void StateVisualizationNode::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {

  foxglove_msgs::msg::SceneUpdate scene_update_pos;
  geometry_msgs::msg::PoseArray pose_array_msg;
  foxglove_msgs::msg::SceneUpdate scene_update_orient;
  scene_update_pos.entities.push_back(make_position_entity(
      msg->pose, msg->header, pos_mahalanobis_threshold_, 0));

  scene_update_orient.entities.push_back(
      make_orientation_entity(msg->pose, msg->header, 0));

  pose_array_msg.header = msg->header;
  pose_array_msg.poses.push_back(msg->pose.pose);
  pose_array_pub_->publish(pose_array_msg);

  scene_pub_pos_->publish(scene_update_pos);
  scene_pub_orient_->publish(scene_update_orient);
}

void StateVisualizationNode::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {

  foxglove_msgs::msg::SceneUpdate scene_update_pos;
  foxglove_msgs::msg::SceneUpdate scene_update_orient;

  scene_update_pos.entities.push_back(make_position_entity(
      msg->pose, msg->header, pos_mahalanobis_threshold_, 0));

  scene_update_orient.entities.push_back(
      make_orientation_entity(msg->pose, msg->header, 0));

  geometry_msgs::msg::PoseArray pose_array_msg;
  pose_array_msg.header = msg->header;
  pose_array_msg.poses.push_back(msg->pose.pose);
  pose_array_pub_->publish(pose_array_msg);

  scene_pub_pos_->publish(scene_update_pos);
  scene_pub_orient_->publish(scene_update_orient);
}

void StateVisualizationNode::landmark_array_callback(
    const vortex_msgs::msg::LandmarkArray::SharedPtr msg) {

  foxglove_msgs::msg::SceneUpdate scene_update_pos;
  foxglove_msgs::msg::SceneUpdate scene_update_orient;
  geometry_msgs::msg::PoseArray pose_array_msg;

  pose_array_msg.header = msg->header;

  for (const auto &landmark : msg->landmarks) {
    scene_update_pos.entities.push_back(make_position_entity(
        landmark.pose, msg->header, pos_mahalanobis_threshold_, 0));
    scene_update_orient.entities.push_back(
        make_orientation_entity(landmark.pose, msg->header, 0));
    pose_array_msg.poses.push_back(landmark.pose.pose);
  }

  pose_array_pub_->publish(pose_array_msg);
  scene_pub_pos_->publish(scene_update_pos);
  scene_pub_orient_->publish(scene_update_orient);
}

} // namespace vortex::visualization
