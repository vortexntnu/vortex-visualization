#ifndef STATE_VISUALIZATION__STATE_VISUALIZATION_ROS_HPP_
#define STATE_VISUALIZATION__STATE_VISUALIZATION_ROS_HPP_

#include <foxglove_msgs/msg/scene_update.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vortex_msgs/msg/landmark_array.hpp>

namespace vortex::visualization {

class StateVisualizationNode : public rclcpp::Node {
public:
  StateVisualizationNode();

  void pose_callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  void
  landmark_array_callback(const vortex_msgs::msg::LandmarkArray::SharedPtr msg);

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_sub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Subscription<vortex_msgs::msg::LandmarkArray>::SharedPtr
      landmark_array_sub_;

  rclcpp::Publisher<foxglove_msgs::msg::SceneUpdate>::SharedPtr scene_pub_pos_;

  rclcpp::Publisher<foxglove_msgs::msg::SceneUpdate>::SharedPtr
      scene_pub_orient_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;

  double pos_mahalanobis_threshold_{2.0};
};

} // namespace vortex::visualization

#endif // STATE_VISUALIZATION__STATE_VISUALIZATION__ROS_HPP_
