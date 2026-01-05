#ifndef STATE_VISUALIZATION__STATE_VISUALIZATION_ROS_HPP_
#define STATE_VISUALIZATION__STATE_VISUALIZATION_ROS_HPP_

#include <foxglove_msgs/msg/scene_update.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace vortex::visualization {

class StateVisualizationNode : public rclcpp::Node {
public:
  StateVisualizationNode();

  void pose_callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_sub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Publisher<foxglove_msgs::msg::SceneUpdate>::SharedPtr scene_pub_;
};

} // namespace vortex::visualization

#endif // STATE_VISUALIZATION__STATE_VISUALIZATION__ROS_HPP_
