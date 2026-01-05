#include "state_visualization/ros/state_visualization_ros.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Started state visualization Node");
  rclcpp::spin(
      std::make_shared<vortex::visualization::StateVisualizationNode>());
  rclcpp::shutdown();
  return 0;
}
