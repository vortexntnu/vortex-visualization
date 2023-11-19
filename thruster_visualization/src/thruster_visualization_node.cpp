#include "thruster_visualization/thruster_broadcaster.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ThrusterVisualization>();
    RCLCPP_INFO(node->get_logger(), "Thruster visualization initiated");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
