#include "thruster_visualization_auv/thruster_auv_broadcaster.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ThrusterVisualizationAUV>();
    RCLCPP_INFO(node->get_logger(), "Thruster visualization for AUV initiated");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
