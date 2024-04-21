#include <target_tracking_visualization/target_tracking_visualization_ros.hpp>

int main(int argc, char *argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetTrackingVisualizationNode>());
    rclcpp::shutdown();
    return 0;
}
