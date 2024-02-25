#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vortex_msgs/msg/visualization_data.hpp>
#include <vortex_msgs/msg/visualization_data_array.hpp>
#include <foxglove_msgs/msg/scene_update.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vortex_filtering/vortex_filtering.hpp>

class TargetTrackingVisualizationNode : public rclcpp::Node {
public:
    TargetTrackingVisualizationNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

protected:
    void topic_callback(const vortex_msgs::msg::VisualizationDataArray visualisation_data_array);

    void visualize_state(const vortex_msgs::msg::VisualizationDataArray &visualisation_data_array);

private:
    rclcpp::Subscription<vortex_msgs::msg::VisualizationDataArray>::SharedPtr subscription_;

    rclcpp::Publisher<foxglove_msgs::msg::SceneUpdate>::SharedPtr scene_entity_publisher_;
};
