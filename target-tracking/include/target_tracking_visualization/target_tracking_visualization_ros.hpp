#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vortex_msgs/msg/landmark.hpp>
#include <vortex_msgs/msg/landmark_array.hpp>
#include <foxglove_msgs/msg/scene_update.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vortex_filtering/vortex_filtering.hpp>
#include <vortex_msgs/msg/parameter.hpp>
#include <vortex_msgs/msg/parameter_array.hpp>

struct previous_positions {
    int id;
    std::vector<geometry_msgs::msg::Point> previous;

    void add_position(geometry_msgs::msg::Point& position) {
        previous.insert(previous.begin(), position);

        // If the vector size exceeds 15, remove the last element
        if (previous.size() > 20) {
            previous.pop_back();
        }
    }
};

class TargetTrackingVisualizationNode : public rclcpp::Node {
public:
    TargetTrackingVisualizationNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

protected:
    void topic_callback(const vortex_msgs::msg::LandmarkArray landmark_array);

    void parameter_callback(const vortex_msgs::msg::ParameterArray parameter_array);

private:

    std::vector<previous_positions> previous_positions_;

    rclcpp::Subscription<vortex_msgs::msg::LandmarkArray>::SharedPtr landmark_subscription_;

    rclcpp::Subscription<vortex_msgs::msg::ParameterArray>::SharedPtr parameter_subscription_;

    rclcpp::Publisher<foxglove_msgs::msg::SceneUpdate>::SharedPtr scene_entity_publisher_;

    double gate_threshold_;
    double gate_min_threshold_;
    double gate_max_threshold_;
};
