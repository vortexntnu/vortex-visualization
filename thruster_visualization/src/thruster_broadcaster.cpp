#include "thruster_visualization/thruster_broadcaster.hpp"

ThrusterVisualization::ThrusterVisualization() : Node("thruster_visualization_node") {
    thruster_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker", 1);
    thruster_forces_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("thrust/thruster_forces", 10, std::bind(&ThrusterVisualization::thruster_forces_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ThrusterVisualization::publish_markers, this));

    num_thrusters_ = 4;
    thruster_positions_ = std::vector<std::vector<double>>(num_thrusters_, std::vector<double>(3, 0.0));
    thruster_orientations_ = std::vector<double>(num_thrusters_, 0.0);

    for (int i = 0; i < num_thrusters_; ++i) {
        this->declare_parameter<std::vector<double>>("thruster" + std::to_string(i) + "_position");
        this->declare_parameter<double>("thruster" + std::to_string(i) + "_orientation");
        
        this->get_parameter("thruster" + std::to_string(i) + "_position", thruster_positions_[i]);
        this->get_parameter("thruster" + std::to_string(i) + "_orientation", thruster_orientations_[i]);
    }

    thruster_data_ = {0.0, 0.0, 0.0, 0.0};
    total_force_magnitude_ = 0.0;
    total_force_orientation_ = 0.0;
}

void ThrusterVisualization::publish_markers() {
    visualization_msgs::msg::MarkerArray marker_array;
    double max_force = 100.0;
    double total_force_x = 0.0;
    double total_force_y = 0.0;

    for (size_t i = 0; i < thruster_data_.size(); ++i)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "thrusters";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the position and orientation of the arrow here
        marker.pose.position.x = thruster_positions_[i][0];
        marker.pose.position.y = thruster_positions_[i][1];
        marker.pose.position.z = thruster_positions_[i][2];
        
        tf2::Quaternion quat;
        float thruster_angle = thruster_orientations_[i];
        
        if (thruster_data_[i] < 0) {
            thruster_angle += M_PI;
        }

        quat.setRPY(0, 0, thruster_angle);
        marker.pose.orientation = tf2::toMsg(quat);

        double force_magnitude = std::abs(thruster_data_[i]);
        marker.scale.x = std::abs(thruster_data_[i])*0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        // Set color and other properties
        double color_intensity = std::min(force_magnitude / max_force, 1.0);
        marker.color.r = color_intensity;
        marker.color.g = 1.0 - color_intensity;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.lifetime = rclcpp::Duration::from_seconds(0);
        double force_x = (thruster_data_[i]) * cos(thruster_orientations_[i]);
        double force_y = (thruster_data_[i]) * sin(thruster_orientations_[i]);
        total_force_x += force_x;
        total_force_y += force_y;

        marker_array.markers.push_back(marker);
    }

    total_force_magnitude_ = sqrt(pow(total_force_x, 2) + pow(total_force_y, 2));
    total_force_orientation_ = atan2(total_force_y, total_force_x);

    visualization_msgs::msg::Marker total_force_marker;
    total_force_marker.header.frame_id = "base_link";
    total_force_marker.header.stamp = this->get_clock()->now();
    total_force_marker.ns = "thrusters";
    total_force_marker.id = thruster_data_.size();
    total_force_marker.type = visualization_msgs::msg::Marker::ARROW;
    total_force_marker.action = visualization_msgs::msg::Marker::ADD;

    // Position at the ASV's center
    total_force_marker.pose.position.x = 0.0;
    total_force_marker.pose.position.y = 0.0;
    total_force_marker.pose.position.z = 0.0;

    tf2::Quaternion quat;
    quat.setRPY(0, 0, total_force_orientation_);
    total_force_marker.pose.orientation = tf2::toMsg(quat);

    total_force_marker.scale.x = total_force_magnitude_ * 0.05;
    total_force_marker.scale.y = 0.05;
    total_force_marker.scale.z = 0.05;

    total_force_marker.color.r = 1.0;
    total_force_marker.color.g = 0.0;
    total_force_marker.color.b = 1.0;
    total_force_marker.color.a = 1.0;

    total_force_marker.lifetime = rclcpp::Duration::from_seconds(0);

    // Add the total force marker to the array
    marker_array.markers.push_back(total_force_marker);

    thruster_marker_publisher_->publish(marker_array);
}

void ThrusterVisualization::thruster_forces_callback(const std_msgs::msg::Float32MultiArray &msg) {
    thruster_data_.clear();
    for (float value : msg.data) {
        thruster_data_.push_back(static_cast<double>(value));
    }
}