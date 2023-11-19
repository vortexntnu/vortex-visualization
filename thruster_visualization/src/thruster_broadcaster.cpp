#include "thruster_visualization/thruster_broadcaster.hpp"

ThrusterVisualization::ThrusterVisualization() : Node("thruster_visualization_node") {
    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker", 10);
    subscription_ = this->create_subscription<vortex_msgs::msg::ThrusterForces>("thrust/thruster_forces", 10, std::bind(&ThrusterVisualization::topic_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ThrusterVisualization::publish_markers, this));

    thruster_positions_ = {
        {-0.7, -0.5, 0.4},  // Thruster 0
        {0.7, -0.5, 0.4},   // Thruster 1
        {0.7, 0.5, 0.4},    // Thruster 2
        {-0.7, 0.5, 0.4}    // Thruster 3
        };

    thruster_orientations_ = {
        {3.0 * M_PI / 4.0}, // Thruster 0
        {5*M_PI / 4.0},     // Thruster 1
        {3.0 * M_PI / 4.0}, // Thruster 2
        {5*M_PI / 4.0}      // Thruster 3
    };

    thruster_data_ = {0, 0, 0, 0};
}

void ThrusterVisualization::publish_markers() {
    visualization_msgs::msg::MarkerArray marker_array;

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

        marker.scale.x = std::abs(thruster_data_[i])*0.05;
        marker.scale.y = 0.05; // Head diameter
        marker.scale.z = 0.05; // Length of the arrow

        // Set color and other properties
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.lifetime = rclcpp::Duration::from_seconds(0);

        marker_array.markers.push_back(marker);
    }
    publisher_->publish(marker_array);
}

void ThrusterVisualization::topic_callback(const vortex_msgs::msg::ThrusterForces &msg) {
    thruster_data_ = msg.thrust;
}

