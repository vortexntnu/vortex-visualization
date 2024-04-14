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
    double max_torque = 100.0;
    double total_force_x = 0.0;
    double total_force_y = 0.0;
    double total_torque_z = 0.0;
    tf2::Quaternion quat;

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
        double torque_z = thruster_positions_[i][0] * force_y - thruster_positions_[i][1] * force_x;
        total_force_x += force_x;
        total_force_y += force_y;
        total_torque_z += torque_z;

        marker_array.markers.push_back(marker);
    }
    total_force_magnitude_ = sqrt(pow(total_force_x, 2) + pow(total_force_y, 2));
    total_force_orientation_ = atan2(total_force_y, total_force_x);

    // Visualize the total force
    visualization_msgs::msg::Marker total_force_marker = create_total_force_marker(total_force_magnitude_, total_force_orientation_);
    marker_array.markers.push_back(total_force_marker);

    // Visualize the total torque
    double normalized_torque = std::min(std::abs(total_torque_z) / max_torque, 1.0);
    double max_arc_degrees = 270.0; 
    double arc_degrees = normalized_torque * max_arc_degrees;
    double arc_radians = arc_degrees * (M_PI / 180.0);

    geometry_msgs::msg::Point center;
    center.x = 0.0;
    center.y = 0.0;
    center.z = -0.6;
    double radius = 0.5;
    double start_angle = 0;
    double end_angle;

    if (total_torque_z >= 0) {
        end_angle = start_angle + arc_radians;
    } else {
        end_angle = start_angle - arc_radians;
    }
    int num_segments = 30;
    
    visualization_msgs::msg::Marker arc = create_arc_marker("base_link", thruster_data_.size()+1, center, radius, start_angle, end_angle, num_segments);
    marker_array.markers.push_back(arc);

    double angle_step = (end_angle - start_angle) / num_segments;
    double final_angle = start_angle + num_segments * angle_step;

    visualization_msgs::msg::Marker torque_direction_arrow = create_torque_direction_marker("base_link", thruster_data_.size() + 2, center, radius, final_angle, total_torque_z);
    marker_array.markers.push_back(torque_direction_arrow);

    thruster_marker_publisher_->publish(marker_array);
}

void ThrusterVisualization::thruster_forces_callback(const std_msgs::msg::Float32MultiArray &msg) {
    thruster_data_.clear();
    for (float value : msg.data) {
        thruster_data_.push_back(static_cast<double>(value));
    }
}

visualization_msgs::msg::Marker ThrusterVisualization::create_total_force_marker(double total_force_magnitude, double total_force_orientation) {
    visualization_msgs::msg::Marker total_force_marker;
    total_force_marker.header.frame_id = "base_link";
    total_force_marker.header.stamp = this->get_clock()->now();
    total_force_marker.ns = "total_force_arrow";
    total_force_marker.id = thruster_data_.size();
    total_force_marker.type = visualization_msgs::msg::Marker::ARROW;
    total_force_marker.action = visualization_msgs::msg::Marker::ADD;

    // Position at the ASV's center
    total_force_marker.pose.position.x = 0.0;
    total_force_marker.pose.position.y = 0.0;
    total_force_marker.pose.position.z = 0.0;

    tf2::Quaternion quat;
    quat.setRPY(0, 0, total_force_orientation);
    total_force_marker.pose.orientation = tf2::toMsg(quat);

    total_force_marker.scale.x = total_force_magnitude * 0.05;
    total_force_marker.scale.y = 0.05;
    total_force_marker.scale.z = 0.05;

    total_force_marker.color.r = 1.0;
    total_force_marker.color.g = 0.0;
    total_force_marker.color.b = 1.0;
    total_force_marker.color.a = 1.0;

    total_force_marker.lifetime = rclcpp::Duration::from_seconds(0);

    return total_force_marker;
}

visualization_msgs::msg::Marker ThrusterVisualization::create_arc_marker(
    const std::string& frame_id,
    int id,
    const geometry_msgs::msg::Point& center,
    double radius,
    double start_angle,
    double end_angle,
    int num_segments)
{
    visualization_msgs::msg::Marker arc;
    arc.header.frame_id = frame_id;
    arc.header.stamp = rclcpp::Clock().now();
    arc.ns = "torque_direction_arrow";
    arc.id = id;
    arc.type = visualization_msgs::msg::Marker::LINE_STRIP;
    arc.action = visualization_msgs::msg::Marker::ADD;
    
    // Yellow
    arc.scale.x = 0.05; // Line width
    arc.color.r = 1.0;
    arc.color.g = 1.0;
    arc.color.b = 0.0;
    arc.color.a = 1.0; // Alpha (opacity)
    
    double angle_step = (end_angle - start_angle) / num_segments;
    for(int i = 0; i <= num_segments; ++i) {
        double angle = start_angle + i * angle_step;
        geometry_msgs::msg::Point p;
        p.x = center.x + radius * cos(angle);
        p.y = center.y + radius * sin(angle);
        p.z = center.z;
        arc.points.push_back(p);
    }
    
    return arc;
}

visualization_msgs::msg::Marker ThrusterVisualization::create_torque_direction_marker(const std::string& frame_id,
    int id,
    const geometry_msgs::msg::Point& center,
    double radius,
    double final_angle, 
    double total_torque_z) {

    // Calculate the end point for the arrow
    geometry_msgs::msg::Point end_point;
    end_point.x = center.x + radius * cos(final_angle);
    end_point.y = center.y + radius * sin(final_angle);
    end_point.z = center.z;

    // Create the arrow marker
    visualization_msgs::msg::Marker torque_direction_arrow;
    torque_direction_arrow.header.frame_id = frame_id;
    torque_direction_arrow.header.stamp = this->get_clock()->now();
    torque_direction_arrow.ns = "torque_direction_arrow";
    torque_direction_arrow.id = id; // Unique ID, different from arc and other markers
    torque_direction_arrow.type = visualization_msgs::msg::Marker::ARROW;
    torque_direction_arrow.action = visualization_msgs::msg::Marker::ADD;

    // Arrow pointing in the direction of the rotation
    if (abs(total_torque_z) < 1.0) {
        torque_direction_arrow.scale.x = 0.0;  // Length of the arrow
        torque_direction_arrow.scale.y = 0.0; // Width of the arrow head
        torque_direction_arrow.scale.z = 0.0; // Height of the arrow head
    }
    else {
        torque_direction_arrow.scale.x = 0.2;  // Length of the arrow
        torque_direction_arrow.scale.y = 0.05; // Width of the arrow head
        torque_direction_arrow.scale.z = 0.05; // Height of the arrow head
    }

    // Yellow
    torque_direction_arrow.color.r = 1.0;
    torque_direction_arrow.color.g = 1.0;
    torque_direction_arrow.color.b = 0.0;
    torque_direction_arrow.color.a = 1.0;

    // Set the position of the arrow
    torque_direction_arrow.pose.position = end_point;

    // Orient the arrow to point along the tangent to the arc at the endpoint
    tf2::Quaternion quat;
    quat.setRPY(0, 0, final_angle + (total_torque_z >= 0 ? M_PI / 2 : -M_PI / 2));
    torque_direction_arrow.pose.orientation = tf2::toMsg(quat);

    return torque_direction_arrow;
};
