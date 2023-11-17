#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <array>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vortex_msgs/msg/thruster_forces.hpp>
#include "geometry_msgs/msg/point.hpp"

class ThrusterVisualizationNode : public rclcpp::Node
{
public:
    ThrusterVisualizationNode() : Node("thruster_visualization_node")
    {
        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker", 10);
        subscription_ = this->create_subscription<vortex_msgs::msg::ThrusterForces>("thrust/thruster_forces", 10, std::bind(&ThrusterVisualizationNode::topic_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ThrusterVisualizationNode::publishMarkers, this));
    }

private:
    std::vector<std::array<float, 3>> thruster_positions {
        {-0.7, -0.5, 0.4}, // Thruster 0
        {0.7, -0.5, 0.4}, // Thruster 1
        {0.7, 0.5, 0.4},  // Thruster 2
        {-0.7, 0.5, 0.4} // Thruster 3
        };

    std::vector<float> thruster_orientations {
        {3.0 * M_PI / 4.0}, // Thruster 0 (3/4 pi)
        {5*M_PI / 4.0},       // Thruster 1 (5/4 pi)
        {3.0 * M_PI / 4.0}, // Thruster 2 (3/4 pi)
        {5*M_PI / 4.0} // Thruster 4 (5/4 pi)
    };
    
    std::vector<double> thruster_data = {0, 0, 0, 0};
    void publishMarkers()
    {
        visualization_msgs::msg::MarkerArray marker_array;

        for (size_t i = 0; i < thruster_data.size(); ++i)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "base_link"; // Adjust as needed
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "thrusters";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Set the position and orientation of the arrow here
            marker.pose.position.x = thruster_positions[i][0];
            marker.pose.position.y = thruster_positions[i][1];
            marker.pose.position.z = thruster_positions[i][2];
            
            tf2::Quaternion quat;
            float thruster_angle = thruster_orientations[i];
            
            if (thruster_data[i] < 0) {
                thruster_angle += M_PI;
            }

            quat.setRPY(0, 0, thruster_angle);
            marker.pose.orientation = tf2::toMsg(quat);

            marker.scale.x = std::abs(thruster_data[i])*0.05;
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

    void topic_callback(const vortex_msgs::msg::ThrusterForces &msg) {
        thruster_data = msg.thrust;
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::Subscription<vortex_msgs::msg::ThrusterForces>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ThrusterVisualizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
