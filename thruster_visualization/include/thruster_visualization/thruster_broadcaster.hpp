/**
 * @file thruster_broadcaster.hpp
 * @brief Header file for the ThrusterVisualization class.
 * 
 * This file contains the declaration of the ThrusterVisualization class, which is responsible for visualizing thruster forces in ROS.
 */

#ifndef VORTEX_VISUALIZATION_VISUALIZATION_ROS_HPP
#define VORTEX_VISUALIZATION_VISUALIZATION_ROS_HPP

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <array>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

/**
 * @class ThrusterVisualization
 * @brief Class for visualizing thruster forces in ROS.
 * 
 * The ThrusterVisualization class is a ROS node that subscribes to thruster forces and publishes visualization markers.
 */
class ThrusterVisualization : public rclcpp::Node {
    public:
        /**
         * @brief Constructor for ThrusterVisualization class.
         */
        explicit ThrusterVisualization();
    private:
        /**
         * @brief Publishes visualization_msg MarkerArray of type arrow according to the thruster positions, orientations and thruster forces data.
         */
        void publish_markers();

        /**
         * @brief Callback function for setting thruster data equal to the thruster forces message.
         * @param msg The thruster forces message.
         */
        void thruster_forces_callback(const std_msgs::msg::Float32MultiArray &msg);


        visualization_msgs::msg::Marker create_arc_marker(
        const std::string& frame_id,
        int id,
        const geometry_msgs::msg::Point& center,
        double radius,
        double start_angle,
        double end_angle,
        int num_segments);

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr thruster_marker_publisher_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr thruster_forces_subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::vector<std::vector<double>> thruster_positions_;
        std::vector<double> thruster_orientations_;
        std::vector<double> thruster_data_;
        int num_thrusters_;
        double total_force_magnitude_;
        double total_force_orientation_;

};

#endif // VORTEX_VISUALIZATION_VISUALIZATION_ROS_HPP