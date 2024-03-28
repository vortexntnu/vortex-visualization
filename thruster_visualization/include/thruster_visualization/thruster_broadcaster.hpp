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
        void topic_callback(const std_msgs::msg::Float32MultiArray &msg);

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::vector<std::array<float, 3>> thruster_positions_;
        std::vector<float> thruster_orientations_;
        std::vector<double> thruster_data_;

};

#endif // VORTEX_VISUALIZATION_VISUALIZATION_ROS_HPP