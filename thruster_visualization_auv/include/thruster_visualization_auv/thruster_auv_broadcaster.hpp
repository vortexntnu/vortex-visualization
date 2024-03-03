/**
 * @file thruster_auv_broadcaster.hpp
 * @brief Header file for the ThrusterVisualizationAUV class.
 * 
 * This file contains the declaration of the ThrusterVisualizationAUV class, which is responsible for visualizing thruster forces in ROS.
 */

#ifndef VORTEX_VISUALIZATION_ROS_HPP
#define VORTEX_VISUALIZATION_ROS_HPP

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <array>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vortex_msgs/msg/thruster_forces.hpp>

/**
 * @class ThrusterVisualizationAUV
 * @brief Class for visualizing thruster forces in ROS.
 * 
 * The ThrusterVisualizationAUV class is a ROS node that subscribes to thruster forces and publishes visualization markers.
 */
class ThrusterVisualizationAUV : public rclcpp::Node {
    public:
        /**
         * @brief Constructor for ThrusterVisualizationAUV class.
         */
        explicit ThrusterVisualizationAUV();
    private:
        /**
         * @brief Publishes visualization_msg MarkerArray of type arrow according to the thruster positions, orientations and thruster forces data.
         */
        void publish_markers();

        /**
         * @brief Callback function for setting thruster data equal to the thruster forces message.
         * @param msg The thruster forces message.
         */
        void topic_callback(const vortex_msgs::msg::ThrusterForces &msg);

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
        rclcpp::Subscription<vortex_msgs::msg::ThrusterForces>::SharedPtr subscription_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::vector<std::vector<double>> thruster_positions_;
        std::vector<std::vector<double>> thruster_orientations_;
        std::vector<double> thruster_data_;
        int num_thrusters_;

};

#endif // VORTEX_VISUALIZATION_ROS_HPP