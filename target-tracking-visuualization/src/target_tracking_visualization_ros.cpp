#include <target_tracking_visualization/target_tracking_visualization_ros.hpp>
#include <vortex_msgs/msg/landmark_array.hpp>

using std::placeholders::_1;

vortex::plotting::Ellipse gauss_to_ellipse(const vortex::prob::Gauss2d &gauss, double scaling)
{
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigenSolver(gauss.cov());
  Eigen::Vector2d eigenValues  = eigenSolver.eigenvalues();
  Eigen::Matrix2d eigenVectors = eigenSolver.eigenvectors();

  if (eigenValues(0) > eigenValues(1)) {
    std::swap(eigenValues(0), eigenValues(1));
    std::swap(eigenVectors(0, 0), eigenVectors(0, 1));
    std::swap(eigenVectors(1, 0), eigenVectors(1, 1));
  }

  double majorAxisLength = sqrt(eigenValues(1) * scaling) * 2;
  double minorAxisLength = sqrt(eigenValues(0) * scaling) * 2;
  double angle           = atan2(eigenVectors(1, 1), eigenVectors(0, 1)) * 180.0 / M_PI; // Convert to degrees

  vortex::plotting::Ellipse ellipse;
  ellipse.x     = gauss.mean()(0);
  ellipse.y     = gauss.mean()(1);
  ellipse.a     = majorAxisLength;
  ellipse.b     = minorAxisLength;
  ellipse.angle = angle;
  return ellipse;
}

TargetTrackingVisualizationNode::TargetTrackingVisualizationNode(const rclcpp::NodeOptions &options)
    : Node("target_tracking_visualization_node", options) {
    
    gate_threshold_ = 1.5;
    gate_min_threshold_ = 0.5;
    gate_max_threshold_ = 2.5;

    // Define the quality of service profile for publisher and subscriber
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    landmark_subscription_ = create_subscription<vortex_msgs::msg::LandmarkArray>(
        "target_tracking/landmarks", qos, std::bind(&TargetTrackingVisualizationNode::topic_callback, this, _1));

    parameter_subscription_ = create_subscription<vortex_msgs::msg::ParameterArray>(
        "target_tracking/parameters", qos, std::bind(&TargetTrackingVisualizationNode::parameter_callback, this, _1));

    scene_entity_publisher_ = this->create_publisher<foxglove_msgs::msg::SceneUpdate>("scene_entity", 10);
}

void TargetTrackingVisualizationNode::topic_callback(const vortex_msgs::msg::LandmarkArray landmark_array) {
    // Create a scene entity message
    foxglove_msgs::msg::SceneEntity scene_entity;
    scene_entity.timestamp = this->now(); // Set timestamp
    scene_entity.id = 1; // Set entity ID
    scene_entity.frame_id = "world_frame"; // Set entity frame ID
    scene_entity.lifetime.sec = 5; // Set entity lifetime
    scene_entity.lifetime.nanosec = 0;
    scene_entity.frame_locked = false; // Set entity frame locked

    foxglove_msgs::msg::CylinderPrimitive cylinder;

    bool landmark_found = false;

    for (vortex_msgs::msg::Landmark landmark : landmark_array.landmarks) {

        landmark_found = false;

        if (landmark.action == 0) {
            // delete the landmark from previous_positions_
            previous_positions_.erase(std::remove_if(previous_positions_.begin(), previous_positions_.end(),
                [&](const previous_positions& pos) { return pos.id == landmark.id; }), previous_positions_.end());
    
            continue;
        } else {
            // add the landmark to previous_positions_
            for (auto &it : previous_positions_) {
                if (it.id == landmark.id) {
                    it.add_position(landmark.odom.pose.pose.position);
                    landmark_found = true;
                }
            }
            

            if (!landmark_found) {
                previous_positions previous_position;
                previous_position.id = landmark.id;
                previous_position.previous.push_back(landmark.odom.pose.pose.position);
                previous_positions_.push_back(previous_position);
            }

            Eigen::Vector2d position{landmark.odom.pose.pose.position.x, landmark.odom.pose.pose.position.y};

            Eigen::Matrix4d covariance;

            covariance << landmark.odom.pose.covariance.at(0), landmark.odom.pose.covariance.at(1), 0.0, 0.0,
                        landmark.odom.pose.covariance.at(6), landmark.odom.pose.covariance.at(7), 0.0, 0.0,
                        0.0, 0.0, 1.0, 0.0,
                        0.0, 0.0, 0.0, 1.0;

            Eigen::Matrix2d position_covariance = covariance.block<2, 2>(0, 0);

            vortex::prob::Gauss2d gauss(position, position_covariance);

            vortex::plotting::Ellipse ellipse = gauss_to_ellipse(gauss, gate_threshold_); // Fix gate threshold param

            double a = ellipse.a;
            a = (a < gate_min_threshold_) ? gate_min_threshold_ : a;
            a = (a > gate_max_threshold_) ? gate_max_threshold_ : a;

            double b = ellipse.b;
            b = (b < gate_min_threshold_) ? gate_min_threshold_ : b;
            b = (b > gate_max_threshold_) ? gate_max_threshold_ : b;

            // Create a cylinder primitive
            cylinder.pose.position.x = landmark.odom.pose.pose.position.x;
            cylinder.pose.position.y = landmark.odom.pose.pose.position.y;
            cylinder.pose.position.z = 0.5;
            cylinder.pose.orientation.x = 0.0;
            cylinder.pose.orientation.y = 0.0;
            cylinder.pose.orientation.z = landmark.odom.pose.pose.orientation.z;
            cylinder.pose.orientation.w = landmark.odom.pose.pose.orientation.w;
            cylinder.size.x = a;
            cylinder.size.y = b;
            cylinder.size.z = 2.0; 
            cylinder.bottom_scale = 1.0; 
            cylinder.top_scale = 1.0; 
            cylinder.color.r = 1.0;
            cylinder.color.g = 0.0;
            cylinder.color.b = 0.0;
            cylinder.color.a = 0.3;

            scene_entity.cylinders.push_back(cylinder);

            
            foxglove_msgs::msg::LinePrimitive line;

            line.type = 0; // Set the line's type
            line.pose.position.x = 0.0;
            line.pose.position.y = 0.0;
            line.pose.position.z = 0.5;
            line.pose.orientation.x = 0.0;
            line.pose.orientation.y = 0.0;
            line.pose.orientation.z = 0.0;
            line.pose.orientation.w = 1.0;
            line.thickness = 0.1; // Set the line's thickness
            line.scale_invariant = false; // Set the line's scale invariant
            line.color.r = 1.0; // Set the line's color
            line.color.g = 0.0;
            line.color.b = 0.0;
            line.color.a = 1.0;

            // Add previous positions to the line with the same id
            for (auto previous_position : previous_positions_) {
                if (previous_position.id == landmark.id) {
                    for (auto previous_position : previous_position.previous) {
                        line.points.push_back(previous_position);
                    }
                }
            }
            scene_entity.lines.push_back(line);


            // Arrow for velocity
            foxglove_msgs::msg::ArrowPrimitive arrow;

            arrow.pose.position.x = landmark.odom.pose.pose.position.x;
            arrow.pose.position.y = landmark.odom.pose.pose.position.y;
            arrow.pose.position.z = 0.5;

            arrow.pose.orientation.x = 0.0;
            arrow.pose.orientation.y = 0.0;
            arrow.pose.orientation.z = landmark.odom.pose.pose.orientation.z;
            arrow.pose.orientation.w = landmark.odom.pose.pose.orientation.w;

            // Normalize the orientation quaternion
            double orientation_magnitude = sqrt(arrow.pose.orientation.x * arrow.pose.orientation.x +
                                                arrow.pose.orientation.y * arrow.pose.orientation.y +
                                                arrow.pose.orientation.z * arrow.pose.orientation.z +
                                                arrow.pose.orientation.w * arrow.pose.orientation.w);

            arrow.pose.orientation.x /= orientation_magnitude;
            arrow.pose.orientation.y /= orientation_magnitude;
            arrow.pose.orientation.z /= orientation_magnitude;
            arrow.pose.orientation.w /= orientation_magnitude;

            double velocity_magnitude = sqrt(pow(landmark.odom.twist.twist.linear.x, 2) + pow(landmark.odom.twist.twist.linear.y, 2));
                    
            arrow.shaft_length = velocity_magnitude*3;
            arrow.shaft_diameter = 0.1;
            arrow.head_length = 0.2;
            arrow.head_diameter = 0.2;
            arrow.color.r = 0.0;
            arrow.color.g = 1.0;
            arrow.color.b = 0.0;
            arrow.color.a = 1.0;

            scene_entity.arrows.push_back(arrow);
        }
    }
    // Create a scene update message
    foxglove_msgs::msg::SceneUpdate update;
    update.entities.push_back(scene_entity);

    // Publish the scene entity
    scene_entity_publisher_->publish(update);
}

void TargetTrackingVisualizationNode::parameter_callback(const vortex_msgs::msg::ParameterArray parameter_array) {
    for (vortex_msgs::msg::Parameter parameter : parameter_array.parameters) {
        if (parameter.name == "gate_threshold") {
            gate_threshold_ = std::stod(parameter.value);
        } else if (parameter.name == "gate_min_threshold") {
            gate_min_threshold_ = std::stod(parameter.value);
        } else if (parameter.name == "gate_max_threshold") {
            gate_max_threshold_ = std::stod(parameter.value);
        }
    }
}
