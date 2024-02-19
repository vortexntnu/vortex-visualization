#include <target_tracking_visualization/target_tracking_visualization_ros.hpp>

using std::placeholders::_1;


TargetTrackingVisualizationNode::TargetTrackingVisualizationNode(const rclcpp::NodeOptions &options)
    : Node("target_tracking_visualization_node", options) {

    // Define the quality of service profile for publisher and subscriber
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    subscription_ = create_subscription<vortex_msgs::msg::VisualizationDataArray>(
        "target_tracking/visualization", qos, std::bind(&TargetTrackingVisualizationNode::topic_callback, this, _1));

    scene_entity_publisher_ = this->create_publisher<foxglove_msgs::msg::SceneUpdate>("scene_entity", 10);
}

void TargetTrackingVisualizationNode::topic_callback(const vortex_msgs::msg::VisualizationDataArray visualisation_data_array) {
    TargetTrackingVisualizationNode::visualize_state(visualisation_data_array);
}

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

void TargetTrackingVisualizationNode::visualize_state(const vortex_msgs::msg::VisualizationDataArray &visualisation_data_array) 
{
    // Create a scene entity message
    foxglove_msgs::msg::SceneEntity scene_entity;
    scene_entity.timestamp = this->now(); // Set timestamp
    scene_entity.id = 1; // Set entity ID
    scene_entity.frame_id = "world_frame"; // Set entity frame ID
    scene_entity.lifetime.sec = 0; // Set entity lifetime
    scene_entity.lifetime.nanosec = 500000000;
    scene_entity.frame_locked = false; // Set entity frame locked

    foxglove_msgs::msg::CylinderPrimitive cylinder;

    for (auto visualisation_data : visualisation_data_array.visualization_data) {

        Eigen::Vector2d position{visualisation_data.x_final.x, visualisation_data.x_final.y};
        Eigen::Matrix4d covariance = Eigen::Map<Eigen::Matrix4d>(visualisation_data.x_final.covariance.data());

        Eigen::Matrix2d position_covariance = covariance.block<2, 2>(0, 0);

        vortex::prob::Gauss2d gauss(position, position_covariance);

        vortex::plotting::Ellipse ellipse = gauss_to_ellipse(gauss, visualisation_data.gate_threshold);

        Eigen::Quaterniond q;
        q = Eigen::AngleAxisd(ellipse.angle*M_PI/180, Eigen::Vector3d::UnitZ());

        // Create a cylinder primitive
        cylinder.pose.position.x = visualisation_data.x_final.x;
        cylinder.pose.position.y = visualisation_data.x_final.y;
        cylinder.pose.position.z = 0.5;
        cylinder.pose.orientation.x = q.x();
        cylinder.pose.orientation.y = q.y();
        cylinder.pose.orientation.z = q.z();
        cylinder.pose.orientation.w = q.w();
        cylinder.size.x = ellipse.a; 
        cylinder.size.y = ellipse.b; 
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

        for (auto previous_position : visualisation_data.previous) {
            geometry_msgs::msg::Point point;
            point.x = previous_position.x;
            point.y = previous_position.y;
            point.z = 0.0;
            line.points.push_back(point);
        }
        scene_entity.lines.push_back(line);


        // Arrow for velocity
        foxglove_msgs::msg::ArrowPrimitive arrow;

        arrow.pose.position.x = visualisation_data.x_final.x;
        arrow.pose.position.y = visualisation_data.x_final.y;
        arrow.pose.position.z = 0.5;

        arrow.pose.orientation.x = 0.0;
        arrow.pose.orientation.y = 0.0;
        arrow.pose.orientation.z = atan2(visualisation_data.x_final.v_y, visualisation_data.x_final.v_x);
        arrow.pose.orientation.w = 1.0;

        // Normalize the orientation quaternion
        double orientation_magnitude = sqrt(arrow.pose.orientation.x * arrow.pose.orientation.x +
                                            arrow.pose.orientation.y * arrow.pose.orientation.y +
                                            arrow.pose.orientation.z * arrow.pose.orientation.z +
                                            arrow.pose.orientation.w * arrow.pose.orientation.w);

        arrow.pose.orientation.x /= orientation_magnitude;
        arrow.pose.orientation.y /= orientation_magnitude;
        arrow.pose.orientation.z /= orientation_magnitude;
        arrow.pose.orientation.w /= orientation_magnitude;

        double velocity_magnitude = sqrt(pow(visualisation_data.x_final.v_x, 2) + pow(visualisation_data.x_final.v_y, 2));
                
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

    // Create a scene update message
    foxglove_msgs::msg::SceneUpdate update;
    update.entities.push_back(scene_entity);

    // Publish the scene entity
    scene_entity_publisher_->publish(update);
};


