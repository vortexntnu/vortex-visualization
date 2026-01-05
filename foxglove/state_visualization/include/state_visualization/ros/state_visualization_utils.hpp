#ifndef STATE_VISUALIZATION__STATE_VISUALIZATION_UTILS_HPP_
#define STATE_VISUALIZATION__STATE_VISUALIZATION_UTILS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <foxglove_msgs/msg/cylinder_primitive.hpp>
#include <foxglove_msgs/msg/scene_entity.hpp>
#include <foxglove_msgs/msg/scene_update.hpp>
#include <foxglove_msgs/msg/sphere_primitive.hpp>

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <string>

namespace vortex::visualization {

struct EllipsoidData {
  Eigen::Vector3d axes; // full axis lengths
  Eigen::Quaterniond q; // orientation
};

// Extract full 3x3 position covariance from PoseWithCovariance (row-major 6x6)
inline Eigen::Matrix3d extract_position_covariance(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr &msg) {
  Eigen::Matrix3d cov3d;
  cov3d(0, 0) = msg->pose.covariance[0 * 6 + 0];
  cov3d(0, 1) = msg->pose.covariance[0 * 6 + 1];
  cov3d(0, 2) = msg->pose.covariance[0 * 6 + 2];
  cov3d(1, 0) = msg->pose.covariance[1 * 6 + 0];
  cov3d(1, 1) = msg->pose.covariance[1 * 6 + 1];
  cov3d(1, 2) = msg->pose.covariance[1 * 6 + 2];
  cov3d(2, 0) = msg->pose.covariance[2 * 6 + 0];
  cov3d(2, 1) = msg->pose.covariance[2 * 6 + 1];
  cov3d(2, 2) = msg->pose.covariance[2 * 6 + 2];
  return cov3d;
}

// Eigen decomposition: get ellipsoid axes and orientation
inline EllipsoidData
compute_covariance_ellipsoid(const Eigen::Matrix3d &cov3d) {
  EllipsoidData out;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov3d);
  Eigen::Vector3d evals = es.eigenvalues();
  Eigen::Matrix3d evecs = es.eigenvectors();

  std::array<int, 3> idx = {0, 1, 2};
  std::sort(idx.begin(), idx.end(),
            [&](int a, int b) { return evals(a) > evals(b); });

  double var0 = std::max(evals(idx[0]), 1e-12);
  double var1 = std::max(evals(idx[1]), 1e-12);
  double var2 = std::max(evals(idx[2]), 1e-12);

  // chi-square scaling for 95% confidence (3 DOF)
  const double chi2_95_3d = 7.815;

  out.axes.x() = 2.0 * std::sqrt(chi2_95_3d * var0);
  out.axes.y() = 2.0 * std::sqrt(chi2_95_3d * var1);
  out.axes.z() = 2.0 * std::sqrt(chi2_95_3d * var2);

  Eigen::Matrix3d R;
  R.col(0) = evecs.col(idx[0]);
  R.col(1) = evecs.col(idx[1]);
  R.col(2) = evecs.col(idx[2]);

  if (R.determinant() < 0) {
    R.col(2) *= -1.0;
  }

  out.q = Eigen::Quaterniond(R);
  out.q.normalize();
  return out;
}

// Build a SceneUpdate containing ellipsoid + orientation cones
inline foxglove_msgs::msg::SceneUpdate visualize_pose_covariance(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr &msg,
    const std::string &frame_id, int32_t entity_id = 0) {
  foxglove_msgs::msg::SceneEntity entity;
  entity.timestamp = msg->header.stamp;
  entity.frame_id = frame_id;
  entity.id = entity_id;
  entity.frame_locked = false;
  entity.lifetime.sec = 0;
  entity.lifetime.nanosec = 0;

  const double px = msg->pose.pose.position.x;
  const double py = msg->pose.pose.position.y;
  const double pz = msg->pose.pose.position.z;

  // --- Position covariance ellipsoid ---
  EllipsoidData ell =
      compute_covariance_ellipsoid(extract_position_covariance(msg));

  foxglove_msgs::msg::SpherePrimitive sphere;
  sphere.pose.position.x = px;
  sphere.pose.position.y = py;
  sphere.pose.position.z = pz;
  sphere.pose.orientation.x = ell.q.x();
  sphere.pose.orientation.y = ell.q.y();
  sphere.pose.orientation.z = ell.q.z();
  sphere.pose.orientation.w = ell.q.w();

  sphere.size.x = ell.axes.x();
  sphere.size.y = ell.axes.y();
  sphere.size.z = ell.axes.z();

  sphere.color.r = 0.0f;
  sphere.color.g = 1.0f;
  sphere.color.b = 0.0f;
  sphere.color.a = 0.35f;

  entity.spheres.push_back(sphere);

  // --- Orientation uncertainty cones ---
  const double ang_scale = 0.5;
  const double min_len = 0.05;

  auto add_cone = [&](const Eigen::Vector3d &dir, double stddev, float r,
                      float g, float b) {
    double len = std::max(min_len, stddev * ang_scale);

    foxglove_msgs::msg::CylinderPrimitive cone;
    cone.size.x = 0.03;
    cone.size.y = 0.03;
    cone.size.z = len;
    cone.bottom_scale = 1.0;
    cone.top_scale = 0.0;

    Eigen::Quaterniond rot = Eigen::Quaterniond::FromTwoVectors(
        Eigen::Vector3d::UnitZ(), dir.normalized());

    cone.pose.orientation.x = rot.x();
    cone.pose.orientation.y = rot.y();
    cone.pose.orientation.z = rot.z();
    cone.pose.orientation.w = rot.w();

    Eigen::Vector3d shift = dir.normalized() * (len * 0.5);
    cone.pose.position.x = px + shift.x();
    cone.pose.position.y = py + shift.y();
    cone.pose.position.z = pz + shift.z();

    cone.color.r = r;
    cone.color.g = g;
    cone.color.b = b;
    cone.color.a = 0.75f;

    entity.cylinders.push_back(cone);
  };

  Eigen::Quaterniond q(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  q.normalize();

  double std_roll = std::sqrt(std::max(0.0, msg->pose.covariance[3 * 6 + 3]));
  double std_pitch = std::sqrt(std::max(0.0, msg->pose.covariance[4 * 6 + 4]));
  double std_yaw = std::sqrt(std::max(0.0, msg->pose.covariance[5 * 6 + 5]));

  add_cone(q * Eigen::Vector3d::UnitX(), std_roll, 1.0f, 0.0f, 0.0f);
  add_cone(q * Eigen::Vector3d::UnitY(), std_pitch, 0.0f, 1.0f, 0.0f);
  add_cone(q * Eigen::Vector3d::UnitZ(), std_yaw, 0.0f, 0.0f, 1.0f);

  foxglove_msgs::msg::SceneUpdate update;
  update.entities.push_back(entity);
  return update;
}

foxglove_msgs::msg::SceneEntity make_scene_entity_base(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr &msg,
    int32_t entity_id) {
  foxglove_msgs::msg::SceneEntity entity;
  entity.timestamp = msg->header.stamp;
  entity.frame_id = msg->header.frame_id;
  entity.id = entity_id;
  entity.frame_locked = false;
  entity.lifetime.sec = 0;
  entity.lifetime.nanosec = 0;
  return entity;
}

foxglove_msgs::msg::SpherePrimitive
make_covariance_sphere(const EllipsoidData &ell, double px, double py,
                       double pz) {
  foxglove_msgs::msg::SpherePrimitive sphere;
  sphere.pose.position.x = px;
  sphere.pose.position.y = py;
  sphere.pose.position.z = pz;
  sphere.pose.orientation.x = ell.q.x();
  sphere.pose.orientation.y = ell.q.y();
  sphere.pose.orientation.z = ell.q.z();
  sphere.pose.orientation.w = ell.q.w();

  sphere.size.x = ell.axes.x();
  sphere.size.y = ell.axes.y();
  sphere.size.z = ell.axes.z();

  sphere.color.r = 0.0f;
  sphere.color.g = 1.0f;
  sphere.color.b = 0.0f;
  sphere.color.a = 0.35f;

  return sphere;
}

Eigen::Vector3d extract_orientation_stddevs(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr &msg) {
  return {std::sqrt(std::max(0.0, msg->pose.covariance[3 * 6 + 3])),
          std::sqrt(std::max(0.0, msg->pose.covariance[4 * 6 + 4])),
          std::sqrt(std::max(0.0, msg->pose.covariance[5 * 6 + 5]))};
}

void add_orientation_cone(foxglove_msgs::msg::SceneEntity &entity,
                          const Eigen::Vector3d &dir, double stddev, double px,
                          double py, double pz, float r, float g, float b) {
  const double ang_scale = 0.5;
  const double min_len = 0.05;
  double len = std::max(min_len, stddev * ang_scale);

  foxglove_msgs::msg::CylinderPrimitive cone;
  cone.size.x = 0.03;
  cone.size.y = 0.03;
  cone.size.z = len;
  cone.bottom_scale = 1.0;
  cone.top_scale = 0.0;

  Eigen::Quaterniond rot = Eigen::Quaterniond::FromTwoVectors(
      Eigen::Vector3d::UnitZ(), dir.normalized());

  cone.pose.orientation.x = rot.x();
  cone.pose.orientation.y = rot.y();
  cone.pose.orientation.z = rot.z();
  cone.pose.orientation.w = rot.w();

  Eigen::Vector3d shift = dir.normalized() * (len * 0.5);
  cone.pose.position.x = px + shift.x();
  cone.pose.position.y = py + shift.y();
  cone.pose.position.z = pz + shift.z();

  cone.color.r = r;
  cone.color.g = g;
  cone.color.b = b;
  cone.color.a = 0.75f;

  entity.cylinders.push_back(cone);
}

} // namespace vortex::visualization

#endif // STATE_VISUALIZATION__STATE_VISUALIZATION_UTILS_HPP_
