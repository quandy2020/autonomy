/*
 * Copyright 2026 The Openbot Authors
 */
#include "frontend/odometry/point_covariance_estimator.hpp"

#include "common/math/so3.hpp"

namespace atlas {
namespace frontend {
namespace odometry {

using ::atlas::frontend::inertial::InertialState;
using ::atlas::map::PointObservation;

void PointCovarianceEstimator::CalculateBodyPointCovariance(
    Eigen::Vector3d body_point, float range_noise_meters,
    float beam_angle_noise_degrees, Eigen::Matrix3d* covariance) {
  if (covariance == nullptr) {
    return;
  }
  if (body_point[2] == 0.0) {
    body_point[2] = 0.0001;
  }
  const float point_range =
      std::sqrt(body_point[0] * body_point[0] + body_point[1] * body_point[1] +
                body_point[2] * body_point[2]);
  const float range_variance = range_noise_meters * range_noise_meters;
  Eigen::Matrix2d direction_covariance;
  const double beam_angle_radians = beam_angle_noise_degrees * M_PI / 180.0;
  const double beam_angle_sine = std::sin(beam_angle_radians);
  direction_covariance << beam_angle_sine * beam_angle_sine, 0, 0,
      beam_angle_sine * beam_angle_sine;
  Eigen::Vector3d direction(body_point);
  direction.normalize();
  const Eigen::Matrix3d direction_skew_symmetric =
      math::SkewSymmetric(direction);
  Eigen::Vector3d first_tangent_basis(
      1, 1, -(direction(0) + direction(1)) / direction(2));
  first_tangent_basis.normalize();
  Eigen::Vector3d second_tangent_basis = first_tangent_basis.cross(direction);
  second_tangent_basis.normalize();
  Eigen::Matrix<double, 3, 2> tangent_basis;
  tangent_basis << first_tangent_basis(0), second_tangent_basis(0),
      first_tangent_basis(1), second_tangent_basis(1), first_tangent_basis(2),
      second_tangent_basis(2);
  const Eigen::Matrix<double, 3, 2> angular_jacobian =
      point_range * direction_skew_symmetric * tangent_basis;
  *covariance =
      direction * range_variance * direction.transpose() +
      angular_jacobian * direction_covariance * angular_jacobian.transpose();
}

void PointCovarianceEstimator::InitializePointObservations(
    const InertialState& extrinsic_state,
    const sensor::PointCloudXYZI& point_cloud, double range_noise_meters,
    double beam_angle_noise_degrees, PointObservationList* observations) {
  if (observations == nullptr) {
    return;
  }
  observations->clear();
  observations->resize(point_cloud.size());
  for (std::size_t point_index = 0; point_index < point_cloud.size();
       ++point_index) {
    const sensor::PointType& point = point_cloud.points[point_index];
    PointObservation& observation = (*observations)[point_index];
    observation.position << point.x, point.y, point.z;
    CalculateBodyPointCovariance(
        observation.position, static_cast<float>(range_noise_meters),
        static_cast<float>(beam_angle_noise_degrees), &observation.covariance);
    observation.position =
        extrinsic_state.R * observation.position + extrinsic_state.p;
    observation.covariance = extrinsic_state.R * observation.covariance *
                             extrinsic_state.R.transpose();
  }
}

}  // namespace odometry
}  // namespace frontend
}  // namespace atlas
