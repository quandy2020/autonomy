/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_FRONTEND_ODOMETRY_POINT_COVARIANCE_ESTIMATOR_HPP_
#define ATLAS_FRONTEND_ODOMETRY_POINT_COVARIANCE_ESTIMATOR_HPP_

#include <Eigen/Core>

#include "frontend/inertial/inertial_state.hpp"
#include "map/point_observation.hpp"
#include "sensor/point_types.hpp"

namespace atlas {
namespace frontend {
namespace odometry {

using ::atlas::map::PointObservationList;

using ::atlas::frontend::inertial::InertialState;

/**
 * @brief Computes body-frame point covariance from range and beam noise.
 * @param body_point Point in the LiDAR body frame; near-zero z is clamped.
 * @param range_noise_meters Standard deviation of the measured range.
 * @param beam_angle_noise_degrees Standard deviation of beam angle.
 * @param covariance Output 3×3 covariance matrix.
 */
class PointCovarianceEstimator {
 public:
  /** Computes covariance for one point in the LiDAR body frame. */
  static void CalculateBodyPointCovariance(Eigen::Vector3d body_point,
                                           float range_noise_meters,
                                           float beam_angle_noise_degrees,
                                           Eigen::Matrix3d* covariance);

  /** Initializes point observations from a cloud in the extrinsic frame. */
  static void InitializePointObservations(
      const InertialState& extrinsic_state,
      const sensor::PointCloudXYZI& point_cloud, double range_noise_meters,
      double beam_angle_noise_degrees, PointObservationList* observations);
};

}  // namespace odometry
}  // namespace frontend
}  // namespace atlas

#endif  // ATLAS_FRONTEND_ODOMETRY_POINT_COVARIANCE_ESTIMATOR_HPP_
