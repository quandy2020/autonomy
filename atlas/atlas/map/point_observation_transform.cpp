/*
 * Copyright 2026 The Openbot Authors
 */
#include "map/point_observation_transform.hpp"

#include "common/math/so3.hpp"

namespace atlas {
namespace map {

void TransformPointObservationsToWorld(const PoseState& state,
                                       PointObservationList* observations,
                                       WorldPointList* world_points) {
  if (observations == nullptr) {
    return;
  }

  const Eigen::Matrix3d rotation_covariance =
      state.covariance.block<3, 3>(0, 0);
  const Eigen::Matrix3d translation_covariance =
      state.covariance.block<3, 3>(3, 3);
  if (world_points != nullptr) {
    world_points->clear();
    world_points->reserve(observations->size());
  }

  for (PointObservation& observation : *observations) {
    const Eigen::Matrix3d point_skew_symmetric =
        math::SkewSymmetric(observation.position);
    observation.covariance =
        state.rotation * observation.covariance * state.rotation.transpose() +
        point_skew_symmetric * rotation_covariance *
            point_skew_symmetric.transpose() +
        translation_covariance;
    if (world_points != nullptr) {
      world_points->push_back(state.rotation * observation.position +
                              state.translation);
    }
  }
}

}  // namespace map
}  // namespace atlas
