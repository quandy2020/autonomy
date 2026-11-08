/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_MAP_POINT_OBSERVATION_TRANSFORM_HPP_
#define ATLAS_MAP_POINT_OBSERVATION_TRANSFORM_HPP_

#include <Eigen/Core>
#include <vector>

#include "common/pose_state.hpp"
#include "map/point_observation.hpp"

namespace atlas {
namespace map {

using ::atlas::common::PoseState;

using WorldPointList =
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;

/** Transforms point observations into world coordinates and propagates
 * covariance. */
void TransformPointObservationsToWorld(const PoseState& state,
                                       PointObservationList* observations,
                                       WorldPointList* world_points);

}  // namespace map
}  // namespace atlas

#endif  // ATLAS_MAP_POINT_OBSERVATION_TRANSFORM_HPP_
