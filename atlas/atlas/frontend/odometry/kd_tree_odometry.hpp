/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_FRONTEND_ODOMETRY_KD_TREE_ODOMETRY_HPP_
#define ATLAS_FRONTEND_ODOMETRY_KD_TREE_ODOMETRY_HPP_

#include <cstddef>
#include <vector>

#include "common/math/kdtree.hpp"
#include "frontend/inertial/inertial_state.hpp"
#include "map/point_observation.hpp"
#include "sensor/point_types.hpp"

namespace atlas {
namespace frontend {
namespace odometry {

using ::atlas::map::PointObservation;
using ::atlas::map::PointObservationList;
using ::atlas::map::SharedPointObservationList;

using ::atlas::frontend::inertial::InertialState;

/**
 * @brief Point-to-plane EKF using a local k-d tree map (Voxel-SLAM
 *        lio_state_estimation_kdtree) — used before voxel map is dense.
 */
class KdTreeOdometry {
 public:
  static constexpr int kNearestNeighborCount = 5;

  /** Adds initial points and reports when the map can support alignment. */
  bool AddPointsOrCheckReadiness(const PointObservationList& point_observations,
                                 const InertialState& inertial_state);

  /**
   * @brief Aligns the current scan to the tree map and updates the state.
   * @return False if not enough valid associations.
   */
  bool EstimatePose(const SharedPointObservationList& point_observations,
                    InertialState* inertial_state);

  void Clear();

  std::size_t map_point_count() const { return map_points_.size(); }

 private:
  std::vector<Eigen::Vector3d> map_points_;
  math::KdTree nearest_neighbor_index_;
  sensor::PointCloudXYZI map_point_cloud_;
};

}  // namespace odometry
}  // namespace frontend
}  // namespace atlas

#endif  // ATLAS_FRONTEND_ODOMETRY_KD_TREE_ODOMETRY_HPP_
