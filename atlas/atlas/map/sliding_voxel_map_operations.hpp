/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_MAP_SLIDING_VOXEL_MAP_OPERATIONS_HPP_
#define ATLAS_MAP_SLIDING_VOXEL_MAP_OPERATIONS_HPP_

#include <cstddef>
#include <memory>
#include <vector>

#include "common/pose_state.hpp"
#include "common/thread_pool.hpp"
#include "map/point_cloud_factor_data.hpp"
#include "map/sliding_voxel_tree.hpp"

namespace atlas {
namespace map {

using ::atlas::map::SharedPointObservationList;

using ::atlas::common::PoseState;

/** @brief Parallel operations over the active sliding voxel map. */
class SlidingVoxelMapOperations {
 public:
  SlidingVoxelMapOperations(common::ThreadPoolInterface* thread_pool,
                            std::size_t worker_count);

  void Marginalize(
      SlidingVoxelMap* voxel_map, double trajectory_distance,
      int active_window_count, const std::vector<PoseState>* pose_window,
      PointCloudFactorData* factor_data,
      std::vector<std::unique_ptr<FrameWindow>>* reusable_frame_windows);

  void Recut(SlidingVoxelMap* voxel_map, int active_window_count,
             const std::vector<PoseState>* pose_window,
             PointCloudFactorData* factor_data,
             std::vector<std::vector<std::unique_ptr<FrameWindow>>>*
                 frame_window_partitions);

  void RebuildAfterPoseCorrection(
      SlidingVoxelMap* voxel_map, const std::vector<PoseState>& pose_window,
      const std::vector<SharedPointObservationList>& point_observation_window,
      int active_window_count, int local_window_size,
      std::vector<std::unique_ptr<FrameWindow>>* reusable_frame_windows,
      std::shared_ptr<const SlidingVoxelMapOptions> map_options);

  void ClearMap(
      SlidingVoxelMap* voxel_map, SlidingVoxelMapView* voxel_tree_view,
      std::vector<std::unique_ptr<FrameWindow>>* reusable_frame_windows);

  void BuildInitializationMap(
      SlidingVoxelMap* voxel_map, SlidingVoxelMapView* voxel_tree_view,
      const std::vector<PoseState>& pose_window,
      const std::vector<SharedPointObservationList>& point_observation_window,
      int active_window_count, int local_window_size,
      PointCloudFactorData* factor_data,
      std::vector<std::vector<std::unique_ptr<FrameWindow>>>*
          frame_window_partitions,
      std::shared_ptr<const SlidingVoxelMapOptions> map_options);

 private:
  common::ThreadPoolInterface* thread_pool_;
  std::size_t worker_count_;
};

}  // namespace map
}  // namespace atlas

#endif  // ATLAS_MAP_SLIDING_VOXEL_MAP_OPERATIONS_HPP_
