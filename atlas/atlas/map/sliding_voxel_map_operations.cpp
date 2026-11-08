/*
 * Copyright 2026 The Openbot Authors
 */
#include "map/sliding_voxel_map_operations.hpp"

#include <algorithm>
#include <exception>
#include <future>
#include <stdexcept>
#include <utility>

#include "map/point_observation_transform.hpp"

namespace atlas {
namespace map {

using ::atlas::map::PointObservationList;
using ::atlas::map::SharedPointObservationList;
using ::atlas::map::TransformPointObservationsToWorld;

using ::atlas::common::PoseState;

namespace {

std::vector<std::vector<SlidingVoxelTree*>> PartitionVoxelTrees(
    SlidingVoxelMap* voxel_map, std::size_t partition_count) {
  std::vector<std::vector<SlidingVoxelTree*>> partitions(partition_count);
  std::size_t partition_index = 0;
  for (auto& map_entry : *voxel_map) {
    partitions[partition_index].push_back(map_entry.second.get());
    if (partitions[partition_index].size() >=
            (voxel_map->size() + partition_count - 1) / partition_count &&
        partition_index + 1 < partition_count) {
      ++partition_index;
    }
  }
  return partitions;
}

template <typename TaskFunction>
void RunPartitions(common::ThreadPoolInterface* thread_pool,
                   std::size_t partition_count, TaskFunction task_function) {
  std::vector<std::future<void>> task_results;
  task_results.reserve(partition_count);
  std::exception_ptr first_exception;
  try {
    for (std::size_t partition_index = 0; partition_index < partition_count;
         ++partition_index) {
      task_results.push_back(
          thread_pool->Submit([partition_index, &task_function] {
            task_function(partition_index);
          }));
    }
  } catch (...) {
    first_exception = std::current_exception();
  }
  for (std::future<void>& task_result : task_results) {
    try {
      task_result.get();
    } catch (...) {
      if (first_exception == nullptr) {
        first_exception = std::current_exception();
      }
    }
  }
  if (first_exception != nullptr) {
    std::rethrow_exception(first_exception);
  }
}

}  // namespace

SlidingVoxelMapOperations::SlidingVoxelMapOperations(
    common::ThreadPoolInterface* thread_pool, std::size_t worker_count)
    : thread_pool_(thread_pool), worker_count_(worker_count) {
  if (thread_pool_ == nullptr || worker_count_ == 0) {
    throw std::invalid_argument(
        "sliding voxel map operations require a thread pool and workers");
  }
}

void SlidingVoxelMapOperations::Marginalize(
    SlidingVoxelMap* voxel_map, double trajectory_distance,
    int active_window_count, const std::vector<PoseState>* pose_window,
    PointCloudFactorData* factor_data,
    std::vector<std::unique_ptr<FrameWindow>>* reusable_frame_windows) {
  if (voxel_map == nullptr || pose_window == nullptr ||
      factor_data == nullptr || reusable_frame_windows == nullptr ||
      voxel_map->empty()) {
    return;
  }
  const std::size_t partition_count =
      std::min(worker_count_, voxel_map->size());
  std::vector<std::vector<SlidingVoxelTree*>> voxel_tree_partitions =
      PartitionVoxelTrees(voxel_map, partition_count);
  for (auto& map_entry : *voxel_map) {
    map_entry.second->jour = static_cast<float>(trajectory_distance);
  }
  RunPartitions(thread_pool_, partition_count,
                [&](std::size_t partition_index) {
                  for (SlidingVoxelTree* voxel_tree :
                       voxel_tree_partitions[partition_index]) {
                    voxel_tree->Marginalize(active_window_count, 1,
                                            *pose_window, *factor_data);
                  }
                });

  for (auto map_entry = voxel_map->begin(); map_entry != voxel_map->end();) {
    if (map_entry->second->isexist) {
      ++map_entry;
    } else {
      map_entry->second->clear_slwd(*reusable_frame_windows);
      map_entry = voxel_map->erase(map_entry);
    }
  }
}

void SlidingVoxelMapOperations::Recut(
    SlidingVoxelMap* voxel_map, int active_window_count,
    const std::vector<PoseState>* pose_window,
    PointCloudFactorData* factor_data,
    std::vector<std::vector<std::unique_ptr<FrameWindow>>>*
        frame_window_partitions) {
  if (voxel_map == nullptr || pose_window == nullptr ||
      factor_data == nullptr || frame_window_partitions == nullptr ||
      frame_window_partitions->empty() || voxel_map->empty()) {
    return;
  }
  const std::size_t partition_count =
      std::min(worker_count_, voxel_map->size());
  frame_window_partitions->resize(
      std::max(frame_window_partitions->size(), partition_count));
  std::vector<std::vector<SlidingVoxelTree*>> voxel_tree_partitions =
      PartitionVoxelTrees(voxel_map, partition_count);
  RunPartitions(
      thread_pool_, partition_count, [&](std::size_t partition_index) {
        for (SlidingVoxelTree* voxel_tree :
             voxel_tree_partitions[partition_index]) {
          voxel_tree->Recut(active_window_count, *pose_window,
                            (*frame_window_partitions)[partition_index]);
        }
      });

  for (std::size_t partition_index = 1;
       partition_index < frame_window_partitions->size(); ++partition_index) {
    auto& destination = (*frame_window_partitions)[0];
    auto& source = (*frame_window_partitions)[partition_index];
    destination.insert(destination.end(),
                       std::make_move_iterator(source.begin()),
                       std::make_move_iterator(source.end()));
    source.clear();
  }
  for (auto& map_entry : *voxel_map) {
    map_entry.second->CollectFactors(*factor_data);
  }
}

void SlidingVoxelMapOperations::RebuildAfterPoseCorrection(
    SlidingVoxelMap* voxel_map, const std::vector<PoseState>& pose_window,
    const std::vector<SharedPointObservationList>& point_observation_window,
    int active_window_count, int local_window_size,
    std::vector<std::unique_ptr<FrameWindow>>* reusable_frame_windows,
    std::shared_ptr<const SlidingVoxelMapOptions> map_options) {
  if (voxel_map == nullptr || reusable_frame_windows == nullptr ||
      map_options == nullptr || active_window_count <= 0) {
    return;
  }

  for (auto& map_entry : *voxel_map) {
    map_entry.second->clear_slwd(*reusable_frame_windows);
  }
  voxel_map->clear();

  const std::size_t available_window_count =
      std::min(pose_window.size(), point_observation_window.size());
  const std::size_t rebuild_window_count = std::min(
      static_cast<std::size_t>(active_window_count), available_window_count);
  for (std::size_t window_index = 0; window_index < rebuild_window_count;
       ++window_index) {
    const SharedPointObservationList& source_point_observations =
        point_observation_window[window_index];
    if (source_point_observations == nullptr) {
      continue;
    }

    PointObservationList world_point_observations = *source_point_observations;
    AlignedVectorList<3> world_points;
    TransformPointObservationsToWorld(pose_window[window_index],
                                      &world_point_observations, &world_points);
    auto point_observation_snapshot = std::make_shared<PointObservationList>(
        std::move(world_point_observations));
    SlidingVoxelMapView voxel_tree_view;
    CutVoxel(*voxel_map, point_observation_snapshot,
             static_cast<int>(window_index), voxel_tree_view, local_window_size,
             world_points, *reusable_frame_windows, map_options);

    for (const auto& map_entry : voxel_tree_view) {
      if (map_entry.second != nullptr) {
        map_entry.second->Recut(active_window_count, pose_window,
                                *reusable_frame_windows);
      }
    }
  }
}

void SlidingVoxelMapOperations::ClearMap(
    SlidingVoxelMap* voxel_map, SlidingVoxelMapView* voxel_tree_view,
    std::vector<std::unique_ptr<FrameWindow>>* reusable_frame_windows) {
  if (voxel_map == nullptr || voxel_tree_view == nullptr ||
      reusable_frame_windows == nullptr) {
    return;
  }
  for (auto& map_entry : *voxel_map) {
    map_entry.second->clear_slwd(*reusable_frame_windows);
  }
  voxel_map->clear();
  voxel_tree_view->clear();
}

void SlidingVoxelMapOperations::BuildInitializationMap(
    SlidingVoxelMap* voxel_map, SlidingVoxelMapView* voxel_tree_view,
    const std::vector<PoseState>& pose_window,
    const std::vector<SharedPointObservationList>& point_observation_window,
    int active_window_count, int local_window_size,
    PointCloudFactorData* factor_data,
    std::vector<std::vector<std::unique_ptr<FrameWindow>>>*
        frame_window_partitions,
    std::shared_ptr<const SlidingVoxelMapOptions> map_options) {
  if (voxel_map == nullptr || voxel_tree_view == nullptr ||
      factor_data == nullptr || frame_window_partitions == nullptr ||
      frame_window_partitions->empty() || map_options == nullptr ||
      active_window_count <= 0 || local_window_size <= 0) {
    return;
  }

  ClearMap(voxel_map, voxel_tree_view, &(*frame_window_partitions)[0]);
  factor_data->Clear();
  factor_data->window_count = local_window_size;

  const std::size_t available_window_count =
      std::min(pose_window.size(), point_observation_window.size());
  const std::size_t build_window_count = std::min(
      static_cast<std::size_t>(active_window_count), available_window_count);
  for (std::size_t window_index = 0; window_index < build_window_count;
       ++window_index) {
    const SharedPointObservationList& source_point_observations =
        point_observation_window[window_index];
    if (source_point_observations == nullptr) {
      continue;
    }

    PointObservationList world_point_observations = *source_point_observations;
    AlignedVectorList<3> world_points;
    TransformPointObservationsToWorld(pose_window[window_index],
                                      &world_point_observations, &world_points);
    auto point_observation_snapshot = std::make_shared<PointObservationList>(
        std::move(world_point_observations));
    CutVoxel(*voxel_map, point_observation_snapshot,
             static_cast<int>(window_index), *voxel_tree_view,
             local_window_size, world_points, (*frame_window_partitions)[0],
             map_options);
  }

  for (auto& map_entry : *voxel_map) {
    map_entry.second->Recut(active_window_count, pose_window,
                            (*frame_window_partitions)[0]);
    map_entry.second->CollectFactors(*factor_data);
  }
}

}  // namespace map
}  // namespace atlas
