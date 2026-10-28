/*
 * Copyright 2026 The Openbot Authors
 */
#include "map/voxel/map_ops.hpp"

#include <future>

namespace argus {
namespace vxslam {

void MarginalizeSlidingVoxelMap(
    std::unordered_map<VoxelKey, SlidingOctree*>* feature_map,
    double traveled_distance, int window_count, std::vector<ImuState>* states,
    LidarFactor* factor_collection,
    std::vector<FrameWindow*>* frame_window_pool,
    common::ThreadPoolInterface* thread_pool, int worker_count) {
  if (feature_map == nullptr || states == nullptr ||
      factor_collection == nullptr || frame_window_pool == nullptr) {
    return;
  }
  int partition_count = worker_count > 0 ? worker_count : 1;
  const int voxel_count = static_cast<int>(feature_map->size());
  if (voxel_count == 0) {
    return;
  }
  if (voxel_count < partition_count || thread_pool == nullptr) {
    partition_count = 1;
  }
  std::vector<std::vector<SlidingOctree*>> partitions(
      static_cast<size_t>(partition_count));
  const double target_partition_size =
      static_cast<double>(voxel_count) / partition_count;
  int partition_index = 0;
  for (auto& entry : *feature_map) {
    entry.second->jour = static_cast<float>(traveled_distance);
    partitions[static_cast<size_t>(partition_index)].push_back(entry.second);
    if (static_cast<double>(
            partitions[static_cast<size_t>(partition_index)].size()) >=
            target_partition_size &&
        partition_index < partition_count - 1) {
      ++partition_index;
    }
  }

  const auto marginalize_partition =
      [&](const std::vector<SlidingOctree*>& partition) {
        for (SlidingOctree* octree : partition) {
          octree->Marginalize(window_count, 1, *states, *factor_collection);
        }
      };
  std::vector<std::future<void>> task_results;
  for (int index = 1; index < partition_count; ++index) {
    task_results.push_back(thread_pool->Submit(
        [&, index] { marginalize_partition(partitions[index]); }));
  }
  marginalize_partition(partitions[0]);
  for (std::future<void>& task_result : task_results) {
    task_result.get();
  }

  for (auto iterator = feature_map->begin(); iterator != feature_map->end();) {
    if (iterator->second->isexist) {
      ++iterator;
    } else {
      iterator->second->clear_slwd(*frame_window_pool);
      iterator = feature_map->erase(iterator);
    }
  }
}

void RecutSlidingVoxelMap(
    std::unordered_map<VoxelKey, SlidingOctree*>* feature_map, int window_count,
    std::vector<ImuState>* states, LidarFactor* factor_collection,
    std::vector<std::vector<FrameWindow*>>* frame_window_pools,
    common::ThreadPoolInterface* thread_pool, int worker_count) {
  if (feature_map == nullptr || states == nullptr ||
      factor_collection == nullptr || frame_window_pools == nullptr ||
      frame_window_pools->empty()) {
    return;
  }
  int partition_count = worker_count > 0 ? worker_count : 1;
  if (static_cast<int>(frame_window_pools->size()) < partition_count) {
    frame_window_pools->resize(static_cast<size_t>(partition_count));
  }
  const int voxel_count = static_cast<int>(feature_map->size());
  if (voxel_count < partition_count || thread_pool == nullptr) {
    partition_count = 1;
  }
  std::vector<std::vector<SlidingOctree*>> partitions(
      static_cast<size_t>(partition_count));
  const double target_partition_size =
      static_cast<double>(voxel_count) / partition_count;
  int partition_index = 0;
  for (auto& entry : *feature_map) {
    partitions[static_cast<size_t>(partition_index)].push_back(entry.second);
    if (static_cast<double>(
            partitions[static_cast<size_t>(partition_index)].size()) >=
            target_partition_size &&
        partition_index < partition_count - 1) {
      ++partition_index;
    }
  }

  const auto recut_partition = [&](int index) {
    for (SlidingOctree* octree : partitions[static_cast<size_t>(index)]) {
      octree->Recut(window_count, *states,
                    (*frame_window_pools)[static_cast<size_t>(index)]);
    }
  };
  std::vector<std::future<void>> task_results;
  for (int index = 1; index < partition_count; ++index) {
    task_results.push_back(
        thread_pool->Submit([&, index] { recut_partition(index); }));
  }
  recut_partition(0);
  for (std::future<void>& task_result : task_results) {
    task_result.get();
  }

  for (size_t index = 1; index < frame_window_pools->size(); ++index) {
    (*frame_window_pools)[0].insert((*frame_window_pools)[0].end(),
                                    (*frame_window_pools)[index].begin(),
                                    (*frame_window_pools)[index].end());
    (*frame_window_pools)[index].clear();
  }
  for (auto& entry : *feature_map) {
    entry.second->CollectFactors(*factor_collection);
  }
}

}  // namespace vxslam
}  // namespace argus
