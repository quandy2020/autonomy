/*
 * Copyright 2026 The Openbot Authors
 * Voxel-SLAM OctreeGBA port (no ROS / PCL publishers).
 */
#ifndef ATLAS_MAP_VOXEL_GLOBAL_BUNDLE_ADJUSTMENT_OCTREE_HPP_
#define ATLAS_MAP_VOXEL_GLOBAL_BUNDLE_ADJUSTMENT_OCTREE_HPP_

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <future>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/thread_pool.hpp"
#include "frontend/lidar/imu/imu_state.hpp"
#include "frontend/lidar/local_mapping/lidar_factor.hpp"
#include "map/voxel/point_cluster.hpp"
#include "map/voxel/voxel_key_hash.hpp"
#include "sensor/point_types.hpp"

namespace atlas {
namespace vxslam {

enum class OctreeSubdivisionState { kLeaf, kSubdivided };

struct GlobalBundleAdjustmentOctreeConfiguration {
  double voxel_size = 1.0;
  double minimum_eigenvalue = 0.01;
  std::vector<double> eigenvalue_thresholds;
  int maximum_octree_layer = 2;
};

/** @brief Hierarchical voxel plane builder for global BA. */
class GlobalBundleAdjustmentOctree {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::vector<VectorList<3>> local_points_by_window;
  std::vector<VectorList<3>> world_points_by_window;
  PointCluster accumulated_world_point_cluster;
  int octree_layer = 0;
  OctreeSubdivisionState subdivision_state = OctreeSubdivisionState::kLeaf;
  int window_size = 0;
  std::shared_ptr<const GlobalBundleAdjustmentOctreeConfiguration>
      configuration;
  std::array<std::unique_ptr<GlobalBundleAdjustmentOctree>, 8> child_octrees;
  Eigen::Vector3d voxel_center = Eigen::Vector3d::Zero();
  double quarter_voxel_length = 0.0;
  bool is_planar = false;

  GlobalBundleAdjustmentOctree(
      int initial_octree_layer, int window_count,
      std::shared_ptr<const GlobalBundleAdjustmentOctreeConfiguration>
          configuration_options)
      : octree_layer(initial_octree_layer),
        window_size(window_count),
        configuration(std::move(configuration_options)) {
    local_points_by_window.resize(static_cast<size_t>(window_size));
    world_points_by_window.resize(static_cast<size_t>(window_size));
  }

  void Reset(int initial_octree_layer, int window_count) {
    octree_layer = initial_octree_layer;
    window_size = window_count;
    local_points_by_window.clear();
    world_points_by_window.clear();
    local_points_by_window.resize(static_cast<size_t>(window_size));
    world_points_by_window.resize(static_cast<size_t>(window_size));
    accumulated_world_point_cluster.Clear();
    subdivision_state = OctreeSubdivisionState::kLeaf;
    is_planar = false;
    for (int i = 0; i < 8; ++i) {
      child_octrees[static_cast<size_t>(i)].reset();
    }
  }

  bool IsPlanarVoxel(const Eigen::Vector3d& eig_values) const {
    if (octree_layer >=
        static_cast<int>(configuration->eigenvalue_thresholds.size())) {
      return eig_values[0] < configuration->minimum_eigenvalue;
    }
    return eig_values[0] < configuration->minimum_eigenvalue &&
           (eig_values[0] / eig_values[2]) <
               configuration
                   ->eigenvalue_thresholds[static_cast<size_t>(octree_layer)];
  }

  void AddPoint(int window_index, const Eigen::Vector3d& local_point,
                const Eigen::Vector3d& world_point) {
    local_points_by_window[static_cast<size_t>(window_index)].push_back(
        local_point);
    world_points_by_window[static_cast<size_t>(window_index)].push_back(
        world_point);
    accumulated_world_point_cluster.Push(world_point);
  }

  void SubdivideByOctant(
      std::vector<std::unique_ptr<GlobalBundleAdjustmentOctree>>* octree_pool) {
    for (int i = 0; i < window_size; ++i) {
      for (size_t j = 0;
           j < local_points_by_window[static_cast<size_t>(i)].size(); ++j) {
        Eigen::Vector3d& local_point =
            local_points_by_window[static_cast<size_t>(i)][j];
        Eigen::Vector3d& world_point =
            world_points_by_window[static_cast<size_t>(i)][j];
        int octant_coordinates[3] = {0, 0, 0};
        for (int k = 0; k < 3; ++k) {
          if (world_point[k] > voxel_center[k]) {
            octant_coordinates[k] = 1;
          }
        }
        const int octant_index = 4 * octant_coordinates[0] +
                                 2 * octant_coordinates[1] +
                                 octant_coordinates[2];
        if (child_octrees[octant_index] == nullptr) {
          if (octree_pool != nullptr && !octree_pool->empty()) {
            child_octrees[octant_index] = std::move(octree_pool->back());
            octree_pool->pop_back();
            child_octrees[octant_index]->Reset(octree_layer + 1, window_size);
          } else {
            child_octrees[octant_index] =
                std::make_unique<GlobalBundleAdjustmentOctree>(
                    octree_layer + 1, window_size, configuration);
          }
          child_octrees[octant_index]->voxel_center =
              voxel_center + Eigen::Vector3d(2 * octant_coordinates[0] - 1,
                                             2 * octant_coordinates[1] - 1,
                                             2 * octant_coordinates[2] - 1) *
                                 quarter_voxel_length;
          child_octrees[octant_index]->quarter_voxel_length =
              quarter_voxel_length / 2.0;
        }
        child_octrees[octant_index]->AddPoint(i, local_point, world_point);
      }
    }
  }

  void RebuildPlaneFactorsRecursively(
      LidarFactor* factor_collection,
      std::vector<std::unique_ptr<GlobalBundleAdjustmentOctree>>* octree_pool) {
    if (factor_collection == nullptr ||
        accumulated_world_point_cluster.N <= 10) {
      return;
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenvalue_solver(
        accumulated_world_point_cluster.cov());
    Eigen::Vector3d eigenvalues = eigenvalue_solver.eigenvalues();
    Eigen::Matrix3d eigenvectors = eigenvalue_solver.eigenvectors();
    is_planar = IsPlanarVoxel(eigenvalues);
    if (is_planar) {
      if (accumulated_world_point_cluster.N < 10) {
        return;
      }
      int nonempty_window_count = 0;
      for (int i = 0; i < window_size; ++i) {
        if (!local_points_by_window[static_cast<size_t>(i)].empty()) {
          ++nonempty_window_count;
        }
      }
      if (nonempty_window_count <= 1) {
        return;
      }
      if (eigenvalues[0] / eigenvalues[1] > 0.12) {
        return;
      }
      const double coefficient = 1.0;
      std::vector<PointCluster> point_clusters_by_window(
          static_cast<size_t>(window_size));
      for (int i = 0; i < window_size; ++i) {
        for (Eigen::Vector3d& v :
             local_points_by_window[static_cast<size_t>(i)]) {
          point_clusters_by_window[static_cast<size_t>(i)].Push(v);
        }
      }
      PointCluster fixed_point_cluster;
      factor_collection->push_voxel(
          point_clusters_by_window, fixed_point_cluster, coefficient,
          eigenvalues, eigenvectors, accumulated_world_point_cluster);
      return;
    }
    if (octree_layer >= configuration->maximum_octree_layer) {
      return;
    }
    SubdivideByOctant(octree_pool);
    subdivision_state = OctreeSubdivisionState::kSubdivided;
    for (int i = 0; i < 8; ++i) {
      if (child_octrees[i] != nullptr) {
        child_octrees[static_cast<size_t>(i)]->RebuildPlaneFactorsRecursively(
            factor_collection, octree_pool);
      }
    }
  }

  void MoveChildOctreesToReusablePool(
      std::vector<std::unique_ptr<GlobalBundleAdjustmentOctree>>* octree_pool) {
    if (subdivision_state != OctreeSubdivisionState::kSubdivided ||
        octree_pool == nullptr) {
      return;
    }
    for (int i = 0; i < 8; ++i) {
      if (child_octrees[i] != nullptr) {
        child_octrees[static_cast<size_t>(i)]->MoveChildOctreesToReusablePool(
            octree_pool);
        octree_pool->push_back(
            std::move(child_octrees[static_cast<size_t>(i)]));
      }
    }
  }

  static void InsertPointCloudIntoVoxelMap(
      std::unordered_map<map::voxel::VoxelKey,
                         std::unique_ptr<GlobalBundleAdjustmentOctree>>*
          voxel_octrees,
      const ImuState& current_pose, const sensor::PointCloudXYZI& point_cloud,
      int window_index, int window_size,
      const std::shared_ptr<const GlobalBundleAdjustmentOctreeConfiguration>&
          configuration) {
    if (voxel_octrees == nullptr || configuration == nullptr ||
        !std::isfinite(configuration->voxel_size) ||
        configuration->voxel_size <= 0.0 || window_size <= 0 ||
        window_index < 0 || window_index >= window_size) {
      return;
    }
    for (const sensor::PointType& sensor_point : point_cloud.points) {
      Eigen::Vector3d local(sensor_point.x, sensor_point.y, sensor_point.z);
      Eigen::Vector3d world = current_pose.R * local + current_pose.p;
      std::array<int64_t, 3> voxel_coordinates;
      bool has_valid_voxel_coordinates = true;
      constexpr double minimum_voxel_coordinate = -9223372036854775808.0;
      constexpr double maximum_voxel_coordinate_exclusive =
          9223372036854775808.0;
      for (int axis = 0; axis < 3; ++axis) {
        const double voxel_coordinate = world[axis] / configuration->voxel_size;
        if (!std::isfinite(voxel_coordinate) ||
            voxel_coordinate < minimum_voxel_coordinate ||
            voxel_coordinate >= maximum_voxel_coordinate_exclusive) {
          has_valid_voxel_coordinates = false;
          break;
        }
        voxel_coordinates[static_cast<size_t>(axis)] =
            static_cast<int64_t>(std::floor(voxel_coordinate));
      }
      if (!has_valid_voxel_coordinates) {
        continue;
      }
      const map::voxel::VoxelKey position(
          voxel_coordinates[0], voxel_coordinates[1], voxel_coordinates[2]);
      auto iter = voxel_octrees->find(position);
      if (iter != voxel_octrees->end()) {
        iter->second->AddPoint(window_index, local, world);
      } else {
        auto octree = std::make_unique<GlobalBundleAdjustmentOctree>(
            0, window_size, configuration);
        octree->AddPoint(window_index, local, world);
        octree->voxel_center =
            (Eigen::Vector3d(static_cast<double>(position.x),
                             static_cast<double>(position.y),
                             static_cast<double>(position.z)) +
             Eigen::Vector3d::Constant(0.5)) *
            configuration->voxel_size;
        octree->quarter_voxel_length = configuration->voxel_size / 4.0;
        (*voxel_octrees)[position] = std::move(octree);
      }
    }
  }
};

inline void RebuildVoxelPlaneFactors(
    std::unordered_map<map::voxel::VoxelKey,
                       std::unique_ptr<GlobalBundleAdjustmentOctree>>*
        voxel_octrees,
    LidarFactor* factor_collection, int worker_thread_count,
    common::ThreadPoolInterface* thread_pool) {
  if (voxel_octrees == nullptr || factor_collection == nullptr) {
    return;
  }
  if (voxel_octrees->empty()) {
    return;
  }
  worker_thread_count = std::max(worker_thread_count, 1);
  if (thread_pool == nullptr) {
    worker_thread_count = 1;
  }
  if (voxel_octrees->size() < static_cast<size_t>(worker_thread_count)) {
    worker_thread_count = static_cast<int>(voxel_octrees->size());
  }
  std::vector<std::vector<GlobalBundleAdjustmentOctree*>> octree_partitions(
      static_cast<size_t>(worker_thread_count));
  std::vector<LidarFactor> partition_factors;
  partition_factors.reserve(static_cast<size_t>(worker_thread_count));
  for (int partition_index = 0; partition_index < worker_thread_count;
       ++partition_index) {
    partition_factors.emplace_back(factor_collection->win_size);
  }
  const size_t map_size = voxel_octrees->size();
  const double partition_size =
      1.0 * map_size / std::max(worker_thread_count, 1);
  int partition_index = 0;
  for (auto& map_entry : *voxel_octrees) {
    octree_partitions[static_cast<size_t>(partition_index)].push_back(
        map_entry.second.get());
    if (static_cast<double>(
            octree_partitions[static_cast<size_t>(partition_index)].size()) >=
            partition_size &&
        partition_index < worker_thread_count - 1) {
      ++partition_index;
    }
  }

  auto recut_func = [](std::vector<GlobalBundleAdjustmentOctree*>* octree_roots,
                       LidarFactor* factor_collection) {
    std::vector<std::unique_ptr<GlobalBundleAdjustmentOctree>> octree_pool;
    for (GlobalBundleAdjustmentOctree* octree : *octree_roots) {
      octree->RebuildPlaneFactorsRecursively(factor_collection, &octree_pool);
      octree->MoveChildOctreesToReusablePool(&octree_pool);
      octree_pool.clear();
    }
  };

  if (thread_pool == nullptr) {
    recut_func(&octree_partitions[0], &partition_factors[0]);
    *factor_collection = std::move(partition_factors[0]);
    voxel_octrees->clear();
    return;
  }

  std::vector<std::future<void>> partition_results;
  partition_results.reserve(static_cast<size_t>(worker_thread_count));
  try {
    for (int partition_index = 0; partition_index < worker_thread_count;
         ++partition_index) {
      partition_results.push_back(thread_pool->Submit([&, partition_index] {
        recut_func(&octree_partitions[static_cast<size_t>(partition_index)],
                   &partition_factors[static_cast<size_t>(partition_index)]);
      }));
    }
  } catch (...) {
    for (std::future<void>& partition_result : partition_results) {
      partition_result.wait();
    }
    throw;
  }
  std::exception_ptr task_exception;
  for (std::future<void>& partition_result : partition_results) {
    try {
      partition_result.get();
    } catch (...) {
      if (task_exception == nullptr) {
        task_exception = std::current_exception();
      }
    }
  }
  if (task_exception != nullptr) {
    std::rethrow_exception(task_exception);
  }
  *factor_collection = std::move(partition_factors[0]);
  for (int partition_index = 1; partition_index < worker_thread_count;
       ++partition_index) {
    factor_collection->plvec_voxels.insert(
        factor_collection->plvec_voxels.end(),
        partition_factors[static_cast<size_t>(partition_index)]
            .plvec_voxels.begin(),
        partition_factors[static_cast<size_t>(partition_index)]
            .plvec_voxels.end());
    factor_collection->sig_vecs.insert(
        factor_collection->sig_vecs.end(),
        partition_factors[static_cast<size_t>(partition_index)]
            .sig_vecs.begin(),
        partition_factors[static_cast<size_t>(partition_index)].sig_vecs.end());
    factor_collection->coeffs.insert(
        factor_collection->coeffs.end(),
        partition_factors[static_cast<size_t>(partition_index)].coeffs.begin(),
        partition_factors[static_cast<size_t>(partition_index)].coeffs.end());
    factor_collection->eig_values.insert(
        factor_collection->eig_values.end(),
        partition_factors[static_cast<size_t>(partition_index)]
            .eig_values.begin(),
        partition_factors[static_cast<size_t>(partition_index)]
            .eig_values.end());
    factor_collection->eig_vectors.insert(
        factor_collection->eig_vectors.end(),
        partition_factors[static_cast<size_t>(partition_index)]
            .eig_vectors.begin(),
        partition_factors[static_cast<size_t>(partition_index)]
            .eig_vectors.end());
    factor_collection->pcr_adds.insert(
        factor_collection->pcr_adds.end(),
        partition_factors[static_cast<size_t>(partition_index)]
            .pcr_adds.begin(),
        partition_factors[static_cast<size_t>(partition_index)].pcr_adds.end());
  }
  voxel_octrees->clear();
}

}  // namespace vxslam
}  // namespace atlas

#endif
