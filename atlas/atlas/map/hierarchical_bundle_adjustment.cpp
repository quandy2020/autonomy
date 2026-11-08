/*
 * Copyright 2026 The Openbot Authors
 */
#include "map/hierarchical_bundle_adjustment.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "map/global_bundle_adjustment_octree.hpp"
#include "map/point_cloud_factor.hpp"
#include "map/point_cloud_factor_optimizer.hpp"

namespace atlas {
namespace map {

using ::atlas::common::PoseState;
using ::atlas::map::PointCloudFactor;
using ::atlas::map::PointCloudFactorOptimizer;
namespace {

GlobalBundleAdjustmentOctreeConfiguration MakeOctreeConfiguration(
    const GlobalBundleAdjustmentConfiguration& configuration) {
  GlobalBundleAdjustmentOctreeConfiguration octree_configuration;
  octree_configuration.voxel_size = configuration.voxel_size;
  octree_configuration.minimum_eigenvalue = configuration.minimum_eigenvalue;
  octree_configuration.eigenvalue_thresholds =
      configuration.eigenvalue_thresholds;
  octree_configuration.maximum_octree_layer =
      configuration.maximum_octree_layer;
  for (double& threshold : octree_configuration.eigenvalue_thresholds) {
    if (threshold > 1e-9) {
      threshold = 1.0 / threshold;
    }
  }
  return octree_configuration;
}

template <typename CloudAccessor>
double OptimizeClouds(const GlobalBundleAdjustmentConfiguration& configuration,
                      common::ThreadPoolInterface* thread_pool,
                      std::vector<PoseState>* poses,
                      CloudAccessor cloud_accessor) {
  if (thread_pool == nullptr || poses == nullptr || poses->empty() ||
      poses->size() >
          static_cast<std::size_t>(std::numeric_limits<int>::max() / 6)) {
    return -1.0;
  }
  const auto octree_configuration =
      std::make_shared<const GlobalBundleAdjustmentOctreeConfiguration>(
          MakeOctreeConfiguration(configuration));
  const int window_size = static_cast<int>(poses->size());

  double last_residual = 0.0;
  for (int iteration_index = 0;
       iteration_index < configuration.maximum_iteration_count;
       ++iteration_index) {
    std::unordered_map<::atlas::map::VoxelKey,
                       std::unique_ptr<GlobalBundleAdjustmentOctree>>
        voxel_octrees;
    for (int window_index = 0; window_index < window_size; ++window_index) {
      GlobalBundleAdjustmentOctree::InsertPointCloudIntoVoxelMap(
          &voxel_octrees, (*poses)[static_cast<size_t>(window_index)],
          cloud_accessor(static_cast<size_t>(window_index)), window_index,
          window_size, octree_configuration);
    }
    PointCloudFactor plane_factors(window_size);
    RebuildVoxelPlaneFactors(&voxel_octrees, &plane_factors,
                             configuration.worker_thread_count, thread_pool);
    if (plane_factors.point_clusters_by_window.size() < 5) {
      std::cerr << "HierarchicalBundleAdjustment: too few voxels\n";
      return last_residual;
    }
    PointCloudFactorOptimizer optimizer(thread_pool,
                                        configuration.worker_thread_count);
    std::vector<double> residual_history;
    Eigen::MatrixXd hessian;
    optimizer.Optimize(*poses, plane_factors, &hessian, residual_history,
                       /*maximum_iteration_count=*/3,
                       /*is_display=*/false);
    if (!residual_history.empty()) {
      last_residual = residual_history.back();
    }
  }
  return last_residual;
}

GlobalBundleAdjustmentConfiguration ValidateConfiguration(
    GlobalBundleAdjustmentConfiguration configuration) {
  if (!std::isfinite(configuration.voxel_size) ||
      configuration.voxel_size <= 0.0 ||
      !std::isfinite(configuration.minimum_eigenvalue) ||
      configuration.minimum_eigenvalue < 0.0 ||
      configuration.maximum_octree_layer < 0 ||
      configuration.maximum_iteration_count < 0 ||
      configuration.worker_thread_count <= 0) {
    throw std::invalid_argument("invalid hierarchical BA configuration");
  }
  for (double threshold : configuration.eigenvalue_thresholds) {
    if (!std::isfinite(threshold) || threshold <= 0.0) {
      throw std::invalid_argument(
          "invalid hierarchical BA eigenvalue threshold");
    }
  }
  return configuration;
}

}  // namespace

HierarchicalBundleAdjustment::HierarchicalBundleAdjustment(
    GlobalBundleAdjustmentConfiguration configuration)
    : configuration_(ValidateConfiguration(std::move(configuration))),
      thread_pool_(
          static_cast<std::size_t>(configuration_.worker_thread_count)) {}

double HierarchicalBundleAdjustment::Optimize(
    std::vector<PoseState>* poses,
    const std::vector<sensor::PointCloudXYZI>& clouds) {
  if (poses == nullptr || poses->size() != clouds.size()) {
    return -1.0;
  }
  return OptimizeClouds(
      configuration_, &thread_pool_, poses,
      [&clouds](std::size_t index) -> const sensor::PointCloudXYZI& {
        return clouds[index];
      });
}

double HierarchicalBundleAdjustment::Optimize(
    std::vector<PoseState>* poses,
    const std::vector<std::shared_ptr<const sensor::PointCloudXYZI>>& clouds) {
  if (poses == nullptr || poses->size() != clouds.size()) {
    return -1.0;
  }
  for (const auto& cloud : clouds) {
    if (!cloud) {
      return -1.0;
    }
  }
  return OptimizeClouds(
      configuration_, &thread_pool_, poses,
      [&clouds](std::size_t index) -> const sensor::PointCloudXYZI& {
        return *clouds[index];
      });
}

void HierarchicalBundleAdjustment::AddEdgesFromHierarchicalBundleAdjustment(
    std::vector<PoseState>* poses,
    const std::vector<BundleAdjustmentKeyframe>& keyframes,
    common::SessionPoseConstraints* pose_constraints,
    const std::vector<int>& session_ids, int maximum_iteration_count,
    int worker_thread_count,
    const GlobalBundleAdjustmentConfiguration& configuration,
    const GlobalBundleAdjustmentConfiguration* final_iteration_configuration) {
  if (poses == nullptr || pose_constraints == nullptr ||
      poses->size() != keyframes.size()) {
    return;
  }
  std::unordered_set<int> selected_session_ids(session_ids.begin(),
                                               session_ids.end());
  std::vector<const BundleAdjustmentKeyframe*> selected_keyframes;
  std::vector<PoseState> selected_poses;
  std::vector<std::size_t> selected_pose_indices;
  selected_keyframes.reserve(keyframes.size());
  selected_poses.reserve(keyframes.size());
  selected_pose_indices.reserve(keyframes.size());
  for (std::size_t keyframe_index = 0; keyframe_index < keyframes.size();
       ++keyframe_index) {
    const BundleAdjustmentKeyframe& keyframe = keyframes[keyframe_index];
    if (keyframe.session_id < 0 || keyframe.cloud == nullptr ||
        selected_session_ids.count(keyframe.session_id) == 0) {
      continue;
    }
    selected_keyframes.push_back(&keyframe);
    selected_poses.push_back((*poses)[keyframe_index]);
    selected_pose_indices.push_back(keyframe_index);
  }
  if (selected_keyframes.size() >
      static_cast<std::size_t>(std::numeric_limits<int>::max() / 6)) {
    return;
  }
  const int window_size = static_cast<int>(selected_keyframes.size());
  if (window_size < 2) {
    return;
  }

  Eigen::MatrixXd hessian;
  int convergence_stage = 0;
  double convergence_threshold = 0.05;
  const int maximum_solver_iterations = 4;
  const int partition_count = worker_thread_count > 0
                                  ? worker_thread_count
                                  : configuration.worker_thread_count;

  for (int iteration_index = 0; iteration_index < maximum_iteration_count;
       ++iteration_index) {
    const bool use_final_configuration =
        convergence_stage == 1 ||
        iteration_index == maximum_iteration_count - 1;
    const auto& active_configuration =
        use_final_configuration && final_iteration_configuration != nullptr
            ? *final_iteration_configuration
            : configuration;
    const auto octree_configuration =
        std::make_shared<const GlobalBundleAdjustmentOctreeConfiguration>(
            MakeOctreeConfiguration(active_configuration));
    std::unordered_map<::atlas::map::VoxelKey,
                       std::unique_ptr<GlobalBundleAdjustmentOctree>>
        voxel_octrees;
    for (int first_window_index = 0; first_window_index < window_size;
         ++first_window_index) {
      GlobalBundleAdjustmentOctree::InsertPointCloudIntoVoxelMap(
          &voxel_octrees,
          selected_poses[static_cast<size_t>(first_window_index)],
          *selected_keyframes[static_cast<size_t>(first_window_index)]->cloud,
          first_window_index, window_size, octree_configuration);
    }
    PointCloudFactor plane_factors(window_size);
    RebuildVoxelPlaneFactors(&voxel_octrees, &plane_factors, partition_count,
                             &thread_pool_);
    PointCloudFactorOptimizer optimizer(&thread_pool_, partition_count);
    optimizer.SetMaximumPartitionCount(partition_count);
    std::vector<double> residual_history;
    const bool has_converged =
        optimizer.Optimize(selected_poses, plane_factors, &hessian,
                           residual_history, maximum_solver_iterations, false);
    if (residual_history.size() >= 2 &&
        ((std::fabs(residual_history[0] - residual_history[1]) /
                  std::max(residual_history[0], 1e-12) <
              convergence_threshold &&
          has_converged) ||
         (iteration_index == maximum_iteration_count - 2 &&
          convergence_stage == 0))) {
      convergence_threshold = 0.01;
      if (convergence_stage == 0) {
        convergence_stage = 1;
      } else {
        break;
      }
    }
  }

  if (hessian.rows() < 6 * window_size || hessian.cols() < 6 * window_size) {
    return;
  }
  for (int first_window_index = 0; first_window_index < window_size - 1;
       ++first_window_index) {
    for (int second_window_index = first_window_index + 1;
         second_window_index < window_size; ++second_window_index) {
      bool is_informative = true;
      Eigen::Matrix<double, 6, 1> variances;
      for (int axis_index = 0; axis_index < 6; ++axis_index) {
        const double information_magnitude =
            std::fabs(hessian(6 * first_window_index + axis_index,
                              6 * second_window_index + axis_index));
        if (!std::isfinite(information_magnitude) ||
            information_magnitude < 1e-6) {
          is_informative = false;
          break;
        }
        variances[axis_index] = 1.0 / information_magnitude;
      }
      if (!is_informative) {
        continue;
      }
      const BundleAdjustmentKeyframe& first_keyframe =
          *selected_keyframes[static_cast<size_t>(first_window_index)];
      const BundleAdjustmentKeyframe& second_keyframe =
          *selected_keyframes[static_cast<size_t>(second_window_index)];
      const Eigen::Vector3d relative_translation =
          selected_poses[static_cast<size_t>(first_window_index)]
              .rotation.transpose() *
          (selected_poses[static_cast<size_t>(second_window_index)]
               .translation -
           selected_poses[static_cast<size_t>(first_window_index)].translation);
      const Eigen::Matrix3d relative_rotation =
          selected_poses[static_cast<size_t>(first_window_index)]
              .rotation.transpose() *
          selected_poses[static_cast<size_t>(second_window_index)].rotation;
      common::SessionPoseConstraint pose_constraint;
      pose_constraint.from_session_id = first_keyframe.session_id;
      pose_constraint.to_session_id = second_keyframe.session_id;
      pose_constraint.from_keyframe_id = first_keyframe.keyframe_id;
      pose_constraint.to_keyframe_id = second_keyframe.keyframe_id;
      pose_constraint.relative_pose = transform::Rigid3d(
          relative_translation, Eigen::Quaterniond(relative_rotation));
      pose_constraint.diagonal_variance = variances;
      pose_constraints->push_back(std::move(pose_constraint));
    }
  }

  for (std::size_t selected_index = 0;
       selected_index < selected_pose_indices.size(); ++selected_index) {
    (*poses)[selected_pose_indices[selected_index]] =
        selected_poses[selected_index];
  }
}

}  // namespace map
}  // namespace atlas
