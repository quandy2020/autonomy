/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_MAP_POINT_CLOUD_FACTOR_OPTIMIZER_HPP_
#define ATLAS_MAP_POINT_CLOUD_FACTOR_OPTIMIZER_HPP_

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <future>
#include <vector>

#include "common/math/so3.hpp"
#include "common/pose_state.hpp"
#include "common/thread_pool.hpp"
#include "map/point_cloud_factor.hpp"

namespace atlas {
namespace map {

using ::atlas::common::PoseState;

inline void WaitForPointCloudFactorTasks(
    std::vector<std::future<void>>& tasks) {
  std::exception_ptr first_exception;
  for (auto& task : tasks) {
    try {
      task.get();
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

// The LM optimizer for LiDAR BA
class PointCloudFactorOptimizer {
 public:
  void SetMaximumPartitionCount(int partition_count) {
    maximum_partition_count_ = std::max(partition_count, 1);
  }

  explicit PointCloudFactorOptimizer(
      common::ThreadPoolInterface* task_executor = nullptr,
      int worker_thread_count = 1)
      : task_executor_(task_executor),
        worker_thread_count_(std::max(worker_thread_count, 1)) {}

  double AccumulateLinearizedSystem(const std::vector<PoseState>& pose_window,
                                    PointCloudFactor& point_cloud_factor,
                                    Eigen::MatrixXd& hessian,
                                    Eigen::VectorXd& jacobian) {
    double residual = 0;
    hessian.setZero();
    jacobian.setZero();
    const int point_cluster_count =
        point_cloud_factor.point_clusters_by_window.size();
    const int partition_count =
        PartitionCount(static_cast<size_t>(point_cluster_count));
    std::vector<Eigen::MatrixXd> hessian_matrices(partition_count);
    std::vector<Eigen::VectorXd> jacobian_vectors(partition_count);

    for (int i = 0; i < partition_count; i++) {
      hessian_matrices[i].resize(hessian.rows(), hessian.cols());
      jacobian_vectors[i].resize(jacobian.size());
    }

    std::vector<double> residuals(partition_count, 0);
    const double partition_size = 1.0 * point_cluster_count / partition_count;
    std::vector<std::future<void>> partition_tasks;
    for (int i = 1; i < partition_count; ++i) {
      partition_tasks.push_back(task_executor_->Submit([&, i] {
        point_cloud_factor.AccumulateEvaluation(
            pose_window, partition_size * i, partition_size * (i + 1),
            hessian_matrices[i], jacobian_vectors[i], residuals[i]);
      }));
    }

    point_cloud_factor.AccumulateEvaluation(pose_window, 0, partition_size,
                                            hessian_matrices[0],
                                            jacobian_vectors[0], residuals[0]);
    WaitForPointCloudFactorTasks(partition_tasks);
    for (int i = 0; i < partition_count; i++) {
      hessian += hessian_matrices[i];
      jacobian += jacobian_vectors[i];
      residual += residuals[i];
    }

    return residual;
  }

  double EvaluateResidual(const std::vector<PoseState>& pose_window,
                          PointCloudFactor& point_cloud_factor) {
    double current_residual = 0;
    std::vector<double> residuals;
    const int point_cluster_count =
        point_cloud_factor.point_clusters_by_window.size();
    const int partition_count =
        PartitionCount(static_cast<size_t>(point_cluster_count));
    residuals.resize(partition_count);
    const double partition_size = 1.0 * point_cluster_count / partition_count;
    std::vector<std::future<void>> partition_tasks;
    for (int i = 1; i < partition_count; ++i) {
      partition_tasks.push_back(task_executor_->Submit([&, i] {
        point_cloud_factor.EvaluateResidual(pose_window, partition_size * i,
                                            partition_size * (i + 1),
                                            residuals[i]);
      }));
    }

    point_cloud_factor.EvaluateResidual(pose_window, 0, partition_size,
                                        residuals[0]);
    WaitForPointCloudFactorTasks(partition_tasks);
    for (int i = 0; i < partition_count; i++) {
      current_residual += residuals[i];
    }

    return current_residual;
  }

  bool Optimize(std::vector<PoseState>& pose_window,
                PointCloudFactor& point_cloud_factor,
                Eigen::MatrixXd* hessian_output,
                std::vector<double>& residual_history,
                int maximum_iteration_count = 3,
                bool display_iterations = false) {
    const int window_size = point_cloud_factor.window_count;
    const int jacobian_length = window_size * 6;

    double damping = 0.01;
    double damping_growth = 2.0;
    Eigen::MatrixXd damping_matrix(jacobian_length, jacobian_length),
        hessian(jacobian_length, jacobian_length);
    Eigen::VectorXd jacobian(jacobian_length), dxi(jacobian_length);
    hessian_output->resize(jacobian_length, jacobian_length);

    damping_matrix.setIdentity();
    double current_residual, candidate_residual;
    bool should_recompute_linearization = true;
    std::vector<PoseState> temporary_states = pose_window;

    bool is_converged = true;

    // double tt1 =
    // std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    // for(int i=0; i<10; i++)
    for (int i = 0; i < maximum_iteration_count; i++) {
      if (should_recompute_linearization) {
        current_residual = AccumulateLinearizedSystem(
            pose_window, point_cloud_factor, hessian, jacobian);
        *hessian_output = hessian;
      }

      if (i == 0) residual_history.push_back(current_residual);

      hessian.topRows(6).setZero();
      hessian.leftCols(6).setZero();
      hessian.block<6, 6>(0, 0).setIdentity();
      jacobian.head(6).setZero();

      damping_matrix.diagonal() = hessian.diagonal();
      dxi = (hessian + damping * damping_matrix).ldlt().solve(-jacobian);

      for (int j = 0; j < window_size; j++) {
        temporary_states[j].rotation =
            pose_window[j].rotation * math::Exp(dxi.block<3, 1>(6 * j, 0));
        temporary_states[j].translation =
            pose_window[j].translation + dxi.block<3, 1>(6 * j + 3, 0);
      }
      const double predicted_reduction =
          0.5 * dxi.dot(damping * damping_matrix * dxi - jacobian);

      candidate_residual =
          EvaluateResidual(temporary_states, point_cloud_factor);

      double actual_reduction = (current_residual - candidate_residual);
      if (display_iterations)
        printf("iteration %d: residuals (%lf %lf), reduction ratio %.2lf\n", i,
               current_residual, candidate_residual,
               actual_reduction / predicted_reduction);

      if (actual_reduction > 0) {
        pose_window = temporary_states;
        double one_three = 1.0 / 3;

        actual_reduction /= predicted_reduction;
        damping_growth = 2.0;
        actual_reduction = 1 - pow(2 * actual_reduction - 1, 3);
        damping *=
            (actual_reduction < one_three ? one_three : actual_reduction);
        should_recompute_linearization = true;
      } else {
        damping *= damping_growth;
        damping_growth *= 2.0;
        should_recompute_linearization = false;
        is_converged = false;
      }

      if (fabs((current_residual - candidate_residual) / current_residual) <
          1e-6)
        break;
    }
    residual_history.push_back(candidate_residual);
    return is_converged;
  }

 private:
  int PartitionCount(size_t voxel_count) const {
    if (task_executor_ == nullptr || voxel_count < 2) return 1;
    return std::min({maximum_partition_count_, worker_thread_count_,
                     static_cast<int>(voxel_count)});
  }

  common::ThreadPoolInterface* task_executor_;
  int worker_thread_count_;
  int maximum_partition_count_ = 2;
};

}  // namespace map
}  // namespace atlas

#endif
