/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_FRONTEND_GRAVITY_CONSTRAINED_BUNDLE_ADJUSTER_HPP_
#define ATLAS_FRONTEND_GRAVITY_CONSTRAINED_BUNDLE_ADJUSTER_HPP_

#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <exception>
#include <future>
#include <vector>

#include "common/math/so3.hpp"
#include "common/thread_pool.hpp"
#include "frontend/inertial/inertial_preintegrator.hpp"
#include "frontend/inertial/inertial_state.hpp"
#include "frontend/local_mapping/bundle_adjustment_support.hpp"
#include "map/point_cloud_factor.hpp"

namespace atlas {
namespace frontend {
namespace local_mapping {

using ::atlas::map::PointCloudFactor;

using ::atlas::frontend::inertial::InertialPreintegrator;
using ::atlas::frontend::inertial::InertialState;
using ::atlas::frontend::inertial::kInertialStateDimension;

class GravityConstrainedBundleAdjuster {
 public:
  explicit GravityConstrainedBundleAdjuster(
      double inertial_factor_coefficient = 1e-4,
      common::ThreadPoolInterface* task_executor = nullptr,
      int worker_thread_count = 1)
      : inertial_factor_coefficient_(inertial_factor_coefficient),
        task_executor_(task_executor),
        worker_thread_count_(std::max(worker_thread_count, 1)) {}

  int win_size, jac_leng, imu_leng;

  void hess_plus(Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT,
                 Eigen::MatrixXd& hs, Eigen::VectorXd& js) {
    for (int i = 0; i < win_size; i++) {
      JacT.block<DVEL, 1>(i * DIM, 0) += js.block<DVEL, 1>(i * DVEL, 0);
      for (int j = 0; j < win_size; j++)
        Hess.block<DVEL, DVEL>(i * DIM, j * DIM) +=
            hs.block<DVEL, DVEL>(i * DVEL, j * DVEL);
    }
  }

  double divide_thread(std::vector<InertialState>& x_stats,
                       PointCloudFactor& voxhess,
                       std::deque<InertialPreintegrator*>& imus_factor,
                       Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT) {
    const std::vector<common::PoseState> pose_window = MakePoseWindow(x_stats);
    const int thd_num = PartitionCount(voxhess.point_clusters_by_window.size());
    double residual = 0;
    Hess.setZero();
    JacT.setZero();
    std::vector<Eigen::MatrixXd> hessians(thd_num);
    std::vector<Eigen::VectorXd> jacobins(thd_num);
    std::vector<double> resis(thd_num, 0);

    for (int i = 0; i < thd_num; i++) {
      hessians[i].resize(jac_leng, jac_leng);
      jacobins[i].resize(jac_leng);
    }

    int g_size = voxhess.point_clusters_by_window.size();
    double part = 1.0 * g_size / thd_num;
    std::vector<std::future<void>> partition_tasks;
    for (int i = 1; i < thd_num; ++i) {
      partition_tasks.push_back(task_executor_->Submit([&, i] {
        voxhess.AccumulateEvaluation(pose_window, part * i, part * (i + 1),
                                     hessians[i], jacobins[i], resis[i]);
      }));
    }

    Eigen::MatrixXd jtj(2 * DIM + 3, 2 * DIM + 3);
    Eigen::VectorXd gg(2 * DIM + 3);

    for (int i = 0; i < win_size - 1; i++) {
      jtj.setZero();
      gg.setZero();
      residual += imus_factor[i]->EvaluateGravity(x_stats[i], x_stats[i + 1],
                                                  jtj, gg, true);
      Hess.block<DIM * 2, DIM * 2>(i * DIM, i * DIM) +=
          jtj.block<2 * DIM, 2 * DIM>(0, 0);
      Hess.block<DIM * 2, 3>(i * DIM, imu_leng - 3) +=
          jtj.block<2 * DIM, 3>(0, 2 * DIM);
      Hess.block<3, DIM * 2>(imu_leng - 3, i * DIM) +=
          jtj.block<3, 2 * DIM>(2 * DIM, 0);
      Hess.block<3, 3>(imu_leng - 3, imu_leng - 3) +=
          jtj.block<3, 3>(2 * DIM, 2 * DIM);

      JacT.block<DIM * 2, 1>(i * DIM, 0) += gg.head(2 * DIM);
      JacT.tail(3) += gg.tail(3);
    }

    Eigen::Matrix<double, DIM, DIM> joc;
    Eigen::Matrix<double, DIM, 1> rr;
    joc.setIdentity();
    rr.setZero();

    Hess *= inertial_factor_coefficient_;
    JacT *= inertial_factor_coefficient_;
    residual *= (inertial_factor_coefficient_ * 0.5);

    // printf("resi: %lf\n", residual);

    voxhess.AccumulateEvaluation(pose_window, 0, part, hessians[0], jacobins[0],
                                 resis[0]);
    WaitForPartitionTasks(partition_tasks);
    for (int i = 0; i < thd_num; i++) {
      hess_plus(Hess, JacT, hessians[i], jacobins[i]);
      residual += resis[i];
    }

    return residual;
  }

  double only_residual(std::vector<InertialState>& x_stats,
                       PointCloudFactor& voxhess,
                       std::deque<InertialPreintegrator*>& imus_factor) {
    const std::vector<common::PoseState> pose_window = MakePoseWindow(x_stats);
    double residual1 = 0, residual2 = 0;
    Eigen::MatrixXd jtj(2 * DIM, 2 * DIM);
    Eigen::VectorXd gg(2 * DIM);

    const int thd_num = PartitionCount(voxhess.point_clusters_by_window.size());
    std::vector<double> residuals(thd_num, 0);
    int g_size = voxhess.point_clusters_by_window.size();
    double part = 1.0 * g_size / thd_num;
    std::vector<std::future<void>> partition_tasks;
    for (int i = 1; i < thd_num; ++i) {
      partition_tasks.push_back(task_executor_->Submit([&, i] {
        voxhess.EvaluateResidual(pose_window, part * i, part * (i + 1),
                                 residuals[i]);
      }));
    }

    for (int i = 0; i < win_size - 1; i++)
      residual1 += imus_factor[i]->EvaluateGravity(x_stats[i], x_stats[i + 1],
                                                   jtj, gg, false);
    residual1 *= (inertial_factor_coefficient_ * 0.5);

    voxhess.EvaluateResidual(pose_window, 0, part, residuals[0]);
    WaitForPartitionTasks(partition_tasks);
    for (int i = 0; i < thd_num; i++) {
      residual2 += residuals[i];
    }

    return (residual1 + residual2);
  }

  void Optimize(std::vector<InertialState>& x_stats, PointCloudFactor& voxhess,
                std::deque<InertialPreintegrator*>& imus_factor,
                std::vector<double>& resis, Eigen::MatrixXd* hess,
                int max_iter = 2) {
    win_size = voxhess.window_count;
    jac_leng = win_size * 6;
    imu_leng = win_size * DIM + 3;
    double u = 0.01, v = 2;
    Eigen::MatrixXd D(imu_leng, imu_leng), Hess(imu_leng, imu_leng);
    Eigen::VectorXd JacT(imu_leng), dxi(imu_leng);

    D.setIdentity();
    double residual1, residual2, q;
    bool is_calc_hess = true;
    std::vector<InertialState> x_stats_temp = x_stats;

    for (int i = 0; i < max_iter; i++) {
      if (is_calc_hess) {
        residual1 = divide_thread(x_stats, voxhess, imus_factor, Hess, JacT);
        *hess = Hess;
      }

      if (i == 0) resis.push_back(residual1);

      Hess.topRows(6).setZero();
      Hess.leftCols(6).setZero();
      Hess.block<6, 6>(0, 0).setIdentity();
      JacT.head(6).setZero();

      // Hess.rightCols(3).setZero();
      // Hess.bottomRows(3).setZero();
      // Hess.block<3, 3>(imu_leng-3, imu_leng-3).setIdentity();
      // JacT.tail(3).setZero();

      D.diagonal() = Hess.diagonal();
      dxi = (Hess + u * D).ldlt().solve(-JacT);

      x_stats_temp[0].g += dxi.tail(3);
      for (int j = 0; j < win_size; j++) {
        x_stats_temp[j].R = x_stats[j].R * Exp(dxi.block<3, 1>(DIM * j, 0));
        x_stats_temp[j].p = x_stats[j].p + dxi.block<3, 1>(DIM * j + 3, 0);
        x_stats_temp[j].v = x_stats[j].v + dxi.block<3, 1>(DIM * j + 6, 0);
        x_stats_temp[j].bg = x_stats[j].bg + dxi.block<3, 1>(DIM * j + 9, 0);
        x_stats_temp[j].ba = x_stats[j].ba + dxi.block<3, 1>(DIM * j + 12, 0);
        x_stats_temp[j].g = x_stats_temp[0].g;
      }

      for (int j = 0; j < win_size - 1; j++)
        imus_factor[j]->ApplyDelta(dxi.block<DIM, 1>(DIM * j, 0));

      double q1 = 0.5 * dxi.dot(u * D * dxi - JacT);
      residual2 = only_residual(x_stats_temp, voxhess, imus_factor);
      q = (residual1 - residual2);
      // printf("iter%d: (%lf %lf) u: %lf v: %.1lf q: %.2lf %lf %lf\n", i,
      // residual1, residual2, u, v, q/q1, q1, q);

      if (q > 0) {
        x_stats = x_stats_temp;
        double one_three = 1.0 / 3;

        q = q / q1;
        v = 2;
        q = 1 - pow(2 * q - 1, 3);
        u *= (q < one_three ? one_three : q);
        is_calc_hess = true;
      } else {
        u = u * v;
        v = 2 * v;
        is_calc_hess = false;

        for (int j = 0; j < win_size - 1; j++) {
          imus_factor[j]->dbg = imus_factor[j]->dbg_buf;
          imus_factor[j]->dba = imus_factor[j]->dba_buf;
        }
      }

      if (fabs((residual1 - residual2) / residual1) < 1e-6) break;
    }
    resis.push_back(residual2);
  }

 private:
  int PartitionCount(std::size_t voxel_count) const {
    if (task_executor_ == nullptr || voxel_count < 2) return 1;
    return std::min({5, worker_thread_count_, static_cast<int>(voxel_count)});
  }

  double inertial_factor_coefficient_;
  common::ThreadPoolInterface* task_executor_;
  int worker_thread_count_;
};

// 10 scans merge into a keyframe

}  // namespace local_mapping
}  // namespace frontend
}  // namespace atlas

#endif
