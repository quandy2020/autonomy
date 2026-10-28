/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ARGUS_VXSLAM_LI_BA_OPTIMIZER_HPP_
#define ARGUS_VXSLAM_LI_BA_OPTIMIZER_HPP_

#include <Eigen/Core>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <thread>
#include <vector>

#include "common/math/so3.hpp"
#include "frontend/lidar/imu/imu_preintegration.hpp"
#include "frontend/lidar/imu/imu_state.hpp"
#include "frontend/lidar/local_mapping/lidar_factor.hpp"

namespace argus {
namespace vxslam {

#ifndef ARGUS_VXSLAM_MATH_ALIASES_
#define ARGUS_VXSLAM_MATH_ALIASES_
inline Eigen::Matrix3d Exp(const Eigen::Vector3d& a) { return math::Exp(a); }
inline Eigen::Matrix3d Exp(const Eigen::Vector3d& a, double dt) {
  return math::Exp(a, dt);
}
inline Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
  return math::SkewSymmetric(v);
}
#endif
#ifndef I33
#define I33 Eigen::Matrix3d::Identity()
#endif
#ifndef ARGUS_VXSLAM_DIM_DEFINED_
#define ARGUS_VXSLAM_DIM_DEFINED_
constexpr int DIM = kStateDim;
#endif
constexpr int DVEL = 6;

extern double imu_coef;

class LiBaOptimizer {
 public:
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

  double divide_thread(std::vector<ImuState>& x_stats, LidarFactor& voxhess,
                       std::deque<ImuPreintegration*>& imus_factor,
                       Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT) {
    int thd_num = 5;
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

    int tthd_num = thd_num;
    int g_size = voxhess.plvec_voxels.size();
    if (g_size < tthd_num) tthd_num = 1;
    double part = 1.0 * g_size / tthd_num;

    std::vector<std::thread*> mthreads(tthd_num);
    // for(int i=0; i<tthd_num; i++)
    for (int i = 1; i < tthd_num; i++)
      mthreads[i] =
          new std::thread(&LidarFactor::acc_evaluate2, &voxhess, x_stats,
                          part * i, part * (i + 1), std::ref(hessians[i]),
                          std::ref(jacobins[i]), std::ref(resis[i]));

    Eigen::MatrixXd jtj(2 * DIM, 2 * DIM);
    Eigen::VectorXd gg(2 * DIM);

    for (int i = 0; i < win_size - 1; i++) {
      jtj.setZero();
      gg.setZero();
      residual +=
          imus_factor[i]->Evaluate(x_stats[i], x_stats[i + 1], jtj, gg, true);
      Hess.block<DIM * 2, DIM * 2>(i * DIM, i * DIM) += jtj;
      JacT.block<DIM * 2, 1>(i * DIM, 0) += gg;
    }

    Eigen::Matrix<double, DIM, DIM> joc;
    Eigen::Matrix<double, DIM, 1> rr;
    joc.setIdentity();
    rr.setZero();

    Hess *= imu_coef;
    JacT *= imu_coef;
    residual *= (imu_coef * 0.5);

    // printf("resi: %lf\n", residual);

    for (int i = 0; i < tthd_num; i++) {
      // mthreads[i]->join();
      if (i != 0)
        mthreads[i]->join();
      else
        voxhess.acc_evaluate2(x_stats, 0, part, hessians[0], jacobins[0],
                              resis[0]);
      hess_plus(Hess, JacT, hessians[i], jacobins[i]);
      residual += resis[i];
      delete mthreads[i];
    }

    return residual;
  }

  double only_residual(std::vector<ImuState>& x_stats, LidarFactor& voxhess,
                       std::deque<ImuPreintegration*>& imus_factor) {
    double residual1 = 0, residual2 = 0;
    Eigen::MatrixXd jtj(2 * DIM, 2 * DIM);
    Eigen::VectorXd gg(2 * DIM);

    int thd_num = 5;
    std::vector<double> residuals(thd_num, 0);
    int g_size = voxhess.plvec_voxels.size();
    if (g_size < thd_num) {
      // printf("Too Less Voxel\n"); return false;
      thd_num = 1;
    }
    std::vector<std::thread*> mthreads(thd_num, nullptr);
    double part = 1.0 * g_size / thd_num;
    for (int i = 1; i < thd_num; i++)
      mthreads[i] = new std::thread(&LidarFactor::evaluate_only_residual,
                                    &voxhess, x_stats, part * i, part * (i + 1),
                                    std::ref(residuals[i]));

    for (int i = 0; i < win_size - 1; i++)
      residual1 +=
          imus_factor[i]->Evaluate(x_stats[i], x_stats[i + 1], jtj, gg, false);
    residual1 *= (imu_coef * 0.5);

    for (int i = 0; i < thd_num; i++) {
      if (i != 0) {
        mthreads[i]->join();
        delete mthreads[i];
      } else
        voxhess.evaluate_only_residual(x_stats, part * i, part * (i + 1),
                                       residuals[i]);
      residual2 += residuals[i];
    }

    return (residual1 + residual2);
  }

  void DampingIter(std::vector<ImuState>& x_stats, LidarFactor& voxhess,
                   std::deque<ImuPreintegration*>& imus_factor,
                   Eigen::MatrixXd* hess) {
    win_size = voxhess.win_size;
    jac_leng = win_size * 6;
    imu_leng = win_size * DIM;
    double u = 0.01, v = 2;
    Eigen::MatrixXd D(imu_leng, imu_leng), Hess(imu_leng, imu_leng);
    Eigen::VectorXd JacT(imu_leng), dxi(imu_leng);
    hess->resize(imu_leng, imu_leng);

    D.setIdentity();
    double residual1, residual2, q;
    bool is_calc_hess = true;
    std::vector<ImuState> x_stats_temp = x_stats;

    double hesstime = 0;
    double resitime = 0;

    // for(int i=0; i<10; i++)
    for (int i = 0; i < 3; i++) {
      if (is_calc_hess) {
        double tm = std::chrono::duration<double>(
                        std::chrono::steady_clock::now().time_since_epoch())
                        .count();
        residual1 = divide_thread(x_stats, voxhess, imus_factor, Hess, JacT);
        hesstime += std::chrono::duration<double>(
                        std::chrono::steady_clock::now().time_since_epoch())
                        .count() -
                    tm;
        *hess = Hess;
      }

      Hess.topRows(DIM).setZero();
      Hess.leftCols(DIM).setZero();
      Hess.block<DIM, DIM>(0, 0).setIdentity();
      JacT.head(DIM).setZero();

      D.diagonal() = Hess.diagonal();
      dxi = (Hess + u * D).ldlt().solve(-JacT);

      for (int j = 0; j < win_size; j++) {
        x_stats_temp[j].R = x_stats[j].R * Exp(dxi.block<3, 1>(DIM * j, 0));
        x_stats_temp[j].p = x_stats[j].p + dxi.block<3, 1>(DIM * j + 3, 0);
        x_stats_temp[j].v = x_stats[j].v + dxi.block<3, 1>(DIM * j + 6, 0);
        x_stats_temp[j].bg = x_stats[j].bg + dxi.block<3, 1>(DIM * j + 9, 0);
        x_stats_temp[j].ba = x_stats[j].ba + dxi.block<3, 1>(DIM * j + 12, 0);
      }

      for (int j = 0; j < win_size - 1; j++)
        imus_factor[j]->ApplyDelta(dxi.block<DIM, 1>(DIM * j, 0));

      double q1 = 0.5 * dxi.dot(u * D * dxi - JacT);

      double tl1 = std::chrono::duration<double>(
                       std::chrono::steady_clock::now().time_since_epoch())
                       .count();
      residual2 = only_residual(x_stats_temp, voxhess, imus_factor);
      double tl2 = std::chrono::duration<double>(
                       std::chrono::steady_clock::now().time_since_epoch())
                       .count();
      // printf("onlyresi: %lf\n", tl2-tl1);
      resitime += tl2 - tl1;

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

    // printf("ba: %lf %lf %zu\n", hesstime, resitime,
    // voxhess.plvec_voxels.size());
  }
};

// The LiDAR-Inertial BA optimizer with gravity optimization
class GravityLiBaOptimizer {
 public:
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

  double divide_thread(std::vector<ImuState>& x_stats, LidarFactor& voxhess,
                       std::deque<ImuPreintegration*>& imus_factor,
                       Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT) {
    int thd_num = 5;
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

    int tthd_num = thd_num;
    int g_size = voxhess.plvec_voxels.size();
    if (g_size < tthd_num) tthd_num = 1;
    double part = 1.0 * g_size / tthd_num;

    std::vector<std::thread*> mthreads(tthd_num);
    // for(int i=0; i<tthd_num; i++)
    for (int i = 1; i < tthd_num; i++)
      mthreads[i] =
          new std::thread(&LidarFactor::acc_evaluate2, &voxhess, x_stats,
                          part * i, part * (i + 1), std::ref(hessians[i]),
                          std::ref(jacobins[i]), std::ref(resis[i]));

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

    Hess *= imu_coef;
    JacT *= imu_coef;
    residual *= (imu_coef * 0.5);

    // printf("resi: %lf\n", residual);

    for (int i = 0; i < tthd_num; i++) {
      // mthreads[i]->join();
      if (i != 0)
        mthreads[i]->join();
      else
        voxhess.acc_evaluate2(x_stats, 0, part, hessians[0], jacobins[0],
                              resis[0]);
      hess_plus(Hess, JacT, hessians[i], jacobins[i]);
      residual += resis[i];
      delete mthreads[i];
    }

    return residual;
  }

  double only_residual(std::vector<ImuState>& x_stats, LidarFactor& voxhess,
                       std::deque<ImuPreintegration*>& imus_factor) {
    double residual1 = 0, residual2 = 0;
    Eigen::MatrixXd jtj(2 * DIM, 2 * DIM);
    Eigen::VectorXd gg(2 * DIM);

    int thd_num = 5;
    std::vector<double> residuals(thd_num, 0);
    int g_size = voxhess.plvec_voxels.size();
    if (g_size < thd_num) {
      // printf("Too Less Voxel\n"); return false;
      thd_num = 1;
    }
    std::vector<std::thread*> mthreads(thd_num, nullptr);
    double part = 1.0 * g_size / thd_num;
    for (int i = 1; i < thd_num; i++)
      mthreads[i] = new std::thread(&LidarFactor::evaluate_only_residual,
                                    &voxhess, x_stats, part * i, part * (i + 1),
                                    std::ref(residuals[i]));

    for (int i = 0; i < win_size - 1; i++)
      residual1 += imus_factor[i]->EvaluateGravity(x_stats[i], x_stats[i + 1],
                                                   jtj, gg, false);
    residual1 *= (imu_coef * 0.5);

    for (int i = 0; i < thd_num; i++) {
      if (i != 0) {
        mthreads[i]->join();
        delete mthreads[i];
      } else
        voxhess.evaluate_only_residual(x_stats, part * i, part * (i + 1),
                                       residuals[i]);
      residual2 += residuals[i];
    }

    return (residual1 + residual2);
  }

  void DampingIter(std::vector<ImuState>& x_stats, LidarFactor& voxhess,
                   std::deque<ImuPreintegration*>& imus_factor,
                   std::vector<double>& resis, Eigen::MatrixXd* hess,
                   int max_iter = 2) {
    win_size = voxhess.win_size;
    jac_leng = win_size * 6;
    imu_leng = win_size * DIM + 3;
    double u = 0.01, v = 2;
    Eigen::MatrixXd D(imu_leng, imu_leng), Hess(imu_leng, imu_leng);
    Eigen::VectorXd JacT(imu_leng), dxi(imu_leng);

    D.setIdentity();
    double residual1, residual2, q;
    bool is_calc_hess = true;
    std::vector<ImuState> x_stats_temp = x_stats;

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
};

// 10 scans merge into a keyframe

}  // namespace vxslam
}  // namespace argus

#endif
