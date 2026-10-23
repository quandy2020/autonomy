/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_VXSLAM_LIDAR_FACTOR_HPP_
#define ATLAS_VXSLAM_LIDAR_FACTOR_HPP_

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <thread>
#include <vector>

#include "common/math/so3.hpp"
#include "frontend/lidar/imu/imu_state.hpp"
#include "frontend/lidar/odometry/point_var.hpp"
#include "map/voxel/point_cluster.hpp"

namespace atlas {
namespace vxslam {

#ifndef ATLAS_VXSLAM_MATH_ALIASES_
#define ATLAS_VXSLAM_MATH_ALIASES_
inline Eigen::Matrix3d Exp(const Eigen::Vector3d& a) { return math::Exp(a); }
inline Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
  return math::SkewSymmetric(v);
}
#endif
#ifndef I33
#define I33 Eigen::Matrix3d::Identity()
#endif

inline void Bf_var(const PointVar& pv, Eigen::Matrix<double, 9, 9>& bcov,
                   const Eigen::Vector3d& vec) {
  Eigen::Matrix<double, 6, 3> Bi;
  // Eigen::Vector3d &vec = pv.world;
  Bi << 2 * vec(0), 0, 0, vec(1), vec(0), 0, vec(2), 0, vec(0), 0, 2 * vec(1),
      0, 0, vec(2), vec(1), 0, 0, 2 * vec(2);
  Eigen::Matrix<double, 6, 3> Biup = Bi * pv.var;
  bcov.block<6, 6>(0, 0) = Biup * Bi.transpose();
  bcov.block<6, 3>(0, 6) = Biup;
  bcov.block<3, 6>(6, 0) = Biup.transpose();
  bcov.block<3, 3>(6, 6) = pv.var;
}

// The LiDAR BA factor in optimization
class LidarFactor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::vector<PointCluster> sig_vecs;
  std::vector<std::vector<PointCluster>> plvec_voxels;
  std::vector<double> coeffs;
  VectorList<3> eig_values;
  MatrixList<3> eig_vectors;
  std::vector<PointCluster> pcr_adds;
  int win_size;

  LidarFactor(int _w) : win_size(_w) {}

  void push_voxel(std::vector<PointCluster>& vec_orig, PointCluster& fix,
                  double coe, Eigen::Vector3d& eig_value,
                  Eigen::Matrix3d& eig_vector, PointCluster& pcr_add) {
    plvec_voxels.push_back(vec_orig);
    sig_vecs.push_back(fix);
    coeffs.push_back(coe);
    eig_values.push_back(eig_value);
    eig_vectors.push_back(eig_vector);
    pcr_adds.push_back(pcr_add);
  }

  void acc_evaluate2(const std::vector<ImuState>& xs, int head, int end,
                     Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT,
                     double& residual) {
    Hess.setZero();
    JacT.setZero();
    residual = 0;
    std::vector<PointCluster> sig_tran(win_size);
    const int kk = 0;

    VectorList<3> viRiTuk(win_size);
    MatrixList<3> viRiTukukT(win_size);

    std::vector<Eigen::Matrix<double, 3, 6>,
                Eigen::aligned_allocator<Eigen::Matrix<double, 3, 6>>>
        Auk(win_size);
    Eigen::Matrix3d umumT;

    for (int a = head; a < end; a++) {
      std::vector<PointCluster>& sig_orig = plvec_voxels[a];
      double coe = coeffs[a];

      // PointCluster sig = sig_vecs[a];
      // for(int i=0; i<win_size; i++)
      // if(sig_orig[i].N != 0)
      // {
      //   sig_tran[i].transform(sig_orig[i], xs[i]);
      //   sig += sig_tran[i];
      // }

      // const Eigen::Vector3d &vBar = sig.v / sig.N;
      // Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(sig.P/sig.N - vBar
      // * vBar.transpose()); const Eigen::Vector3d &lmbd = saes.eigenvalues();
      // const Eigen::Matrix3d &U = saes.eigenvectors();
      // int NN = sig.N;

      Eigen::Vector3d lmbd = eig_values[a];
      Eigen::Matrix3d U = eig_vectors[a];
      int NN = pcr_adds[a].N;
      Eigen::Vector3d vBar = pcr_adds[a].v / NN;

      Eigen::Vector3d u[3] = {U.col(0), U.col(1), U.col(2)};
      Eigen::Vector3d& uk = u[kk];
      Eigen::Matrix3d ukukT = uk * uk.transpose();
      umumT.setZero();
      for (int i = 0; i < 3; i++)
        if (i != kk)
          umumT += 2.0 / (lmbd[kk] - lmbd[i]) * u[i] * u[i].transpose();

      for (int i = 0; i < win_size; i++)
        // for(int i=1; i<win_size; i++)
        if (sig_orig[i].N != 0) {
          Eigen::Matrix3d Pi = sig_orig[i].P;
          Eigen::Vector3d vi = sig_orig[i].v;
          Eigen::Matrix3d Ri = xs[i].R;
          double ni = sig_orig[i].N;

          Eigen::Matrix3d vihat;
          vihat << SKEW_SYM_MATRX(vi);
          Eigen::Vector3d RiTuk = Ri.transpose() * uk;
          Eigen::Matrix3d RiTukhat;
          RiTukhat << SKEW_SYM_MATRX(RiTuk);

          Eigen::Vector3d PiRiTuk = Pi * RiTuk;
          viRiTuk[i] = vihat * RiTuk;
          viRiTukukT[i] = viRiTuk[i] * uk.transpose();

          Eigen::Vector3d ti_v = xs[i].p - vBar;
          double ukTti_v = uk.dot(ti_v);

          Eigen::Matrix3d combo1 = hat(PiRiTuk) + vihat * ukTti_v;
          Eigen::Vector3d combo2 = Ri * vi + ni * ti_v;
          Auk[i].block<3, 3>(0, 0) =
              (Ri * Pi + ti_v * vi.transpose()) * RiTukhat - Ri * combo1;
          Auk[i].block<3, 3>(0, 3) =
              combo2 * uk.transpose() + combo2.dot(uk) * I33;
          Auk[i] /= NN;

          const Eigen::Matrix<double, 6, 1>& jjt = Auk[i].transpose() * uk;
          JacT.block<6, 1>(6 * i, 0) += coe * jjt;

          const Eigen::Matrix3d& HRt =
              2.0 / NN * (1.0 - ni / NN) * viRiTukukT[i];
          Eigen::Matrix<double, 6, 6> Hb = Auk[i].transpose() * umumT * Auk[i];
          Hb.block<3, 3>(0, 0) +=
              2.0 / NN * (combo1 - RiTukhat * Pi) * RiTukhat -
              2.0 / NN / NN * viRiTuk[i] * viRiTuk[i].transpose() -
              0.5 * hat(jjt.block<3, 1>(0, 0));
          Hb.block<3, 3>(0, 3) += HRt;
          Hb.block<3, 3>(3, 0) += HRt.transpose();
          Hb.block<3, 3>(3, 3) += 2.0 / NN * (ni - ni * ni / NN) * ukukT;

          Hess.block<6, 6>(6 * i, 6 * i) += coe * Hb;
        }

      for (int i = 0; i < win_size - 1; i++)
        // for(int i=1; i<win_size-1; i++)
        if (sig_orig[i].N != 0) {
          double ni = sig_orig[i].N;
          for (int j = i + 1; j < win_size; j++)
            if (sig_orig[j].N != 0) {
              double nj = sig_orig[j].N;
              Eigen::Matrix<double, 6, 6> Hb =
                  Auk[i].transpose() * umumT * Auk[j];
              Hb.block<3, 3>(0, 0) +=
                  -2.0 / NN / NN * viRiTuk[i] * viRiTuk[j].transpose();
              Hb.block<3, 3>(0, 3) += -2.0 * nj / NN / NN * viRiTukukT[i];
              Hb.block<3, 3>(3, 0) +=
                  -2.0 * ni / NN / NN * viRiTukukT[j].transpose();
              Hb.block<3, 3>(3, 3) += -2.0 * ni * nj / NN / NN * ukukT;

              Hess.block<6, 6>(6 * i, 6 * j) += coe * Hb;
            }
        }

      residual += coe * lmbd[kk];
    }

    for (int i = 1; i < win_size; i++)
      for (int j = 0; j < i; j++)
        Hess.block<6, 6>(6 * i, 6 * j) =
            Hess.block<6, 6>(6 * j, 6 * i).transpose();
  }

  void evaluate_only_residual(const std::vector<ImuState>& xs, int head,
                              int end, double& residual) {
    residual = 0;
    // std::vector<PointCluster> sig_tran(win_size);
    int kk = 0;  // The kk-th lambda value

    // int gps_size = plvec_voxels.size();
    PointCluster pcr;

    for (int a = head; a < end; a++) {
      const std::vector<PointCluster>& sig_orig = plvec_voxels[a];
      PointCluster sig = sig_vecs[a];

      for (int i = 0; i < win_size; i++)
        if (sig_orig[i].N != 0) {
          pcr.transform(sig_orig[i], xs[i]);
          sig += pcr;
        }

      Eigen::Vector3d vBar = sig.v / sig.N;
      // Eigen::Matrix3d cmt = sig.P/sig.N - vBar * vBar.transpose();
      // Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(sig.P - sig.v *
      // vBar.transpose());
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(
          sig.P / sig.N - vBar * vBar.transpose());
      Eigen::Vector3d lmbd = saes.eigenvalues();

      // centers[a] = vBar;
      eig_values[a] = saes.eigenvalues();
      eig_vectors[a] = saes.eigenvectors();
      pcr_adds[a] = sig;
      // Ns[a] = sig.N;

      residual += coeffs[a] * lmbd[kk];
    }
  }

  void clear() {
    sig_vecs.clear();
    plvec_voxels.clear();
    eig_values.clear();
    eig_vectors.clear();
    pcr_adds.clear();
    coeffs.clear();
  }

  ~LidarFactor() {}
};

// The LM optimizer for LiDAR BA
class LidarBaOptimizer {
 public:
  int win_size, jac_leng, thd_num = 2;

  double divide_thread(std::vector<ImuState>& x_stats, LidarFactor& voxhess,
                       Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT) {
    // int thd_num = 4;
    double residual = 0;
    Hess.setZero();
    JacT.setZero();
    std::vector<Eigen::MatrixXd> hessians(thd_num);
    std::vector<Eigen::VectorXd> jacobins(thd_num);

    for (int i = 0; i < thd_num; i++) {
      hessians[i].resize(jac_leng, jac_leng);
      jacobins[i].resize(jac_leng);
    }

    int tthd_num = thd_num;
    std::vector<double> resis(tthd_num, 0);
    int g_size = voxhess.plvec_voxels.size();
    if (g_size < tthd_num) tthd_num = 1;

    std::vector<std::thread*> mthreads(tthd_num);
    double part = 1.0 * g_size / tthd_num;
    // for(int i=0; i<tthd_num; i++)
    for (int i = 1; i < tthd_num; i++)
      mthreads[i] =
          new std::thread(&LidarFactor::acc_evaluate2, &voxhess, x_stats,
                          part * i, part * (i + 1), std::ref(hessians[i]),
                          std::ref(jacobins[i]), std::ref(resis[i]));

    for (int i = 0; i < tthd_num; i++) {
      if (i != 0)
        mthreads[i]->join();
      else
        voxhess.acc_evaluate2(x_stats, 0, part, hessians[0], jacobins[0],
                              resis[0]);
      Hess += hessians[i];
      JacT += jacobins[i];
      residual += resis[i];
      delete mthreads[i];
    }

    return residual;
  }

  double only_residual(std::vector<ImuState>& x_stats, LidarFactor& voxhess) {
    double residual1 = 0;
    // voxhess.evaluate_only_residual(x_stats, 0, voxhess.plvec_voxels.size(),
    // residual1);

    // int thd_num = 2;
    std::vector<double> residuals(thd_num, 0);
    int g_size = voxhess.plvec_voxels.size();
    if (g_size < thd_num) {
      printf("Too Less Voxel\n");
      thd_num = 1;
    }
    std::vector<std::thread*> mthreads(thd_num, nullptr);
    double part = 1.0 * g_size / thd_num;
    for (int i = 1; i < thd_num; i++)
      mthreads[i] = new std::thread(&LidarFactor::evaluate_only_residual,
                                    &voxhess, x_stats, part * i, part * (i + 1),
                                    std::ref(residuals[i]));

    for (int i = 0; i < thd_num; i++) {
      if (i != 0)
        mthreads[i]->join();
      else
        voxhess.evaluate_only_residual(x_stats, part * i, part * (i + 1),
                                       residuals[i]);
      residual1 += residuals[i];
      delete mthreads[i];
    }

    return residual1;
  }

  bool DampingIter(std::vector<ImuState>& x_stats, LidarFactor& voxhess,
                   Eigen::MatrixXd* hess, std::vector<double>& resis,
                   int max_iter = 3, bool is_display = false) {
    win_size = voxhess.win_size;
    jac_leng = win_size * 6;

    double u = 0.01, v = 2;
    Eigen::MatrixXd D(jac_leng, jac_leng), Hess(jac_leng, jac_leng);
    Eigen::VectorXd JacT(jac_leng), dxi(jac_leng);
    hess->resize(jac_leng, jac_leng);

    D.setIdentity();
    double residual1, residual2, q;
    bool is_calc_hess = true;
    std::vector<ImuState> x_stats_temp = x_stats;

    bool is_converge = true;

    // double tt1 =
    // std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    // for(int i=0; i<10; i++)
    for (int i = 0; i < max_iter; i++) {
      if (is_calc_hess) {
        residual1 = divide_thread(x_stats, voxhess, Hess, JacT);
        *hess = Hess;
      }

      if (i == 0) resis.push_back(residual1);

      Hess.topRows(6).setZero();
      Hess.leftCols(6).setZero();
      Hess.block<6, 6>(0, 0).setIdentity();
      JacT.head(6).setZero();

      D.diagonal() = Hess.diagonal();
      dxi = (Hess + u * D).ldlt().solve(-JacT);

      for (int j = 0; j < win_size; j++) {
        x_stats_temp[j].R = x_stats[j].R * Exp(dxi.block<3, 1>(6 * j, 0));
        x_stats_temp[j].p = x_stats[j].p + dxi.block<3, 1>(6 * j + 3, 0);
      }
      double q1 = 0.5 * dxi.dot(u * D * dxi - JacT);

      residual2 = only_residual(x_stats_temp, voxhess);

      q = (residual1 - residual2);
      if (is_display)
        printf("iter%d: (%lf %lf) u: %lf v: %.1lf q: %.2lf %lf %lf\n", i,
               residual1, residual2, u, v, q / q1, q1, q);

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
        is_converge = false;
      }

      if (fabs((residual1 - residual2) / residual1) < 1e-6) break;
    }
    resis.push_back(residual2);
    return is_converge;
  }
};

}  // namespace vxslam
}  // namespace atlas

#endif
