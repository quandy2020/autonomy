/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_MAP_POINT_CLOUD_FACTOR_HPP_
#define ATLAS_MAP_POINT_CLOUD_FACTOR_HPP_

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <vector>

#include "common/math/so3.hpp"
#include "common/pose_state.hpp"
#include "map/point_cloud_factor_data.hpp"

namespace atlas {
namespace map {

using ::atlas::common::PoseState;

inline Eigen::Matrix3d Skew(const Eigen::Vector3d& vector) {
  return math::SkewSymmetric(vector);
}

// The LiDAR BA factor in optimization
class PointCloudFactor : public PointCloudFactorData {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit PointCloudFactor(int window_count)
      : PointCloudFactorData(window_count) {}

  void AccumulateEvaluation(const std::vector<PoseState>& pose_window,
                            int first_cluster_index, int cluster_end_index,
                            Eigen::MatrixXd& hessian, Eigen::VectorXd& jacobian,
                            double& residual) {
    hessian.setZero();
    jacobian.setZero();
    residual = 0;
    std::vector<PointCluster> sig_tran(static_cast<std::size_t>(window_count));
    const int kk = 0;

    AlignedVectorList<3> viRiTuk(window_count);
    AlignedMatrixList<3> viRiTukukT(window_count);

    std::vector<Eigen::Matrix<double, 3, 6>,
                Eigen::aligned_allocator<Eigen::Matrix<double, 3, 6>>>
        Auk(window_count);
    Eigen::Matrix3d umumT;

    for (int cluster_index = first_cluster_index;
         cluster_index < cluster_end_index; cluster_index++) {
      std::vector<PointCluster>& sig_orig =
          point_clusters_by_window[cluster_index];
      double coe = coefficients[cluster_index];

      // PointCluster sig = fixed_point_clusters[cluster_index];
      // for(int i=0; i<window_count; i++)
      // if(sig_orig[i].N != 0)
      // {
      //   sig_tran[i].transform(sig_orig[i], state_window[i]);
      //   sig += sig_tran[i];
      // }

      // const Eigen::Vector3d &vBar = sig.v / sig.N;
      // Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(sig.P/sig.N - vBar
      // * vBar.transpose()); const Eigen::Vector3d &lmbd = saes.eigenvalues();
      // const Eigen::Matrix3d &U = saes.eigenvectors();
      // int NN = sig.N;

      Eigen::Vector3d lmbd = eigenvalues[cluster_index];
      Eigen::Matrix3d U = eigenvectors[cluster_index];
      int NN = accumulated_point_clusters[cluster_index].N;
      Eigen::Vector3d vBar = accumulated_point_clusters[cluster_index].v / NN;

      Eigen::Vector3d u[3] = {U.col(0), U.col(1), U.col(2)};
      Eigen::Vector3d& uk = u[kk];
      Eigen::Matrix3d ukukT = uk * uk.transpose();
      umumT.setZero();
      for (int i = 0; i < 3; i++)
        if (i != kk)
          umumT += 2.0 / (lmbd[kk] - lmbd[i]) * u[i] * u[i].transpose();

      for (int i = 0; i < window_count; i++)
        // for(int i=1; i<window_count; i++)
        if (sig_orig[i].N != 0) {
          Eigen::Matrix3d Pi = sig_orig[i].P;
          Eigen::Vector3d vi = sig_orig[i].v;
          Eigen::Matrix3d Ri = pose_window[i].rotation;
          double ni = sig_orig[i].N;

          Eigen::Matrix3d vihat;
          vihat << math::SkewSymmetric(vi);
          Eigen::Vector3d RiTuk = Ri.transpose() * uk;
          Eigen::Matrix3d RiTukhat;
          RiTukhat << math::SkewSymmetric(RiTuk);

          Eigen::Vector3d PiRiTuk = Pi * RiTuk;
          viRiTuk[i] = vihat * RiTuk;
          viRiTukukT[i] = viRiTuk[i] * uk.transpose();

          Eigen::Vector3d ti_v = pose_window[i].translation - vBar;
          double ukTti_v = uk.dot(ti_v);

          Eigen::Matrix3d combo1 = Skew(PiRiTuk) + vihat * ukTti_v;
          Eigen::Vector3d combo2 = Ri * vi + ni * ti_v;
          Auk[i].block<3, 3>(0, 0) =
              (Ri * Pi + ti_v * vi.transpose()) * RiTukhat - Ri * combo1;
          Auk[i].block<3, 3>(0, 3) =
              combo2 * uk.transpose() +
              combo2.dot(uk) * Eigen::Matrix3d::Identity();
          Auk[i] /= NN;

          const Eigen::Matrix<double, 6, 1>& jjt = Auk[i].transpose() * uk;
          jacobian.block<6, 1>(6 * i, 0) += coe * jjt;

          const Eigen::Matrix3d& HRt =
              2.0 / NN * (1.0 - ni / NN) * viRiTukukT[i];
          Eigen::Matrix<double, 6, 6> Hb = Auk[i].transpose() * umumT * Auk[i];
          Hb.block<3, 3>(0, 0) +=
              2.0 / NN * (combo1 - RiTukhat * Pi) * RiTukhat -
              2.0 / NN / NN * viRiTuk[i] * viRiTuk[i].transpose() -
              0.5 * Skew(jjt.block<3, 1>(0, 0));
          Hb.block<3, 3>(0, 3) += HRt;
          Hb.block<3, 3>(3, 0) += HRt.transpose();
          Hb.block<3, 3>(3, 3) += 2.0 / NN * (ni - ni * ni / NN) * ukukT;

          hessian.block<6, 6>(6 * i, 6 * i) += coe * Hb;
        }

      for (int i = 0; i < window_count - 1; i++)
        // for(int i=1; i<window_count-1; i++)
        if (sig_orig[i].N != 0) {
          double ni = sig_orig[i].N;
          for (int j = i + 1; j < window_count; j++)
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

              hessian.block<6, 6>(6 * i, 6 * j) += coe * Hb;
            }
        }

      residual += coe * lmbd[kk];
    }

    for (int i = 1; i < window_count; i++)
      for (int j = 0; j < i; j++)
        hessian.block<6, 6>(6 * i, 6 * j) =
            hessian.block<6, 6>(6 * j, 6 * i).transpose();
  }

  void EvaluateResidual(const std::vector<PoseState>& pose_window,
                        int first_cluster_index, int cluster_end_index,
                        double& residual) {
    residual = 0;
    // std::vector<PointCluster> sig_tran(window_count);
    int kk = 0;  // The kk-th lambda value

    // int gps_size = point_clusters_by_window.size();
    PointCluster pcr;

    for (int cluster_index = first_cluster_index;
         cluster_index < cluster_end_index; cluster_index++) {
      const std::vector<PointCluster>& sig_orig =
          point_clusters_by_window[cluster_index];
      PointCluster sig = fixed_point_clusters[cluster_index];

      for (int i = 0; i < window_count; i++)
        if (sig_orig[i].N != 0) {
          pcr.transform(sig_orig[i], pose_window[i].rotation,
                        pose_window[i].translation);
          sig += pcr;
        }

      Eigen::Vector3d vBar = sig.v / sig.N;
      // Eigen::Matrix3d cmt = sig.P/sig.N - vBar * vBar.transpose();
      // Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(sig.P - sig.v *
      // vBar.transpose());
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(
          sig.P / sig.N - vBar * vBar.transpose());
      Eigen::Vector3d lmbd = saes.eigenvalues();

      // centers[cluster_index] = vBar;
      eigenvalues[cluster_index] = saes.eigenvalues();
      eigenvectors[cluster_index] = saes.eigenvectors();
      accumulated_point_clusters[cluster_index] = sig;
      // Ns[cluster_index] = sig.N;

      residual += coefficients[cluster_index] * lmbd[kk];
    }
  }
};

}  // namespace map
}  // namespace atlas

#endif
