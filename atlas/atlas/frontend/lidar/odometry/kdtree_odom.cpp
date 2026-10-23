/*
 * Copyright 2026 The Openbot Authors
 */
#include "frontend/lidar/odometry/kdtree_odom.hpp"

#include "common/math/so3.hpp"
#include "frontend/lidar/odometry/downsample.hpp"

namespace atlas {
namespace vxslam {

void KdTreeOdom::Clear() {
  map_pts_.clear();
  map_cloud_.clear();
}

bool KdTreeOdom::SeedOrReady(const PointVarList& pvec, const ImuState& x) {
  if (map_pts_.size() < 100) {
    for (const PointVar& pv : pvec) {
      const Eigen::Vector3d w = x.R * pv.pnt + x.p;
      map_pts_.push_back(w);
      sensor::PointType pp;
      pp.x = static_cast<float>(w[0]);
      pp.y = static_cast<float>(w[1]);
      pp.z = static_cast<float>(w[2]);
      map_cloud_.push_back(pp);
    }
    kd_.SetInput(map_pts_);
    return false;
  }
  return true;
}

bool KdTreeOdom::Estimate(PointVarListPtr pvec, ImuState* x) {
  if (!pvec || x == nullptr || map_pts_.size() < 100) {
    return false;
  }
  const ImuState x_prop = *x;
  constexpr int kMaxIter = 4;
  Eigen::Matrix<double, kStateDim, kStateDim> G =
      Eigen::Matrix<double, kStateDim, kStateDim>::Zero();
  Eigen::Matrix<double, kStateDim, kStateDim> H_T_H =
      Eigen::Matrix<double, kStateDim, kStateDim>::Zero();
  const Eigen::Matrix<double, kStateDim, kStateDim> I_STATE =
      Eigen::Matrix<double, kStateDim, kStateDim>::Identity();
  const Eigen::Matrix<double, kStateDim, kStateDim> cov_inv = x->cov.inverse();

  Eigen::Matrix<double, kMatch, 1> b =
      Eigen::Matrix<double, kMatch, 1>::Ones() * -1.0;
  const int psize = static_cast<int>(pvec->size());
  std::vector<double> ds(static_cast<size_t>(psize), -1.0);
  VectorList<3> directs(static_cast<size_t>(psize));
  bool refind = true;
  int rematch_num = 0;
  bool flg_converged = false;

  for (int iterCount = 0; iterCount < kMaxIter; ++iterCount) {
    Eigen::Matrix<double, 6, 6> HTH = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> HTz = Eigen::Matrix<double, 6, 1>::Zero();
    int valid = 0;
    for (int i = 0; i < psize; ++i) {
      PointVar& pv = (*pvec)[static_cast<size_t>(i)];
      const Eigen::Matrix3d phat = math::SkewSymmetric(pv.pnt);
      const Eigen::Vector3d wld = x->R * pv.pnt + x->p;
      if (refind) {
        std::vector<int> nearInd;
        std::vector<float> sqdis;
        if (kd_.NearestKSearch(wld, kMatch, &nearInd, &sqdis) < kMatch) {
          ds[static_cast<size_t>(i)] = -1;
          continue;
        }
        Eigen::Matrix<double, kMatch, 3> A;
        for (int k = 0; k < kMatch; ++k) {
          const Eigen::Vector3d& pp =
              map_pts_[static_cast<size_t>(nearInd[static_cast<size_t>(k)])];
          A.row(k) << pp.x(), pp.y(), pp.z();
        }
        Eigen::Vector3d direct = A.colPivHouseholderQr().solve(b);
        bool check_flag = false;
        for (int k = 0; k < kMatch; ++k) {
          if (std::fabs(direct.dot(A.row(k)) + 1.0) > 0.1) {
            check_flag = true;
          }
        }
        if (check_flag) {
          ds[static_cast<size_t>(i)] = -1;
          continue;
        }
        const double d = 1.0 / direct.norm();
        ds[static_cast<size_t>(i)] = d;
        directs[static_cast<size_t>(i)] = direct * d;
      }
      if (ds[static_cast<size_t>(i)] >= 0) {
        const double pd2 = directs[static_cast<size_t>(i)].dot(wld) +
                           ds[static_cast<size_t>(i)];
        Eigen::Matrix<double, 6, 1> jac_s;
        jac_s.head(3) =
            phat * x->R.transpose() * directs[static_cast<size_t>(i)];
        jac_s.tail(3) = directs[static_cast<size_t>(i)];
        HTH += jac_s * jac_s.transpose();
        HTz += jac_s * (-pd2);
        ++valid;
      }
    }
    if (valid < 20) {
      return false;
    }
    H_T_H.block<6, 6>(0, 0) = HTH;
    const Eigen::Matrix<double, kStateDim, kStateDim> K_1 =
        (H_T_H + cov_inv / 1000.0).inverse();
    G.block<kStateDim, 6>(0, 0) = K_1.block<kStateDim, 6>(0, 0) * HTH;
    const Eigen::Matrix<double, kStateDim, 1> vec = x_prop - *x;
    Eigen::Matrix<double, kStateDim, 1> solution =
        K_1.block<kStateDim, 6>(0, 0) * HTz + vec -
        G.block<kStateDim, 6>(0, 0) * vec.block<6, 1>(0, 0);
    *x += solution;
    const Eigen::Vector3d rot_add = solution.block<3, 1>(0, 0);
    const Eigen::Vector3d tra_add = solution.block<3, 1>(3, 0);
    refind = false;
    if ((rot_add.norm() * 57.3 < 0.01) && (tra_add.norm() * 100 < 0.015)) {
      refind = true;
      flg_converged = true;
      ++rematch_num;
    }
    if (iterCount == kMaxIter - 2 && !flg_converged) {
      refind = true;
    }
    if (rematch_num >= 2 || iterCount == kMaxIter - 1) {
      x->cov = (I_STATE - G) * x->cov;
      break;
    }
  }

  for (const PointVar& pv : *pvec) {
    const Eigen::Vector3d w = x->R * pv.pnt + x->p;
    map_pts_.push_back(w);
    sensor::PointType ap;
    ap.x = static_cast<float>(w[0]);
    ap.y = static_cast<float>(w[1]);
    ap.z = static_cast<float>(w[2]);
    map_cloud_.push_back(ap);
  }
  DownsampleVoxel(&map_cloud_, 0.5);
  map_pts_.clear();
  map_pts_.reserve(map_cloud_.size());
  for (const auto& p : map_cloud_.points) {
    map_pts_.emplace_back(p.x, p.y, p.z);
  }
  kd_.SetInput(map_pts_);
  return true;
}

}  // namespace vxslam
}  // namespace atlas
