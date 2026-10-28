/*
 * Copyright 2026 The Openbot Authors
 */

#include "frontend/lidar/odometry/normal_icp.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <vector>

#include "common/math/kdtree.hpp"
#include "common/math/so3.hpp"
#include "glog/logging.h"

namespace argus {
namespace vxslam {

bool NormalIcp(const sensor::PointCloudXYZI& pl_src,
               const sensor::PointCloudXYZI& pl_tar,
               std::pair<Eigen::Vector3d, Eigen::Matrix3d>* pose,
               double icp_eigval) {
  if (pose == nullptr || pl_src.empty() || pl_tar.empty()) {
    return false;
  }
  std::vector<Eigen::Vector3d> tar_pts;
  tar_pts.reserve(pl_tar.size());
  for (const auto& ap : pl_tar.points) {
    tar_pts.emplace_back(ap.x, ap.y, ap.z);
  }
  math::KdTree kd_tree;
  kd_tree.SetInput(tar_pts);

  Eigen::Vector4d paras(0.2, 0.2, 0.5, 3.0);
  int is_converge = 0;
  Eigen::Matrix3d mat_norm = Eigen::Matrix3d::Zero();
  const int ssize = static_cast<int>(pl_src.size());

  for (int iterCount = 0; iterCount < 20; ++iterCount) {
    Eigen::Matrix<double, 6, 6> Hess = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> JacT = Eigen::Matrix<double, 6, 1>::Zero();
    mat_norm.setZero();
    int match_num = 0;
    std::vector<int> idx(1);
    std::vector<float> dist2(1);

    for (int i = 0; i < ssize; ++i) {
      const sensor::PointType& searchPoint =
          pl_src.points[static_cast<size_t>(i)];
      const Eigen::Vector3d plocal(searchPoint.x, searchPoint.y, searchPoint.z);
      const Eigen::Vector3d pi = pose->second * plocal + pose->first;
      Eigen::Vector3d ni(searchPoint.normal_x, searchPoint.normal_y,
                         searchPoint.normal_z);
      ni = pose->second * ni;
      if (kd_tree.NearestKSearch(pi, 1, &idx, &dist2) <= 0) {
        continue;
      }
      const sensor::PointType& nearstPoint =
          pl_tar.points[static_cast<size_t>(idx[0])];
      const Eigen::Vector3d tpi(nearstPoint.x, nearstPoint.y, nearstPoint.z);
      const Eigen::Vector3d tni(nearstPoint.normal_x, nearstPoint.normal_y,
                                nearstPoint.normal_z);
      const Eigen::Vector3d normal_inc = ni - tni;
      const Eigen::Vector3d normal_add = ni + tni;
      const double point_to_point_dis = (pi - tpi).norm();
      const double point_to_plane = std::fabs(tni.dot(pi - tpi));
      if ((normal_inc.norm() < paras[0] || normal_add.norm() < paras[1]) &&
          point_to_plane < paras[2] && point_to_point_dis < paras[3]) {
        const double rr = tni.dot(pi - tpi);
        Eigen::Matrix<double, 6, 1> jac;
        jac.head(3) =
            math::SkewSymmetric(plocal) * pose->second.transpose() * tni;
        jac.tail(3) = tni;
        Hess += jac * jac.transpose();
        JacT += jac * rr;
        ++match_num;
        mat_norm += tni * tni.transpose();
      }
    }
    if (match_num < 10) {
      return false;
    }
    const Eigen::Matrix<double, 6, 1> dxi = Hess.ldlt().solve(-JacT);
    pose->second = pose->second * math::Exp(Eigen::Vector3d(dxi.head(3)));
    pose->first = pose->first + dxi.tail(3);
    if (dxi.head(3).norm() < 1e-3 && dxi.tail(3).norm() < 1e-3) {
      if (is_converge) {
        break;
      }
      paras << 0.1, 0.1, 0.1, 1.0;
      is_converge = 1;
    }
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(mat_norm);
  const Eigen::Vector3d eig_vec = saes.eigenvalues();
  VLOG(1) << "normal_icp eig: " << eig_vec.transpose()
          << " converge=" << is_converge;
  return eig_vec[0] > icp_eigval && is_converge == 1;
}

}  // namespace vxslam
}  // namespace argus
