/*
 * Copyright 2026 The Openbot Authors
 */
#include "frontend/lidar/odometry/point_cov.hpp"

#include "common/math/so3.hpp"

namespace atlas {
namespace vxslam {

void CalcBodyVar(Eigen::Vector3d pb, float range_inc, float degree_inc,
                 Eigen::Matrix3d* var) {
  if (var == nullptr) {
    return;
  }
  if (pb[2] == 0.0) {
    pb[2] = 0.0001;
  }
  const float range = std::sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]);
  const float range_var = range_inc * range_inc;
  Eigen::Matrix2d direction_var;
  const double s = std::sin(degree_inc * M_PI / 180.0);
  direction_var << s * s, 0, 0, s * s;
  Eigen::Vector3d direction(pb);
  direction.normalize();
  const Eigen::Matrix3d direction_hat = math::SkewSymmetric(direction);
  Eigen::Vector3d base_vector1(1, 1,
                               -(direction(0) + direction(1)) / direction(2));
  base_vector1.normalize();
  Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
  base_vector2.normalize();
  Eigen::Matrix<double, 3, 2> N;
  N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1),
      base_vector1(2), base_vector2(2);
  const Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
  *var = direction * range_var * direction.transpose() +
         A * direction_var * A.transpose();
}

void InitPointVars(const ImuState& extrin, const sensor::PointCloudXYZI& cloud,
                   double dept_err, double beam_err, PointVarList* out) {
  if (out == nullptr) {
    return;
  }
  out->clear();
  out->resize(cloud.size());
  for (size_t i = 0; i < cloud.size(); ++i) {
    const sensor::PointType& ap = cloud.points[i];
    PointVar& pv = (*out)[i];
    pv.pnt << ap.x, ap.y, ap.z;
    CalcBodyVar(pv.pnt, static_cast<float>(dept_err),
                static_cast<float>(beam_err), &pv.var);
    pv.pnt = extrin.R * pv.pnt + extrin.p;
    pv.var = extrin.R * pv.var * extrin.R.transpose();
  }
}

void UpdateWorldVars(const ImuState& x, PointVarList* pvec,
                     VectorList<3>* pwld) {
  if (pvec == nullptr) {
    return;
  }
  const Eigen::Matrix3d rot_var = x.cov.block<3, 3>(0, 0);
  const Eigen::Matrix3d tsl_var = x.cov.block<3, 3>(3, 3);
  if (pwld != nullptr) {
    pwld->clear();
    pwld->reserve(pvec->size());
  }
  for (PointVar& pv : *pvec) {
    const Eigen::Matrix3d phat = math::SkewSymmetric(pv.pnt);
    pv.var = x.R * pv.var * x.R.transpose() +
             phat * rot_var * phat.transpose() + tsl_var;
    if (pwld != nullptr) {
      pwld->push_back(x.R * pv.pnt + x.p);
    }
  }
}

}  // namespace vxslam
}  // namespace atlas
