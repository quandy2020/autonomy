/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ARGUS_VXSLAM_POINT_CLUSTER_HPP_
#define ARGUS_VXSLAM_POINT_CLUSTER_HPP_

#include <Eigen/Core>

#include "argus/system/voxel/odometry/imu_state.hpp"

namespace argus {
namespace vxslam {

/** @brief First/second-moment accumulator for plane fitting. */
class PointCluster {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix3d P = Eigen::Matrix3d::Zero();
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    int N = 0;

    void Clear() {
        P.setZero();
        v.setZero();
        N = 0;
    }
    void clear() { Clear(); }

    void Push(const Eigen::Vector3d& vec) {
        ++N;
        P += vec * vec.transpose();
        v += vec;
    }
    void push(const Eigen::Vector3d& vec) { Push(vec); }

    Eigen::Matrix3d Cov() const {
        const Eigen::Vector3d c = v / static_cast<double>(N);
        return P / static_cast<double>(N) - c * c.transpose();
    }
    Eigen::Matrix3d cov() { return Cov(); }

    PointCluster& operator+=(const PointCluster& o) {
        P += o.P;
        v += o.v;
        N += o.N;
        return *this;
    }
    PointCluster& operator-=(const PointCluster& o) {
        P -= o.P;
        v -= o.v;
        N -= o.N;
        return *this;
    }

    void Transform(const PointCluster& src, const ImuState& s) {
        N = src.N;
        v = s.R * src.v + static_cast<double>(N) * s.p;
        const Eigen::Matrix3d rp = s.R * src.v * s.p.transpose();
        P = s.R * src.P * s.R.transpose() + rp + rp.transpose() +
            static_cast<double>(N) * s.p * s.p.transpose();
    }
    void transform(const PointCluster& src, const ImuState& s) {
        Transform(src, s);
    }
};

}  // namespace vxslam
}  // namespace argus

#endif
