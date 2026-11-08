/*
 * Copyright 2026 The Openbot Authors
 */

#include <iostream>
#include <vector>

#include "argus/system/voxel/odometry/imu_preintegration.hpp"
#include "argus/system/voxel/local_mapping/sliding_octree.hpp"
#include "argus/system/voxel/local_mapping/lidar_factor.hpp"
#include "argus/system/voxel/local_mapping/li_ba_optimizer.hpp"

namespace {

int Fail(const char* msg) {
    std::cerr << "FAIL: " << msg << std::endl;
    return 1;
}

}  // namespace

int main() {
    using argus::vxslam::ImuPreintegration;
    using argus::vxslam::ImuState;
    using argus::vxslam::LidarFactor;
    using argus::vxslam::LiBaOptimizer;
    using argus::vxslam::PointCluster;

    const int win = 3;
    LidarFactor factor(win);
    // Synthetic voxel: one plane cluster across window.
    std::vector<PointCluster> orig(static_cast<size_t>(win));
    PointCluster fix;
    PointCluster add;
    for (int i = 0; i < win; ++i) {
        for (int k = 0; k < 30; ++k) {
            orig[static_cast<size_t>(i)].push(
                Eigen::Vector3d(0.1 * k, 0.05 * (k % 5), 0.0));
        }
        add += orig[static_cast<size_t>(i)];
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(add.cov());
    Eigen::Vector3d ev = saes.eigenvalues();
    Eigen::Matrix3d evec = saes.eigenvectors();
    double coe = 1.0;
    factor.push_voxel(orig, fix, coe, ev, evec, add);

    std::vector<ImuState> xs(static_cast<size_t>(win));
    std::deque<ImuPreintegration*> imus;
    for (int i = 0; i < win - 1; ++i) {
        imus.push_back(new ImuPreintegration());
        // Tiny dt so IMU residual is defined.
        Eigen::Vector3d gyr = Eigen::Vector3d::Zero();
        Eigen::Vector3d acc(0, 0, 9.8);
        imus.back()->AddImu(gyr, acc, 0.1);
        // Seed cov to be invertible.
        imus.back()->cov.setIdentity();
        imus.back()->cov *= 1e-3;
    }
    argus::vxslam::imu_coef = 1e-6;
    LiBaOptimizer opt;
    Eigen::MatrixXd hess;
    opt.DampingIter(xs, factor, imus, &hess);
    for (auto* p : imus) {
        delete p;
    }
    if (hess.rows() < win) {
        return Fail("hess");
    }
    std::cout << "PASS test_li_ba" << std::endl;
    return 0;
}
