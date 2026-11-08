/*
 * Copyright 2026 The Openbot Authors
 *
 * Offline Voxel-SLAM pipeline (Argus, no ROS).
 *
 * Usage:
 *   offline_voxel_slam --config config/voxel_slam.yaml \
 *     --lidar_dir <pcd_or_bin_dir> --imu_csv <imu.csv> --out_tum out.tum
 *
 * Minimal smoke: --synth N  (synthetic IMU + wall plane)
 */

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include "argus/common/logger.hpp"
#include "argus/system/voxel/serialization/voxel_map_serializer.hpp"
#include "argus/system/voxel/configuration_files/load_config.hpp"
#include "argus/system/voxel/odometry/odometry.hpp"
#include "glog/logging.h"

namespace {

void PrintUsage() {
    std::cerr
        << "offline_voxel_slam --config <yaml> [--synth N] [--out_tum path]\n";
}

void RunSynth(argus::vxslam::Odometry* odom, int n,
              const std::string& out_tum) {
    std::vector<argus::vxslam::ImuState> traj;
    double t = 0.0;
    for (int i = 0; i < n; ++i) {
        // IMU burst
        for (int k = 0; k < 10; ++k) {
            argus::sensor::ImuData imu;
            imu.timestamp = t + 0.01 * k;
            imu.linear_acceleration = Eigen::Vector3d(0, 0, 9.8);
            imu.angular_velocity = Eigen::Vector3d::Zero();
            odom->AddImu(imu);
        }
        auto cloud = std::make_shared<argus::sensor::PointCloudXYZI>();
        for (int u = 0; u < 40; ++u) {
            for (int v = 0; v < 40; ++v) {
                argus::sensor::PointType p;
                p.x = 0.1f * u;
                p.y = 0.1f * v;
                p.z = 0.f;
                p.curvature = static_cast<float>(0.1 * u / 40.0);
                cloud->push_back(p);
            }
        }
        odom->AddLidar(cloud, t + 0.1);
        argus::odometry::OdometryOutput out;
        while (odom->Process(&out)) {
            if (out.status == argus::odometry::ProcessStatus::kOk) {
                traj.push_back(odom->x_curr());
            }
            if (out.status == argus::odometry::ProcessStatus::kNoPackage) {
                break;
            }
        }
        t += 0.1;
    }
    if (!out_tum.empty()) {
        argus::vxslam::SaveTrajectoryTum(out_tum, traj);
        LOG(INFO) << "Wrote " << out_tum << " poses=" << traj.size();
    }
}

}  // namespace

int main(int argc, char** argv) {
    argus::common::InitLogging(argv[0]);
    std::string config_path = "config/voxel_slam.yaml";
    std::string out_tum = "voxel_slam_est.tum";
    int synth = 0;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        } else if (a == "--synth" && i + 1 < argc) {
            synth = std::atoi(argv[++i]);
        } else if (a == "--out_tum" && i + 1 < argc) {
            out_tum = argv[++i];
        } else if (a == "-h" || a == "--help") {
            PrintUsage();
            return 0;
        }
    }

    argus::vxslam::OdometryConfig cfg;
    if (!argus::vxslam::LoadConfig(config_path, &cfg)) {
        LOG(WARNING) << "Using default OdometryConfig";
    }
    argus::vxslam::Odometry odom(cfg);
    odom.Initialize();

    if (synth > 0) {
        RunSynth(&odom, synth, out_tum);
        odom.RequestFinishGba();
        return 0;
    }
    PrintUsage();
    LOG(ERROR) << "Provide --synth N for smoke test (dataset replay TBD).";
    return 1;
}
