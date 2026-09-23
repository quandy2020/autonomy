/*
 * Copyright 2026 The Openbot Authors
 *
 * Legacy EuRoC mono-inertial entry. Prefer atlas_dataset:
 *   autonomy.localization.atlas_dataset \
 *     --format=euroc --mode=mono_inertial --dataset=... --config=...
 *
 * This binary keeps the previous flag surface for existing scripts.
 */

/**
 * @file euroc_mono_inertial_main.cpp
 * @brief Legacy EuRoC mono-inertial evaluation entry (compat with old script flags).
 *
 * Prefer `atlas_dataset_main` for new runs (`--format=euroc
 * --mode=mono_inertial`). This file keeps the original gflags surface, loads
 * `mav0`, runs `SlamSystem`, and writes a TUM trajectory.
 */

#include "autonomy/localization/atlas/system/slam_system.hpp"

#include "dataset_io.hpp"

#include <chrono>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "gflags/gflags.h"
#include "glog/logging.h"
#include "yaml-cpp/yaml.h"

#include "autolink/common/log.hpp"

DEFINE_string(dataset, "", "EuRoC sequence root (contains mav0/)");
DEFINE_string(config,
              "autonomy/localization/atlas/config/euroc_mono_inertial.yaml",
              "Atlas platform YAML");
DEFINE_string(traj_out, "keyframe_trajectory_euroc.txt",
              "Keyframe trajectory (TUM, Twb)");
DEFINE_string(frame_traj_out, "",
              "Optional frame trajectory (TUM). Empty = skip.");
DEFINE_int32(max_frames, -1, "Stop after N camera frames (-1 = all)");
DEFINE_int32(settle_ms, 2000, "Wait after last frame for mapping (ms)");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    FLAGS_alsologtostderr = 1;

    if (FLAGS_dataset.empty()) {
        AERROR << "required: --dataset=/path/to/EuRoC/MH_01_easy";
        AERROR << "prefer: autonomy.localization.atlas_dataset "
                  "--format=euroc --mode=mono_inertial ...";
        return 1;
    }

    using autonomy::localization::atlas::AtlasConfig;
    using autonomy::localization::atlas::LoadConfig;
    using autonomy::localization::atlas::OdometryResult;
    using autonomy::localization::atlas::SlamSystem;
    using autonomy::localization::atlas::sensor::imu::Calib;
    using autonomy::localization::atlas::sensor::imu::Measurement;
    using autonomy::localization::atlas::test_app::CamSample;
    using autonomy::localization::atlas::test_app::ImuSample;
    using autonomy::localization::atlas::test_app::LoadEurocCamCsv;
    using autonomy::localization::atlas::test_app::LoadEurocImuCsv;
    using autonomy::localization::atlas::test_app::LoadImuCalibFromYaml;
    using autonomy::localization::atlas::test_app::SaveKeyframeTrajectoryTum;
    using autonomy::localization::atlas::test_app::WriteTumPose;

    AtlasConfig config;
    if (!LoadConfig(FLAGS_config, &config)) {
        AERROR << "failed to load config: " << FLAGS_config;
        return 1;
    }
    YAML::Node root = YAML::LoadFile(FLAGS_config);
    Calib imu_calib;
    if (!LoadImuCalibFromYaml(root, &config, &imu_calib)) {
        return 1;
    }

    const std::string mav0 = FLAGS_dataset + "/mav0";
    std::vector<ImuSample> imu;
    std::vector<CamSample> cams;
    if (!LoadEurocImuCsv(mav0 + "/imu0/data.csv", &imu) ||
        !LoadEurocCamCsv(mav0 + "/cam0/data.csv", mav0 + "/cam0/data",
                         &cams)) {
        return 1;
    }

    SlamSystem slam;
    if (!slam.Init(config, SlamSystem::Sensor::kImuMonocular)) {
        return 1;
    }
    slam.SetImuCalib(imu_calib);

    std::ofstream frame_ofs;
    if (!FLAGS_frame_traj_out.empty()) {
        frame_ofs.open(FLAGS_frame_traj_out);
    }

    std::size_t imu_idx = 0;
    int fed = 0;
    for (const auto& cam : cams) {
        if (FLAGS_max_frames >= 0 && fed >= FLAGS_max_frames) {
            break;
        }
        while (imu_idx < imu.size() && imu[imu_idx].t <= cam.t) {
            const auto& m = imu[imu_idx++];
            slam.GrabImuData(
                Measurement(m.ax, m.ay, m.az, m.wx, m.wy, m.wz, m.t));
        }
        cv::Mat img = cv::imread(cam.path, cv::IMREAD_UNCHANGED);
        if (img.empty()) {
            continue;
        }
        slam.TrackMonocular(img, cam.t);
        ++fed;
        if (frame_ofs.is_open()) {
            OdometryResult odom;
            if (slam.GetOdometry(&odom) && odom.valid) {
                WriteTumPose(frame_ofs,
                             static_cast<double>(odom.timestamp) * 1e-9,
                             odom.pose_world_body);
            }
        }
    }
    if (FLAGS_settle_ms > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(FLAGS_settle_ms));
    }
    SaveKeyframeTrajectoryTum(slam.mutable_map(), FLAGS_traj_out, true);
    slam.Shutdown();
    return 0;
}
