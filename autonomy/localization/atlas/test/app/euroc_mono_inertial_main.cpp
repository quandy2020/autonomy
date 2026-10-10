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
 * --mode=mono_inertial`). This file keeps the original CLI flag surface, loads
 * `mav0`, runs `SlamSystem`, and writes a TUM trajectory.
 */

#include "autonomy/localization/atlas/system/slam_system.hpp"

#include "dataset_io.hpp"

#include <chrono>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include <CLI/CLI.hpp>
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

#include "autolink/common/log.hpp"
#include "autonomy/common/cli_options.hpp"
#include "glog/logging.h"

int main(int argc, char** argv) {
    std::string dataset;
    std::string config = "autonomy/localization/atlas/config/euroc_mono_inertial.yaml";
    std::string traj_out = "keyframe_trajectory_euroc.txt";
    std::string frame_traj_out;
    int max_frames = -1;
    int settle_ms = 2000;

    CLI::App app{"Legacy EuRoC mono-inertial Atlas evaluation."};
    app.add_option("--dataset", dataset, "EuRoC sequence root (contains mav0/).");
    app.add_option("--config", config, "Atlas platform YAML.")
        ->capture_default_str();
    app.add_option("--traj_out", traj_out, "Keyframe trajectory (TUM, Twb).")
        ->capture_default_str();
    app.add_option("--frame_traj_out", frame_traj_out,
                   "Optional frame trajectory (TUM). Empty = skip.")
        ->capture_default_str();
    app.add_option("--max_frames", max_frames, "Stop after N camera frames (-1 = all).")
        ->capture_default_str();
    app.add_option("--settle_ms", settle_ms, "Wait after last frame for mapping (ms).")
        ->capture_default_str();

    autonomy::common::ParseOrExit(app, argc, argv);
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;

    if (dataset.empty()) {
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

    AtlasConfig atlas_config;
    if (!LoadConfig(config, &atlas_config)) {
        AERROR << "failed to load config: " << config;
        return 1;
    }
    YAML::Node root = YAML::LoadFile(config);
    Calib imu_calib;
    if (!LoadImuCalibFromYaml(root, &atlas_config, &imu_calib)) {
        return 1;
    }

    const std::string mav0 = dataset + "/mav0";
    std::vector<ImuSample> imu;
    std::vector<CamSample> cams;
    if (!LoadEurocImuCsv(mav0 + "/imu0/data.csv", &imu) ||
        !LoadEurocCamCsv(mav0 + "/cam0/data.csv", mav0 + "/cam0/data", &cams)) {
        return 1;
    }

    SlamSystem slam;
    if (!slam.Init(atlas_config, SlamSystem::Sensor::kImuMonocular)) {
        return 1;
    }
    slam.SetImuCalib(imu_calib);

    std::ofstream frame_ofs;
    if (!frame_traj_out.empty()) {
        frame_ofs.open(frame_traj_out);
    }

    std::size_t imu_idx = 0;
    int fed = 0;
    for (const auto& cam : cams) {
        if (max_frames >= 0 && fed >= max_frames) {
            break;
        }
        while (imu_idx < imu.size() && imu[imu_idx].t <= cam.t) {
            const auto& m = imu[imu_idx++];
            slam.GrabImuData(Measurement(m.ax, m.ay, m.az, m.wx, m.wy, m.wz, m.t));
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
                WriteTumPose(frame_ofs, static_cast<double>(odom.timestamp) * 1e-9, odom.pose_world_body);
            }
        }
    }
    if (settle_ms > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(settle_ms));
    }
    SaveKeyframeTrajectoryTum(slam.mutable_map(), traj_out, true);
    slam.Shutdown();
    return 0;
}
