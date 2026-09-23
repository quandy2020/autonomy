/*
 * Copyright 2026 The Openbot Authors
 *
 * Unified offline dataset runner for Atlas SlamSystem.
 *
 * Modes: mono | mono_inertial | stereo | stereo_inertial | rgbd
 * Formats: euroc | tum_rgbd | kitti
 *
 * Examples:
 *   # EuRoC VIO
 *   autonomy.localization.atlas_dataset \
 *     --format=euroc --mode=mono_inertial \
 *     --dataset=/path/MH_01_easy \
 *     --config=.../euroc_mono_inertial.yaml
 *
 *   # EuRoC stereo VO
 *   ... --mode=stereo --config=.../euroc_stereo.yaml
 *
 *   # TUM RGB-D
 *   ... --format=tum_rgbd --mode=rgbd \
 *       --dataset=/path/rgbd_dataset_freiburg1_desk \
 *       --config=.../tum_rgbd1.yaml
 *
 *   # KITTI odometry stereo
 *   ... --format=kitti --mode=stereo \
 *       --dataset=/path/sequences/00 \
 *       --config=.../kitti_stereo_00-02.yaml
 */

/**
 * @file atlas_dataset_main.cpp
 * @brief Unified Atlas offline dataset evaluation entry (EuRoC / TUM RGB-D / KITTI).
 *
 * Selects `--format` and `--mode` via gflags, loads YAML config, drives
 * `SlamSystem`, and writes keyframe TUM trajectories. See `dataset_io.hpp`.
 */

#include "autonomy/localization/atlas/system/slam_system.hpp"
#include "autonomy/localization/atlas/common/config.hpp"

#include "dataset_io.hpp"

#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "gflags/gflags.h"
#include "glog/logging.h"
#include "yaml-cpp/yaml.h"

#include "autolink/autolink.hpp"
#include "autolink/common/log.hpp"
#include "autolink/time/rate.hpp"

DEFINE_string(dataset, "", "Dataset sequence root");
DEFINE_string(format, "euroc", "Dataset format: euroc | tum_rgbd | kitti");
DEFINE_string(mode, "mono_inertial",
              "Sensor mode: mono | mono_inertial | stereo | stereo_inertial | "
              "rgbd");
DEFINE_string(config, "", "Atlas platform YAML (required)");
DEFINE_string(traj_out, "keyframe_trajectory.txt",
              "Keyframe trajectory (TUM)");
DEFINE_string(frame_traj_out, "", "Optional frame trajectory (TUM)");
DEFINE_string(associations, "",
              "TUM associations.txt (optional; else rgb.txt+depth.txt)");
DEFINE_int32(max_frames, -1, "Stop after N frames (-1 = all)");
DEFINE_int32(settle_ms, 2000, "Wait after last frame for mapping (ms)");
DEFINE_double(sync_slop, 0.02, "Stereo / RGB-D timestamp match (s)");
DEFINE_bool(viz, false,
            "Publish Atlas Autolink channels (TF/odom/path/points/...)");
DEFINE_string(viz_node, "atlas_dataset", "Autolink node name when --viz");
DEFINE_bool(viz_hold, false,
            "After run, keep publishing alive until Ctrl+C (needs --viz)");
DEFINE_double(viz_rate, 0.0,
              "Throttle playback when --viz: Hz (0 = as fast as possible)");

namespace {

using autonomy::localization::atlas::AtlasConfig;
using autonomy::localization::atlas::DumpConfig;
using autonomy::localization::atlas::LoadConfig;
using autonomy::localization::atlas::OdometryResult;
using autonomy::localization::atlas::SlamSystem;
using autonomy::localization::atlas::sensor::imu::Calib;
using autonomy::localization::atlas::sensor::imu::Measurement;
using autonomy::localization::atlas::test_app::AssociateStereo;
using autonomy::localization::atlas::test_app::CamSample;
using autonomy::localization::atlas::test_app::ImuSample;
using autonomy::localization::atlas::test_app::LoadEurocCamCsv;
using autonomy::localization::atlas::test_app::LoadEurocImuCsv;
using autonomy::localization::atlas::test_app::LoadImuCalibFromYaml;
using autonomy::localization::atlas::test_app::LoadKittiMono;
using autonomy::localization::atlas::test_app::LoadKittiStereo;
using autonomy::localization::atlas::test_app::LoadTumAssociations;
using autonomy::localization::atlas::test_app::LoadTumRgbDepthTxt;
using autonomy::localization::atlas::test_app::RgbdSample;
using autonomy::localization::atlas::test_app::SaveKeyframeTrajectoryTum;
using autonomy::localization::atlas::test_app::StereoSample;
using autonomy::localization::atlas::test_app::WriteTumPose;

bool ParseSensor(const std::string& mode, SlamSystem::Sensor* sensor,
                 bool* need_imu) {
    *need_imu = false;
    if (mode == "mono" || mode == "vo" || mode == "rgb") {
        *sensor = SlamSystem::Sensor::kMonocular;
    } else if (mode == "mono_inertial" || mode == "vio") {
        *sensor = SlamSystem::Sensor::kImuMonocular;
        *need_imu = true;
    } else if (mode == "stereo") {
        *sensor = SlamSystem::Sensor::kStereo;
    } else if (mode == "stereo_inertial") {
        *sensor = SlamSystem::Sensor::kImuStereo;
        *need_imu = true;
    } else if (mode == "rgbd" || mode == "rgb-d") {
        *sensor = SlamSystem::Sensor::kRgbd;
    } else if (mode == "rgbd_inertial") {
        *sensor = SlamSystem::Sensor::kImuRgbd;
        *need_imu = true;
    } else {
        AERROR << "unknown --mode=" << mode;
        return false;
    }
    return true;
}

void MaybeWriteFrame(std::ofstream& ofs, SlamSystem* slam) {
    if (!ofs.is_open()) {
        return;
    }
    OdometryResult odom;
    if (slam->GetOdometry(&odom) && odom.valid) {
        WriteTumPose(ofs, static_cast<double>(odom.timestamp) * 1e-9,
                     odom.pose_world_body);
    }
}

void MaybeThrottleViz() {
    if (FLAGS_viz_rate <= 1e-3) {
        return;
    }
    static autolink::Rate rate(FLAGS_viz_rate);
    rate.Sleep();
}

void FeedImuUpTo(SlamSystem* slam, const std::vector<ImuSample>& imu,
                 std::size_t* imu_idx, double t) {
    while (*imu_idx < imu.size() && imu[*imu_idx].t <= t) {
        const auto& m = imu[(*imu_idx)++];
        slam->GrabImuData(
            Measurement(m.ax, m.ay, m.az, m.wx, m.wy, m.wz, m.t));
    }
}

int RunEuroc(SlamSystem* slam, bool need_imu, SlamSystem::Sensor sensor,
             std::ofstream* frame_ofs) {
    const std::string mav0 = FLAGS_dataset + "/mav0";
    std::vector<ImuSample> imu;
    if (need_imu && !LoadEurocImuCsv(mav0 + "/imu0/data.csv", &imu)) {
        return 1;
    }

    std::vector<CamSample> cam0;
    if (!LoadEurocCamCsv(mav0 + "/cam0/data.csv", mav0 + "/cam0/data",
                         &cam0)) {
        return 1;
    }

    std::vector<StereoSample> stereo;
    const bool is_stereo = (sensor == SlamSystem::Sensor::kStereo ||
                            sensor == SlamSystem::Sensor::kImuStereo);
    if (is_stereo) {
        std::vector<CamSample> cam1;
        if (!LoadEurocCamCsv(mav0 + "/cam1/data.csv", mav0 + "/cam1/data",
                             &cam1)) {
            return 1;
        }
        if (!AssociateStereo(cam0, cam1, FLAGS_sync_slop, &stereo)) {
            return 1;
        }
    }

    std::size_t imu_idx = 0;
    int fed = 0;
    const auto t0 = std::chrono::steady_clock::now();

    if (is_stereo) {
        for (const auto& s : stereo) {
            if (FLAGS_max_frames >= 0 && fed >= FLAGS_max_frames) {
                break;
            }
            if (need_imu) {
                FeedImuUpTo(slam, imu, &imu_idx, s.t);
            }
            cv::Mat left = cv::imread(s.left, cv::IMREAD_UNCHANGED);
            cv::Mat right = cv::imread(s.right, cv::IMREAD_UNCHANGED);
            if (left.empty() || right.empty()) {
                AWARN << "skip stereo: " << s.left;
                continue;
            }
            slam->TrackStereo(left, right, s.t);
            ++fed;
            MaybeWriteFrame(*frame_ofs, slam);
            MaybeThrottleViz();
            if (fed % 50 == 0) {
                AINFO << "fed " << fed << "/" << stereo.size();
            }
        }
    } else {
        for (const auto& cam : cam0) {
            if (FLAGS_max_frames >= 0 && fed >= FLAGS_max_frames) {
                break;
            }
            if (need_imu) {
                FeedImuUpTo(slam, imu, &imu_idx, cam.t);
            }
            cv::Mat img = cv::imread(cam.path, cv::IMREAD_UNCHANGED);
            if (img.empty()) {
                AWARN << "skip: " << cam.path;
                continue;
            }
            slam->TrackMonocular(img, cam.t);
            ++fed;
            MaybeWriteFrame(*frame_ofs, slam);
            MaybeThrottleViz();
            if (fed % 50 == 0) {
                AINFO << "fed " << fed << "/" << cam0.size();
            }
        }
    }

    if (need_imu) {
        while (imu_idx < imu.size()) {
            const auto& m = imu[imu_idx++];
            slam->GrabImuData(
                Measurement(m.ax, m.ay, m.az, m.wx, m.wy, m.wz, m.t));
        }
    }

    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0)
                        .count();
    AINFO << "euroc done: frames=" << fed << " elapsed_ms=" << ms;
    return 0;
}

int RunTumRgbd(SlamSystem* slam, bool need_imu, std::ofstream* frame_ofs) {
    if (need_imu) {
        AWARN << "tum_rgbd + IMU: no standard IMU; continuing without IMU "
                 "feed";
    }
    std::vector<RgbdSample> pairs;
    if (!FLAGS_associations.empty()) {
        if (!LoadTumAssociations(FLAGS_associations, FLAGS_dataset, &pairs)) {
            return 1;
        }
    } else {
        const std::string assoc = FLAGS_dataset + "/associations.txt";
        std::ifstream probe(assoc);
        if (probe.good()) {
            if (!LoadTumAssociations(assoc, FLAGS_dataset, &pairs)) {
                return 1;
            }
        } else if (!LoadTumRgbDepthTxt(FLAGS_dataset, FLAGS_sync_slop,
                                       &pairs)) {
            return 1;
        }
    }

    int fed = 0;
    const auto t0 = std::chrono::steady_clock::now();
    for (const auto& s : pairs) {
        if (FLAGS_max_frames >= 0 && fed >= FLAGS_max_frames) {
            break;
        }
        cv::Mat rgb = cv::imread(s.rgb, cv::IMREAD_UNCHANGED);
        cv::Mat depth = cv::imread(s.depth, cv::IMREAD_UNCHANGED);
        if (rgb.empty() || depth.empty()) {
            AWARN << "skip RGB-D: " << s.rgb;
            continue;
        }
        slam->TrackRgbd(rgb, depth, s.t);
        ++fed;
        MaybeWriteFrame(*frame_ofs, slam);
        MaybeThrottleViz();
        if (fed % 50 == 0) {
            AINFO << "fed " << fed << "/" << pairs.size();
        }
    }
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0)
                        .count();
    AINFO << "tum_rgbd done: frames=" << fed << " elapsed_ms=" << ms;
    return 0;
}

int RunKitti(SlamSystem* slam, SlamSystem::Sensor sensor,
             std::ofstream* frame_ofs) {
    const bool is_stereo = (sensor == SlamSystem::Sensor::kStereo);
    if (sensor != SlamSystem::Sensor::kMonocular && !is_stereo) {
        AERROR << "kitti supports --mode=mono|stereo only";
        return 1;
    }

    int fed = 0;
    const auto t0 = std::chrono::steady_clock::now();

    if (is_stereo) {
        std::vector<StereoSample> pairs;
        if (!LoadKittiStereo(FLAGS_dataset, &pairs)) {
            return 1;
        }
        for (const auto& s : pairs) {
            if (FLAGS_max_frames >= 0 && fed >= FLAGS_max_frames) {
                break;
            }
            cv::Mat left = cv::imread(s.left, cv::IMREAD_UNCHANGED);
            cv::Mat right = cv::imread(s.right, cv::IMREAD_UNCHANGED);
            if (left.empty() || right.empty()) {
                AWARN << "skip KITTI stereo: " << s.left;
                continue;
            }
            slam->TrackStereo(left, right, s.t);
            ++fed;
            MaybeWriteFrame(*frame_ofs, slam);
            MaybeThrottleViz();
            if (fed % 50 == 0) {
                AINFO << "fed " << fed << "/" << pairs.size();
            }
        }
    } else {
        std::vector<CamSample> cams;
        if (!LoadKittiMono(FLAGS_dataset, &cams)) {
            return 1;
        }
        for (const auto& cam : cams) {
            if (FLAGS_max_frames >= 0 && fed >= FLAGS_max_frames) {
                break;
            }
            cv::Mat img = cv::imread(cam.path, cv::IMREAD_UNCHANGED);
            if (img.empty()) {
                AWARN << "skip: " << cam.path;
                continue;
            }
            slam->TrackMonocular(img, cam.t);
            ++fed;
            MaybeWriteFrame(*frame_ofs, slam);
            MaybeThrottleViz();
            if (fed % 50 == 0) {
                AINFO << "fed " << fed << "/" << cams.size();
            }
        }
    }

    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0)
                        .count();
    AINFO << "kitti done: frames=" << fed << " elapsed_ms=" << ms;
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    FLAGS_alsologtostderr = 1;

    if (FLAGS_dataset.empty() || FLAGS_config.empty()) {
        AERROR << "required: --dataset and --config";
        return 1;
    }

    SlamSystem::Sensor sensor;
    bool need_imu = false;
    if (!ParseSensor(FLAGS_mode, &sensor, &need_imu)) {
        return 1;
    }

    AtlasConfig config;
    if (!LoadConfig(FLAGS_config, &config)) {
        AERROR << "failed to load config: " << FLAGS_config;
        return 1;
    }
    {
        std::ostringstream oss;
        DumpConfig(oss, config);
        AINFO << oss.str();
    }

    YAML::Node root;
    try {
        root = YAML::LoadFile(FLAGS_config);
    } catch (const std::exception& e) {
        AERROR << "yaml: " << e.what();
        return 1;
    }

    Calib imu_calib;
    if (need_imu) {
        if (!LoadImuCalibFromYaml(root, &config, &imu_calib)) {
            return 1;
        }
    }

    SlamSystem slam;
    if (!slam.Init(config, sensor)) {
        AERROR << "SlamSystem::Init failed";
        return 1;
    }
    if (need_imu) {
        slam.SetImuCalib(imu_calib);
    }

    std::shared_ptr<autolink::Node> viz_node;
    if (FLAGS_viz) {
        if (!autolink::Init(argv[0])) {
            AERROR << "autolink::Init failed (--viz)";
            return 1;
        }
        viz_node = autolink::CreateNode(FLAGS_viz_node);
        if (!viz_node) {
            AERROR << "CreateNode failed: " << FLAGS_viz_node;
            return 1;
        }
        autonomy::localization::atlas::SlamVisualizer::Options viz_opts;
        slam.StartVisualization(viz_node, viz_opts);
        AINFO << "viz enabled: TF/odom/path/points on Autolink node '"
              << FLAGS_viz_node << "'";
    }

    std::ofstream frame_ofs;
    if (!FLAGS_frame_traj_out.empty()) {
        frame_ofs.open(FLAGS_frame_traj_out);
        if (!frame_ofs) {
            AERROR << "cannot write: " << FLAGS_frame_traj_out;
            return 1;
        }
        frame_ofs << "# timestamp tx ty tz qx qy qz qw\n";
    }

    int rc = 1;
    if (FLAGS_format == "euroc") {
        rc = RunEuroc(&slam, need_imu, sensor, &frame_ofs);
    } else if (FLAGS_format == "tum_rgbd") {
        if (sensor != SlamSystem::Sensor::kRgbd &&
            sensor != SlamSystem::Sensor::kImuRgbd) {
            AERROR << "tum_rgbd requires --mode=rgbd";
            return 1;
        }
        rc = RunTumRgbd(&slam, need_imu, &frame_ofs);
    } else if (FLAGS_format == "kitti") {
        if (need_imu) {
            AERROR << "kitti odometry has no synced IMU in this loader";
            return 1;
        }
        rc = RunKitti(&slam, sensor, &frame_ofs);
    } else {
        AERROR << "unknown --format=" << FLAGS_format;
        return 1;
    }
    if (rc != 0) {
        slam.Shutdown();
        if (FLAGS_viz) {
            autolink::Clear();
        }
        return rc;
    }

    if (FLAGS_settle_ms > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(FLAGS_settle_ms));
    }

    const bool use_imu_pose = need_imu;
    SaveKeyframeTrajectoryTum(slam.mutable_map(), FLAGS_traj_out, use_imu_pose);

    if (FLAGS_viz && FLAGS_viz_hold) {
        AINFO << "viz_hold: spinning until Ctrl+C (Autoviz/RViz can subscribe)";
        autolink::Rate rate(10.0);
        while (autolink::OK()) {
            rate.Sleep();
        }
    }

    slam.Shutdown();
    if (FLAGS_viz) {
        autolink::Clear();
    }
    AINFO << "traj=" << FLAGS_traj_out;
    return 0;
}
