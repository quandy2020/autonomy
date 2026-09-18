/*
 * Copyright 2026 The Openbot Authors
 *
 * Offline EuRoC mono-inertial runner for Atlas VIO.
 *
 * Usage:
 *   autonomy.localization.euroc_vio \
 *     --dataset=/path/to/MH_01_easy \
 *     --config=.../EuRoC_mono_inertial.yaml \
 *     --vocab=.../orb_vocab.fbow \
 *     --traj_out=/tmp/kf_traj.txt
 *
 * Dataset layout (ASL EuRoC):
 *   <dataset>/mav0/cam0/data.csv
 *   <dataset>/mav0/cam0/data/<stamp>.png
 *   <dataset>/mav0/imu0/data.csv
 */

#include "autonomy/localization/atlas/config.hpp"
#include "autonomy/localization/atlas/system.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "gflags/gflags.h"
#include "glog/logging.h"
#include "autolink/common/log.hpp"

DEFINE_string(dataset, "", "EuRoC sequence root (contains mav0/)");
DEFINE_string(config, "", "Atlas YAML (e.g. EuRoC_mono_inertial.yaml)");
DEFINE_string(vocab, "autonomy/localization/conf/atlas/orb_vocab.fbow",
              "ORB vocabulary (.fbow)");
DEFINE_string(traj_out, "keyframe_trajectory_euroc.txt",
              "Output keyframe trajectory (TUM format)");
DEFINE_string(frame_traj_out, "",
              "Optional frame trajectory output (TUM). Empty = skip.");
DEFINE_int32(max_frames, -1, "Stop after N camera frames (-1 = all)");
DEFINE_bool(wait_shutdown, true, "Wait for mapping/loop threads on exit");

namespace {

struct ImuSample {
    double t = 0.0;
    double ax = 0, ay = 0, az = 0;
    double wx = 0, wy = 0, wz = 0;
};

struct CamSample {
    double t = 0.0;
    std::string path;
};

bool LoadImuCsv(const std::string& path, std::vector<ImuSample>* out) {
    std::ifstream ifs(path);
    if (!ifs) {
        AERROR << "cannot open IMU csv: " << path;
        return false;
    }
    std::string line;
    // skip header
    std::getline(ifs, line);
    while (std::getline(ifs, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream ss(line);
        long long stamp_ns = 0;
        ImuSample s;
        if (!(ss >> stamp_ns >> s.wx >> s.wy >> s.wz >> s.ax >> s.ay >> s.az)) {
            continue;
        }
        s.t = static_cast<double>(stamp_ns) * 1e-9;
        out->push_back(s);
    }
    AINFO << "loaded " << out->size() << " IMU samples from " << path;
    return !out->empty();
}

bool LoadCamCsv(const std::string& csv_path, const std::string& img_dir,
                std::vector<CamSample>* out) {
    std::ifstream ifs(csv_path);
    if (!ifs) {
        AERROR << "cannot open cam csv: " << csv_path;
        return false;
    }
    std::string line;
    std::getline(ifs, line);
    while (std::getline(ifs, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream ss(line);
        long long stamp_ns = 0;
        std::string fname;
        if (!(ss >> stamp_ns >> fname)) {
            continue;
        }
        CamSample s;
        s.t = static_cast<double>(stamp_ns) * 1e-9;
        s.path = img_dir + "/" + fname;
        out->push_back(s);
    }
    AINFO << "loaded " << out->size() << " camera frames from " << csv_path;
    return !out->empty();
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

    const std::string mav0 = FLAGS_dataset + "/mav0";
    std::vector<ImuSample> imu;
    std::vector<CamSample> cams;
    if (!LoadImuCsv(mav0 + "/imu0/data.csv", &imu)) {
        return 1;
    }
    if (!LoadCamCsv(mav0 + "/cam0/data.csv", mav0 + "/cam0/data", &cams)) {
        return 1;
    }

    auto cfg = std::make_shared<autonomy::localization::atlas::config>(FLAGS_config);
    autonomy::localization::atlas::system slam(cfg, FLAGS_vocab);
    slam.startup(/*need_initialize=*/true);

    std::size_t imu_idx = 0;
    int fed = 0;
    const auto t0 = std::chrono::steady_clock::now();

    for (const auto& cam : cams) {
        if (FLAGS_max_frames >= 0 && fed >= FLAGS_max_frames) {
            break;
        }
        // Feed all IMU up to (and including) this image time.
        while (imu_idx < imu.size() && imu[imu_idx].t <= cam.t) {
            const auto& m = imu[imu_idx++];
            slam.feed_imu(m.t, m.ax, m.ay, m.az, m.wx, m.wy, m.wz);
        }

        cv::Mat img = cv::imread(cam.path, cv::IMREAD_GRAYSCALE);
        if (img.empty()) {
            AWARN << "skip missing image: " << cam.path;
            continue;
        }
        slam.feed_monocular_frame(img, cam.t);
        ++fed;
        if (fed % 50 == 0) {
            AINFO << "fed " << fed << "/" << cams.size() << " frames, imu_idx=" << imu_idx;
        }
    }

    // Drain remaining IMU past last frame (optional, helps last preint).
    while (imu_idx < imu.size()) {
        const auto& m = imu[imu_idx++];
        slam.feed_imu(m.t, m.ax, m.ay, m.az, m.wx, m.wy, m.wz);
    }

    if (FLAGS_wait_shutdown) {
        // Give local mapping / loop a moment to settle.
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    slam.shutdown();

    slam.save_keyframe_trajectory(FLAGS_traj_out, "TUM");
    if (!FLAGS_frame_traj_out.empty()) {
        slam.save_frame_trajectory(FLAGS_frame_traj_out, "TUM");
    }

    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0)
                        .count();
    AINFO << "done: frames=" << fed << " elapsed_ms=" << ms
          << " traj=" << FLAGS_traj_out;
    return 0;
}
