/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Shared offline dataset loaders for Atlas evaluation apps.
 */

/**
 * @file dataset_io.hpp
 * @brief Dataset I/O helpers for Atlas offline evaluation (EuRoC / TUM / KITTI).
 *
 * Used by test/app dataset mains to load IMU, mono, stereo, and RGB-D sequences, and to
 * fill IMU calibration from YAML; trajectories are written as TUM text for evo / ORB scripts.
 *
 * @par Typical usage
 * @code{.cpp}
 * std::vector<test_app::ImuSample> imu;
 * test_app::LoadEurocImuCsv(mav0 + "/imu0/data.csv", &imu);
 * std::vector<test_app::CamSample> cam;
 * test_app::LoadEurocCamCsv(mav0 + "/cam0/data.csv",
 *                           mav0 + "/cam0/data", &cam);
 * @endcode
 *
 * @note Timestamps are in seconds (EuRoC CSV ns ×1e-9); paths are absolute or cwd-relative.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_TEST_APP_DATASET_IO_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_TEST_APP_DATASET_IO_HPP_

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Geometry"
#include "yaml-cpp/yaml.h"

#include "autolink/common/log.hpp"

#include "autonomy/localization/atlas/common/config.hpp"
#include "autonomy/localization/atlas/common/types.hpp"
#include "autonomy/localization/atlas/map/keyframe.hpp"
#include "autonomy/localization/atlas/map/map.hpp"
#include "autonomy/localization/atlas/sensor/imu/types.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace test_app {

/**
 * @struct autonomy::localization::atlas::test_app::ImuSample
 * @brief Single IMU sample (EuRoC / generic).
 */
struct ImuSample {
    double t = 0.0;   ///< Timestamp [s]
    double ax = 0;    ///< Acceleration x [m/s²]
    double ay = 0;    ///< Acceleration y
    double az = 0;    ///< Acceleration z
    double wx = 0;    ///< Angular velocity x [rad/s]
    double wy = 0;    ///< Angular velocity y
    double wz = 0;    ///< Angular velocity z
};

/**
 * @struct autonomy::localization::atlas::test_app::CamSample
 * @brief Monocular image frame: timestamp + image file path.
 */
struct CamSample {
    double t = 0.0;      ///< Timestamp [s]
    std::string path;    ///< Absolute/relative image path
};

/**
 * @struct autonomy::localization::atlas::test_app::StereoSample
 * @brief Time-aligned stereo sample.
 */
struct StereoSample {
    double t = 0.0;       ///< Timestamp from left camera [s]
    std::string left;     ///< Left image path
    std::string right;    ///< Right image path
};

/**
 * @struct autonomy::localization::atlas::test_app::RgbdSample
 * @brief Aligned RGB-D sample (TUM style).
 */
struct RgbdSample {
    double t = 0.0;       ///< RGB timestamp [s]
    std::string rgb;      ///< Color image path
    std::string depth;    ///< Depth image path
};

/**
 * @brief Load EuRoC `imu0/data.csv`.
 * @param[in] path CSV path (with header; `#` lines skipped).
 * @param[out] out Samples appended; may clear before calling.
 * @return `true` if opened and at least one valid row.
 *
 * CSV column order (ORB/EuRoC): `timestamp_ns, wx, wy, wz, ax, ay, az`.
 */
inline bool LoadEurocImuCsv(const std::string& path,
                            std::vector<ImuSample>* out) {
    std::ifstream ifs(path);
    if (!ifs) {
        AERROR << "cannot open IMU csv: " << path;
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
        ImuSample s;
        if (!(ss >> stamp_ns >> s.wx >> s.wy >> s.wz >> s.ax >> s.ay >>
              s.az)) {
            continue;
        }
        s.t = static_cast<double>(stamp_ns) * 1e-9;
        out->push_back(s);
    }
    AINFO << "loaded " << out->size() << " IMU samples from " << path;
    return !out->empty();
}

/**
 * @brief Load EuRoC cam0/cam1 data.csv and build image paths.
 * @param[in] csv_path Timestamp–filename CSV.
 * @param[in] img_dir Image directory (usually `…/cam0/data`).
 * @param[out] out Output frame list.
 * @return `true` if at least one valid frame.
 *
 * Each row: `timestamp_ns,filename`; `path = img_dir + "/" + filename`.
 */
inline bool LoadEurocCamCsv(const std::string& csv_path,
                            const std::string& img_dir,
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

/**
 * @brief Associate left/right cameras into stereo pairs by nearest timestamp.
 * @param[in] left Left sequence (preferably time-sorted ascending).
 * @param[in] right Right sequence.
 * @param[in] max_dt Max allowed time delta [s]; drop left frame if exceeded.
 * @param[out] out Cleared then filled; timestamp from left.
 * @return `true` if at least one pair succeeded.
 */
inline bool AssociateStereo(const std::vector<CamSample>& left,
                            const std::vector<CamSample>& right,
                            double max_dt, std::vector<StereoSample>* out) {
    out->clear();
    if (left.empty() || right.empty()) {
        return false;
    }
    size_t j = 0;
    for (const auto& L : left) {
        while (j + 1 < right.size() &&
               std::abs(right[j + 1].t - L.t) <= std::abs(right[j].t - L.t)) {
            ++j;
        }
        if (std::abs(right[j].t - L.t) > max_dt) {
            continue;
        }
        StereoSample s;
        s.t = L.t;
        s.left = L.path;
        s.right = right[j].path;
        out->push_back(s);
    }
    AINFO << "stereo pairs=" << out->size();
    return !out->empty();
}

/**
 * @brief Load TUM RGB-D `associations.txt`.
 * @param[in] path Associations file.
 * @param[in] root Dataset root (for relative paths).
 * @param[out] out RGB-D pairs.
 * @return `true` if at least one pair.
 *
 * Row format: `t_rgb rgb_rel t_depth depth_rel` (space-separated; `#` comments ignored).
 */
inline bool LoadTumAssociations(const std::string& path,
                                const std::string& root,
                                std::vector<RgbdSample>* out) {
    std::ifstream ifs(path);
    if (!ifs) {
        AERROR << "cannot open associations: " << path;
        return false;
    }
    std::string line;
    while (std::getline(ifs, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream ss(line);
        double t_rgb = 0, t_depth = 0;
        std::string rgb_rel, depth_rel;
        if (!(ss >> t_rgb >> rgb_rel >> t_depth >> depth_rel)) {
            continue;
        }
        RgbdSample s;
        s.t = t_rgb;
        s.rgb = root + "/" + rgb_rel;
        s.depth = root + "/" + depth_rel;
        out->push_back(s);
    }
    AINFO << "loaded " << out->size() << " RGB-D pairs from " << path;
    return !out->empty();
}

/**
 * @brief Without associations, nearest-neighbor associate `rgb.txt` + `depth.txt`.
 * @param[in] root TUM sequence root (contains `rgb.txt` / `depth.txt`).
 * @param[in] max_dt Max time delta [s].
 * @param[out] out RGB-D pairs.
 * @return `true` if association succeeded and non-empty.
 */
inline bool LoadTumRgbDepthTxt(const std::string& root, double max_dt,
                               std::vector<RgbdSample>* out) {
    auto LoadList = [](const std::string& path,
                       std::vector<std::pair<double, std::string>>* list) {
        std::ifstream ifs(path);
        if (!ifs) {
            return false;
        }
        std::string line;
        while (std::getline(ifs, line)) {
            if (line.empty() || line[0] == '#') {
                continue;
            }
            std::istringstream ss(line);
            double t = 0;
            std::string rel;
            if (!(ss >> t >> rel)) {
                continue;
            }
            list->emplace_back(t, rel);
        }
        return !list->empty();
    };

    std::vector<std::pair<double, std::string>> rgb, depth;
    if (!LoadList(root + "/rgb.txt", &rgb) ||
        !LoadList(root + "/depth.txt", &depth)) {
        AERROR << "need " << root << "/rgb.txt and depth.txt (or associations)";
        return false;
    }
    size_t j = 0;
    for (const auto& [t, rel] : rgb) {
        while (j + 1 < depth.size() &&
               std::abs(depth[j + 1].first - t) <=
                   std::abs(depth[j].first - t)) {
            ++j;
        }
        if (std::abs(depth[j].first - t) > max_dt) {
            continue;
        }
        RgbdSample s;
        s.t = t;
        s.rgb = root + "/" + rel;
        s.depth = root + "/" + depth[j].second;
        out->push_back(s);
    }
    AINFO << "associated RGB-D pairs=" << out->size();
    return !out->empty();
}

/**
 * @brief Load KITTI odometry mono sequence (`times.txt` + `image_0/######.png`).
 * @param[in] seq_root Sequence directory (e.g. `…/sequences/00`).
 * @param[out] out Cleared then filled.
 * @return `true` if at least one frame.
 */
inline bool LoadKittiMono(const std::string& seq_root,
                          std::vector<CamSample>* out) {
    std::ifstream ftimes(seq_root + "/times.txt");
    if (!ftimes) {
        AERROR << "cannot open " << seq_root << "/times.txt";
        return false;
    }
    std::vector<double> times;
    std::string line;
    while (std::getline(ftimes, line)) {
        if (line.empty()) {
            continue;
        }
        std::istringstream ss(line);
        double t = 0;
        if (ss >> t) {
            times.push_back(t);
        }
    }
    out->clear();
    out->reserve(times.size());
    for (size_t i = 0; i < times.size(); ++i) {
        std::ostringstream name;
        name << seq_root << "/image_0/" << std::setfill('0') << std::setw(6)
             << i << ".png";
        CamSample s;
        s.t = times[i];
        s.path = name.str();
        out->push_back(s);
    }
    AINFO << "loaded KITTI mono frames=" << out->size();
    return !out->empty();
}

/**
 * @brief Load KITTI stereo (left `image_0`, right `image_1`, same-index aligned).
 * @param[in] seq_root Sequence directory.
 * @param[out] out Stereo pairs.
 * @return `true` on success.
 */
inline bool LoadKittiStereo(const std::string& seq_root,
                            std::vector<StereoSample>* out) {
    std::vector<CamSample> left;
    if (!LoadKittiMono(seq_root, &left)) {
        return false;
    }
    out->clear();
    out->reserve(left.size());
    for (size_t i = 0; i < left.size(); ++i) {
        std::ostringstream right;
        right << seq_root << "/image_1/" << std::setfill('0') << std::setw(6)
              << i << ".png";
        StereoSample s;
        s.t = left[i].t;
        s.left = left[i].path;
        s.right = right.str();
        out->push_back(s);
    }
    AINFO << "loaded KITTI stereo pairs=" << out->size();
    return !out->empty();
}

/**
 * @brief Fill IMU noise and `T_b_c` calibration from YAML (and filled AtlasConfig).
 * @param[in] root Parsed YAML root; may read `imu.noise_*` / `imu.T_b_c`.
 * @param[in,out] cfg Config; noise and `T_b_c` written back.
 * @param[out] calib IMU calibration after `Calib::Set`.
 * @return `false` if `T_b_c` dimension is invalid; otherwise `true`.
 *
 * Prefer `cfg->T_b_c` if already a 16-element row-major 4×4; YAML `imu` may override.
 */
inline bool LoadImuCalibFromYaml(const YAML::Node& root, AtlasConfig* cfg,
                                 sensor::imu::Calib* calib) {
    double ng = cfg->imu_gyro_noise;
    double na = cfg->imu_accel_noise;
    double ngw = cfg->imu_gyro_bias_random_walk;
    double naw = cfg->imu_accel_bias_random_walk;
    SE3 T_bc = SE3Identity();

    auto apply_tbc = [&](const std::vector<double>& v) -> bool {
        if (v.size() != 16) {
            AERROR << "imu.T_b_c must be 16 doubles (row-major 4x4)";
            return false;
        }
        Mat33 R;
        R << v[0], v[1], v[2], v[4], v[5], v[6], v[8], v[9], v[10];
        const Vec3 t(v[3], v[7], v[11]);
        T_bc = SE3Identity();
        T_bc.linear() = R;
        T_bc.translation() = t;
        return true;
    };

    // Prefer already-loaded AtlasConfig (LoadConfig fills imu.*).
    if (cfg->T_b_c.size() == 16) {
        if (!apply_tbc(cfg->T_b_c)) {
            return false;
        }
    }

    if (root["imu"]) {
        const auto& imu = root["imu"];
        if (imu["noise_gyro"]) {
            ng = imu["noise_gyro"].as<double>();
        }
        if (imu["noise_acc"]) {
            na = imu["noise_acc"].as<double>();
        }
        if (imu["walk_gyro"]) {
            ngw = imu["walk_gyro"].as<double>();
        }
        if (imu["walk_acc"]) {
            naw = imu["walk_acc"].as<double>();
        }
        if (imu["T_b_c"]) {
            const auto v = imu["T_b_c"].as<std::vector<double>>();
            if (!apply_tbc(v)) {
                return false;
            }
            cfg->T_b_c = v;
        }
    }

    cfg->imu_gyro_noise = ng;
    cfg->imu_accel_noise = na;
    cfg->imu_gyro_bias_random_walk = ngw;
    cfg->imu_accel_bias_random_walk = naw;
    calib->Set(T_bc, ng, na, ngw, naw);
    return true;
}

/**
 * @brief Write one TUM pose line: `t tx ty tz qx qy qz qw`.
 * @param[in,out] os Output stream.
 * @param[in] t_sec Timestamp [s].
 * @param[in] Tw World←body/camera (translation + rotation quaternion xyzw).
 */
inline void WriteTumPose(std::ostream& os, double t_sec, const SE3& Tw) {
    const Eigen::Quaterniond q(Tw.rotation());
    const Eigen::Vector3d p = Tw.translation();
    os << std::fixed << t_sec << " " << p.x() << " " << p.y() << " " << p.z()
       << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
}

/**
 * @brief Save map keyframe trajectory as TUM text.
 * @param[in] map Non-null map; skip `isBad()` keyframes.
 * @param[in] path Output file path.
 * @param[in] use_imu_pose `true` writes `GetImuPose()` (\(T_{wb}\));
 *            `false` writes inverse camera pose (\(T_{wc}=T_{cw}^{-1}\)).
 * @return `true` if at least one frame was written.
 *
 * Keyframes sorted by `timestamp` ascending; header `# timestamp tx ty tz qx qy qz qw`.
 */
inline bool SaveKeyframeTrajectoryTum(Map* map, const std::string& path,
                                      bool use_imu_pose) {
    if (!map) {
        return false;
    }
    auto keyframes = map->GetAllKeyFrames();
    std::sort(keyframes.begin(), keyframes.end(),
              [](const auto& a, const auto& b) {
                  return a->timestamp < b->timestamp;
              });
    std::ofstream ofs(path);
    if (!ofs) {
        AERROR << "cannot write: " << path;
        return false;
    }
    ofs << "# timestamp tx ty tz qx qy qz qw\n";
    int n = 0;
    for (const auto& kf : keyframes) {
        if (!kf || kf->isBad()) {
            continue;
        }
        const SE3 Tw =
            use_imu_pose ? kf->GetImuPose() : kf->GetPose().inverse();
        WriteTumPose(ofs, kf->timestamp, Tw);
        ++n;
    }
    AINFO << "wrote " << n << " keyframes → " << path;
    return n > 0;
}

}  // namespace test_app
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_TEST_APP_DATASET_IO_HPP_
