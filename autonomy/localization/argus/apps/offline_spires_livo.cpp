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
 */

/**
 * @file offline_spires_livo.cpp
 * @brief Oxford Spires raw → LivoOdometry (LIO/LIVO) → TUM estimate.
 *
 * Usage:
 *   offline_spires_livo --spires <seq_dir> --calib <calibration_dir>
 *       [--config livo_esikf.yaml] [--camera-yaml cam.yaml]
 *       [--out-dir <dir>] [--max-frames N] [--mode lio|livo|lo]
 *       [--camera cam0] [--loop|--no-loop]
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <filesystem>
#include <opencv2/core.hpp>

#include "argus/common/logger.hpp"
#include "argus/map/voxel_map.hpp"
#include "argus/odometry/load_livo_config.hpp"
#include "argus/odometry/livo_odometry.hpp"
#include "argus/sensor/image_data.hpp"
#include "argus/sensor/imu_data.hpp"
#include "apps/datasets/oxford_spires_dataset.hpp"

namespace fs = std::filesystem;

namespace {

struct ImuSample {
  double t = 0.0;
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyr = Eigen::Vector3d::Zero();
};

bool LoadImuCsv(const fs::path& path, std::vector<ImuSample>* out) {
  if (out == nullptr || !fs::exists(path)) {
    return false;
  }
  std::ifstream ifs(path);
  if (!ifs) {
    return false;
  }
  std::string line;
  std::getline(ifs, line);  // header
  out->clear();
  while (std::getline(ifs, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    for (char& c : line) {
      if (c == ',') {
        c = ' ';
      }
    }
    std::istringstream ss(line);
    double sec = 0, nsec = 0, ax = 0, ay = 0, az = 0, wx = 0, wy = 0, wz = 0;
    if (!(ss >> sec >> nsec >> ax >> ay >> az >> wx >> wy >> wz)) {
      continue;
    }
    ImuSample s;
    s.t = sec + nsec * 1e-9;
    s.acc = Eigen::Vector3d(ax, ay, az);
    s.gyr = Eigen::Vector3d(wx, wy, wz);
    out->push_back(s);
  }
  return !out->empty();
}

/** @brief Parse cam0 T_cam_lidar + T_cam_imu → Lidar→IMU (R,t). */
bool LoadLidarImuExtrinsic(const fs::path& yaml_path, Eigen::Matrix3d* R_li,
                           Eigen::Vector3d* t_li) {
  if (R_li == nullptr || t_li == nullptr || !fs::exists(yaml_path)) {
    return false;
  }
  // Minimal line parser for DiffCal-style YAML (no full YAML dependency).
  std::ifstream ifs(yaml_path);
  if (!ifs) {
    return false;
  }
  std::string text((std::istreambuf_iterator<char>(ifs)),
                   std::istreambuf_iterator<char>());
  auto find_arr = [&](const std::string& key, int n,
                      std::vector<double>* v) -> bool {
    const auto pos = text.find(key);
    if (pos == std::string::npos) {
      return false;
    }
    const auto lb = text.find('[', pos);
    const auto rb = text.find(']', lb);
    if (lb == std::string::npos || rb == std::string::npos) {
      return false;
    }
    std::string inner = text.substr(lb + 1, rb - lb - 1);
    for (char& c : inner) {
      if (c == ',') {
        c = ' ';
      }
    }
    std::istringstream ss(inner);
    v->clear();
    double x;
    while (ss >> x) {
      v->push_back(x);
    }
    return static_cast<int>(v->size()) >= n;
  };

  std::vector<double> q_ci, r_ci, Tflat;
  // First cam0 block: C_q_CI / C_r_CI / T_cam_lidar
  if (!find_arr("C_q_CI:", 4, &q_ci) || !find_arr("C_r_CI:", 3, &r_ci)) {
    return false;
  }
  // Flatten first 4x4 T_cam_lidar under cam0
  const auto tpos = text.find("T_cam_lidar:");
  if (tpos == std::string::npos) {
    return false;
  }
  std::string block = text.substr(tpos, 800);
  for (char& c : block) {
    if (c == ',' || c == '[' || c == ']' || c == ':') {
      c = ' ';
    }
  }
  std::istringstream ss(block);
  std::string tok;
  ss >> tok;  // T_cam_lidar
  Tflat.clear();
  double x;
  while (ss >> x && Tflat.size() < 16) {
    Tflat.push_back(x);
  }
  if (Tflat.size() < 16) {
    return false;
  }

  Eigen::Matrix4d T_CL = Eigen::Matrix4d::Identity();
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      T_CL(r, c) = Tflat[static_cast<std::size_t>(r * 4 + c)];
    }
  }
  // DiffCal: [qx,qy,qz,qw]
  Eigen::Quaterniond q_CI(q_ci[3], q_ci[0], q_ci[1], q_ci[2]);
  q_CI.normalize();
  Eigen::Matrix4d T_CI = Eigen::Matrix4d::Identity();
  T_CI.block<3, 3>(0, 0) = q_CI.toRotationMatrix();
  T_CI.block<3, 1>(0, 3) = Eigen::Vector3d(r_ci[0], r_ci[1], r_ci[2]);

  // p_c = T_CL p_l = T_CI p_i  →  p_i = T_CI^{-1} T_CL p_l
  // Lidar→IMU: T_IL = T_CI^{-1} * T_CL
  const Eigen::Matrix4d T_IL = T_CI.inverse() * T_CL;
  *R_li = T_IL.block<3, 3>(0, 0);
  *t_li = T_IL.block<3, 1>(0, 3);
  return true;
}

void ApplySpiresCalib(argus::odometry::LivoOdometryConfig* cfg,
                      const argus::datasets::OxfordSpiresCalib& calib,
                      const Eigen::Matrix3d& R_li, const Eigen::Vector3d& t_li,
                      bool have_li) {
  if (cfg == nullptr) {
    return;
  }
  if (calib.has_extrinsics) {
    const Eigen::Matrix3d R = calib.R_cl();
    const Eigen::Vector3d t = calib.t_cl();
    cfg->vio.Rcl = {R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2),
                    R(2, 0), R(2, 1), R(2, 2)};
    cfg->vio.Pcl = {t.x(), t.y(), t.z()};
  }
  if (have_li) {
    cfg->ext_R = R_li;
    cfg->ext_T = t_li;
  }
}

Eigen::Quaterniond RotToQuat(const Eigen::Matrix3d& R) {
  return Eigen::Quaterniond(R).normalized();
}

bool WriteTumLine(std::ofstream& ofs, double t, const Eigen::Vector3d& p,
                  const Eigen::Matrix3d& R) {
  const Eigen::Quaterniond q = RotToQuat(R);
  ofs << std::fixed << t << " " << p.x() << " " << p.y() << " " << p.z() << " "
      << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
  return static_cast<bool>(ofs);
}

}  // namespace

int main(int argc, char** argv) {
  argus::common::InitLogging(argv[0]);

  std::string yaml = "config/livo_esikf.yaml";
  std::string spires_root;
  std::string calib_dir;
  std::string camera_yaml;
  std::string out_dir;
  std::string mode = "livo";  // livo | lio | lo
  std::string camera = "cam0";
  int max_frames = 0;  // 0 = all
  int loop_override = 0;  // 0=yaml, 1=force on, -1=force off
  int drift_enable = -1;  // -1=yaml, 0=off, 1=on
  double drift_yaw_per_m = -1.0;
  double drift_tx_per_m = -1.0;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto need = [&](const char* name) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "missing value for " << name << "\n";
        std::exit(2);
      }
      return argv[++i];
    };
    if (a == "--spires") {
      spires_root = need("--spires");
    } else if (a == "--calib") {
      calib_dir = need("--calib");
    } else if (a == "--config") {
      yaml = need("--config");
    } else if (a == "--camera-yaml") {
      camera_yaml = need("--camera-yaml");
    } else if (a == "--out-dir") {
      out_dir = need("--out-dir");
    } else if (a == "--max-frames") {
      max_frames = std::stoi(need("--max-frames"));
    } else if (a == "--mode") {
      mode = need("--mode");
    } else if (a == "--camera") {
      camera = need("--camera");
    } else if (a == "--loop") {
      loop_override = 1;
    } else if (a == "--no-loop") {
      loop_override = -1;
    } else if (a == "--drift") {
      drift_enable = 1;
    } else if (a == "--no-drift") {
      drift_enable = 0;
    } else if (a == "--drift-yaw-per-m") {
      drift_yaw_per_m = std::stod(need("--drift-yaw-per-m"));
      drift_enable = 1;
    } else if (a == "--drift-tx-per-m") {
      drift_tx_per_m = std::stod(need("--drift-tx-per-m"));
      drift_enable = 1;
    } else if (a == "--help" || a == "-h") {
      std::cout
          << "Usage: offline_spires_livo --spires <seq> --calib <dir> "
             "[--mode livo|lio|lo] [--loop|--no-loop] "
             "[--drift|--drift-yaw-per-m R]\n";
      return 0;
    }
  }

  if (spires_root.empty()) {
    std::cerr << "need --spires <sequence_dir>\n";
    return 2;
  }
  if (out_dir.empty()) {
    out_dir = (fs::path(spires_root) / "argus_eval").string();
  }
  fs::create_directories(out_dir);

  argus::datasets::OxfordSpiresOptions opt;
  opt.camera = camera;
  if (!calib_dir.empty()) {
    opt.calibration_dir = calib_dir;
  }
  argus::datasets::OxfordSpiresDataset ds;
  if (!ds.Open(spires_root, opt)) {
    std::cerr << "OxfordSpires Open failed: " << spires_root << "\n";
    return 1;
  }

  std::vector<ImuSample> imu;
  const fs::path imu_path = fs::path(ds.raw_dir()) / "imu.csv";
  const bool have_imu = LoadImuCsv(imu_path, &imu);
  std::cout << "[offline_spires] lidar_frames=" << ds.size()
            << " imu=" << (have_imu ? static_cast<int>(imu.size()) : 0)
            << " calib_K=" << ds.calib().has_intrinsics
            << " T_cl=" << ds.calib().has_extrinsics << "\n";

  // Prefer YAML load; on OpenCV 5 sequence-parse failures fall back to a
  // minimal in-code config sufficient for offline Spires evaluation.
  argus::odometry::LivoOdometryConfig cfg;
  bool yaml_ok = false;
  try {
    const auto loaded = argus::odometry::LoadLivoOdometryConfig(yaml);
    if (loaded.ok) {
      cfg = loaded.config;
      yaml_ok = true;
      std::cout << "[offline_spires] loaded yaml: " << yaml << "\n";
    } else {
      std::cerr << "[offline_spires] yaml load failed (" << loaded.error
                << "); using defaults\n";
    }
  } catch (const cv::Exception& e) {
    std::cerr << "[offline_spires] yaml OpenCV exception: " << e.what()
              << "; using defaults\n";
  } catch (const std::exception& e) {
    std::cerr << "[offline_spires] yaml exception: " << e.what()
              << "; using defaults\n";
  }
  if (!yaml_ok) {
    argus::map::SetDefaultVoxelMapConfig(&cfg.voxel);
    cfg.surface_voxel_size = 0.5;
    cfg.imu_init_frames = 20;
    cfg.enable_mapping = true;
  }
  cfg.enable_mapping = true;
  cfg.mapping.session_id = "spires";
  cfg.mapping_output_dir = (fs::path(out_dir) / "map").string();
  cfg.relocalization.enable = false;
  if (loop_override > 0) {
    cfg.loop_closure.enable = true;
  } else if (loop_override < 0) {
    cfg.loop_closure.enable = false;
  }
  if (drift_enable == 0) {
    cfg.synthetic_drift.enable = false;
  } else if (drift_enable > 0) {
    cfg.synthetic_drift.enable = true;
    if (drift_yaw_per_m >= 0.0) {
      cfg.synthetic_drift.yaw_rad_per_m = drift_yaw_per_m;
    }
    if (drift_tx_per_m >= 0.0) {
      cfg.synthetic_drift.tx_m_per_m = drift_tx_per_m;
    }
    if (cfg.synthetic_drift.yaw_rad_per_m == 0.0 &&
        cfg.synthetic_drift.tx_m_per_m == 0.0) {
      // Defaults chosen so Umeyama-aligned ATE still sees residual error
      // (pure yaw is largely absorbed by trajectory alignment).
      cfg.synthetic_drift.yaw_rad_per_m = 0.002;
      cfg.synthetic_drift.tx_m_per_m = 0.008;
    }
  }
  // Offline eval: keep async worker (refuse sync Optimize).
  if (cfg.loop_closure.enable) {
    cfg.loop_closure.async = true;
  }
  cfg.enable_degeneracy = true;
  cfg.dynamic_filter.lidar = argus::frontend::DynamicFilterType::kNone;
  cfg.dynamic_filter.visual = argus::frontend::DynamicFilterType::kNone;
  std::cout << "[offline_spires] loop_closure.enable="
            << (cfg.loop_closure.enable ? 1 : 0)
            << " fpr=" << (cfg.loop_closure.fpr.enable ? 1 : 0)
            << " lta=" << (cfg.loop_closure.lta.enable ? 1 : 0)
            << " drift=" << (cfg.synthetic_drift.enable ? 1 : 0)
            << " yaw_per_m=" << cfg.synthetic_drift.yaw_rad_per_m << "\n";

  if (mode == "lo") {
    cfg.slam_mode = argus::frontend::ONLY_LO;
    cfg.enable_lidar = true;
    cfg.enable_imu = false;
    cfg.enable_image = false;
  } else if (mode == "lio") {
    cfg.slam_mode = argus::frontend::kOnlyLio;
    cfg.enable_lidar = true;
    cfg.enable_imu = have_imu;
    cfg.enable_image = false;
  } else {
    cfg.slam_mode = argus::frontend::LIVO;
    cfg.enable_lidar = true;
    cfg.enable_imu = have_imu;
    cfg.enable_image = true;
    if (!camera_yaml.empty()) {
      cfg.vio.camera_calib_file = camera_yaml;
    }
  }

  Eigen::Matrix3d R_li = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_li = Eigen::Vector3d::Zero();
  bool have_li = false;
  if (!calib_dir.empty()) {
    have_li = LoadLidarImuExtrinsic(
        fs::path(calib_dir) / "cam-lidar-imu.yaml", &R_li, &t_li);
  }
  ApplySpiresCalib(&cfg, ds.calib(), R_li, t_li, have_li);

  argus::odometry::LivoOdometry odom(cfg);
  if (cfg.enable_image) {
    if (!odom.InitializeVio()) {
      std::cerr << "[offline_spires] InitializeVio failed; falling back to LIO\n";
      cfg.slam_mode = argus::frontend::kOnlyLio;
      cfg.enable_image = false;
      odom.SetConfig(cfg);
      odom.Initialize();
    }
  }

  const std::string est_path = (fs::path(out_dir) / "estimate.tum").string();
  std::ofstream tum(est_path);
  if (!tum) {
    std::cerr << "cannot write " << est_path << "\n";
    return 1;
  }

  int limit = static_cast<int>(ds.size());
  if (max_frames > 0) {
    limit = std::min(limit, max_frames);
  }

  std::size_t imu_i = 0;
  int processed = 0;
  for (int i = 0; i < limit; ++i) {
    argus::datasets::OxfordSpiresFrame fr;
    const bool want_img = cfg.enable_image;
    if (!ds.Read(static_cast<std::size_t>(i), &fr, want_img)) {
      std::cerr << "Read failed at " << i << "\n";
      break;
    }

    if (cfg.enable_imu && have_imu) {
      while (imu_i < imu.size() && imu[imu_i].t <= fr.timestamp + 1e-4) {
        argus::sensor::ImuData m;
        m.timestamp = imu[imu_i].t;
        m.linear_acceleration = imu[imu_i].acc;
        m.angular_velocity = imu[imu_i].gyr;
        odom.AddImu(m);
        ++imu_i;
      }
    }

    if (want_img && !fr.image.empty()) {
      argus::sensor::ImageData img;
      img.timestamp = fr.image_timestamp > 0 ? fr.image_timestamp : fr.timestamp;
      img.frame_id = camera;
      img.encoding = "bgr8";
      img.image = fr.image;
      odom.AddImage(img);
    }

    odom.AddLidar(fr.cloud, fr.timestamp);
    // Drain sync packages for this sweep; log one TUM pose at LiDAR time so
    // the estimate spans the full sequence (LIVO may emit LIO+VIO packages).
    argus::odometry::OdometryOutput out;
    bool any = false;
    argus::odometry::ProcessStatus last_status =
        argus::odometry::ProcessStatus::kNoPackage;
    while (odom.Process(&out)) {
      any = true;
      last_status = out.status;
    }
    if (!any) {
      continue;
    }
    WriteTumLine(tum, fr.timestamp, out.state.pos_end, out.state.rot_end);
    tum.flush();
    ++processed;
    if (processed % 50 == 0 || i + 1 == limit) {
      std::cout << "[offline_spires] i=" << i << " processed=" << processed
                << " status=" << argus::odometry::ProcessStatusName(last_status)
                << " t_lidar=" << fr.timestamp << "\n";
    }
  }
  tum.close();

  if (processed == 0) {
    std::cerr << "[offline_spires] no frames processed\n";
    return 1;
  }

  if (auto* lc = odom.loop_closure_manager()) {
    // Drain async detections / FPR for the last keyframes.
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    lc->Stop();
    const auto st = lc->Stats();
    std::cout << "[offline_spires] loop stats: accepted=" << st.accepted
              << " fpr_reject=" << st.fpr_reject
              << " inject_restore=" << st.inject_restore
              << " geom_reject=" << st.geometric_reject
              << " geom{miss=" << st.geom_missing_cloud
              << ",fit=" << st.geom_fitness << ",corr=" << st.geom_corr
              << ",t=" << st.geom_translation << ",yaw=" << st.geom_yaw
              << ",inlier=" << st.geom_inlier << "}"
              << " temporal_reject=" << st.temporal_reject
              << " temporal_pending=" << st.temporal_pending
              << " signature_reject=" << st.signature_reject
              << " cross_session=" << st.cross_session
              << " prior_clouds=" << lc->prior_cloud_count() << "\n";
  }
  if (cfg.synthetic_drift.enable) {
    std::cout << "[offline_spires] synthetic_drift_distance_m="
              << odom.synthetic_drift_distance_m() << "\n";
  }

  odom.SaveBuiltMap();
  std::cout << "[offline_spires] done processed=" << processed
            << " estimate=" << est_path << " map=" << cfg.mapping_output_dir
            << "\n";
  return 0;
}
