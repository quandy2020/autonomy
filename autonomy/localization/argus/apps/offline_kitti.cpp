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
 * @file offline_kitti.cpp
 * @brief KITTI subset → semantic dynamic filter → LivoOdometry LO Process.
 *
 * Usage:
 *   offline_kitti --kitti <subset_dir> [--config livo_esikf.yaml]
 *                 [--max-frames N] [--expect-reject-ratio R]
 *                 [--filter-only]
 *
 * Default semantic backend: mask_file (no ONNX required for CI).
 * With ONNX: set frontend.dynamic_filter.semantic_model + backend=onnx.
 */

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

#include "argus/common/logger.hpp"
#include "argus/frontend/semantic_dynamic_filter.hpp"
#include "argus/odometry/load_livo_config.hpp"
#include "argus/odometry/livo_odometry.hpp"
#include "argus/sensor/image_data.hpp"
#include "apps/datasets/kitti_dataset.hpp"

namespace {

void ApplyKittiCalib(argus::odometry::LivoOdometryConfig* cfg,
                     const argus::datasets::KittiCalib& calib) {
  if (cfg == nullptr) {
    return;
  }
  if (calib.has_Tr) {
    const Eigen::Matrix3d R = calib.R_cl();
    const Eigen::Vector3d t = calib.t_cl();
    cfg->vio.Rcl = {R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2),
                    R(2, 0), R(2, 1), R(2, 2)};
    cfg->vio.Pcl = {t.x(), t.y(), t.z()};
  }
}

void ConfigureFilterFromCalib(argus::frontend::SemanticDynamicFilter* sem,
                              const argus::datasets::KittiCalib& calib) {
  if (sem == nullptr) {
    return;
  }
  if (calib.has_P2) {
    sem->SetCameraIntrinsics(calib.fx(), calib.fy(), calib.cx(), calib.cy(),
                             1242, 375);
  }
  if (calib.has_Tr) {
    sem->SetLidarCameraExtrinsic(calib.R_cl(), calib.t_cl());
  }
}

int RunFilterOnly(const argus::datasets::KittiDataset& ds, int max_frames,
                  double expect_reject) {
  argus::frontend::SemanticDynamicFilterConfig scfg;
  scfg.backend = argus::frontend::SemanticBackend::kMaskFile;
  argus::frontend::SemanticDynamicFilter filter(scfg);
  ConfigureFilterFromCalib(&filter, ds.calib());
  argus::frontend::StatesGroup state;
  double last_ratio = 0.0;
  int n = 0;
  const int limit = std::min(max_frames, static_cast<int>(ds.size()));
  for (int i = 0; i < limit; ++i) {
    argus::datasets::KittiFrame fr;
    if (!ds.Read(static_cast<std::size_t>(i), &fr)) {
      break;
    }
    if (!fr.keep_mask.empty()) {
      filter.SetKeepMask(fr.keep_mask);
    } else if (!fr.image.empty()) {
      cv::Mat keep(fr.image.rows, fr.image.cols, CV_8UC1, cv::Scalar(1));
      filter.SetKeepMask(keep);
    }
    const auto mask = filter.FilterLidar(*fr.cloud, state);
    last_ratio = mask.RejectedRatio();
    ++n;
    std::cout << "[offline_kitti] frame=" << i << " pts=" << fr.cloud->size()
              << " reject=" << last_ratio << "\n";
  }
  if (n == 0) {
    std::cerr << "[offline_kitti] no frames\n";
    return 1;
  }
  if (expect_reject >= 0.0 && last_ratio + 1e-6 < expect_reject) {
    std::cerr << "[offline_kitti] reject ratio " << last_ratio
              << " < expected " << expect_reject << "\n";
    return 1;
  }
  std::cout << "[offline_kitti] filter-only pass frames=" << n
            << " last_reject=" << last_ratio << "\n";
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  argus::common::InitLogging(argv[0]);
  std::string yaml = "config/livo_esikf.yaml";
  std::string kitti_root;
  int max_frames = 8;
  double expect_reject = -1.0;
  bool filter_only = false;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto need = [&](const char* name) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "missing value for " << name << "\n";
        std::exit(2);
      }
      return argv[++i];
    };
    if (a == "--kitti") {
      kitti_root = need("--kitti");
    } else if (a == "--config") {
      yaml = need("--config");
    } else if (a == "--max-frames") {
      max_frames = std::stoi(need("--max-frames"));
    } else if (a == "--expect-reject-ratio") {
      expect_reject = std::stod(need("--expect-reject-ratio"));
    } else if (a == "--filter-only") {
      filter_only = true;
    } else if (a == "--help" || a == "-h") {
      std::cout
          << "Usage: offline_kitti --kitti <dir> [--config yaml]\n"
          << "  --max-frames N              limit frames (default 8)\n"
          << "  --expect-reject-ratio R     fail if last reject ratio < R\n"
          << "  --filter-only               mask+project only (no LO/ESIKF)\n";
      return 0;
    } else if (kitti_root.empty() && !a.empty() && a[0] != '-') {
      kitti_root = a;
    }
  }

  if (kitti_root.empty()) {
    std::cerr << "[offline_kitti] need --kitti <subset_dir>\n";
    return 2;
  }

  argus::datasets::KittiDataset ds;
  if (!ds.Open(kitti_root)) {
    return 1;
  }

  if (filter_only) {
    return RunFilterOnly(ds, max_frames, expect_reject);
  }

  const auto loaded = argus::odometry::LoadLivoOdometryConfig(yaml);
  if (!loaded.ok) {
    std::cerr << "[offline_kitti] config error: " << loaded.error << "\n";
    return 1;
  }
  auto cfg = loaded.config;
  cfg.slam_mode = argus::frontend::ONLY_LO;
  cfg.enable_lidar = true;
  cfg.enable_imu = false;
  cfg.enable_image = false;
  cfg.enable_mapping = true;
  if (cfg.mapping.session_id.empty()) {
    cfg.mapping.session_id = "kitti";
  }
  if (cfg.mapping_output_dir.empty()) {
    cfg.mapping_output_dir = "argus_kitti_map_out";
  }
  cfg.dynamic_filter.lidar = argus::frontend::DynamicFilterType::kSemantic;
  cfg.dynamic_filter.semantic_backend = "mask_file";
  ApplyKittiCalib(&cfg, ds.calib());

  argus::odometry::LivoOdometry odom(cfg);
  auto* sem = dynamic_cast<argus::frontend::SemanticDynamicFilter*>(
      odom.lidar_dynamic_filter());
  if (sem == nullptr) {
    std::cerr << "[offline_kitti] semantic filter not active\n";
    return 1;
  }
  ConfigureFilterFromCalib(sem, ds.calib());

  int processed = 0;
  double last_ratio = 0.0;
  const int limit = std::min(max_frames, static_cast<int>(ds.size()));
  for (int i = 0; i < limit; ++i) {
    argus::datasets::KittiFrame fr;
    if (!ds.Read(static_cast<std::size_t>(i), &fr)) {
      break;
    }
    if (!fr.keep_mask.empty()) {
      sem->SetKeepMask(fr.keep_mask);
    } else if (!fr.image.empty()) {
      cv::Mat keep(fr.image.rows, fr.image.cols, CV_8UC1, cv::Scalar(1));
      sem->SetKeepMask(keep);
    }
    if (!fr.image.empty()) {
      argus::sensor::ImageData img;
      img.timestamp = fr.timestamp;
      img.frame_id = "camera2";
      img.encoding = "bgr8";
      img.image = fr.image;
      odom.AddImage(img);
    }
    odom.AddLidar(fr.cloud, fr.timestamp);
    argus::odometry::OdometryOutput out;
    if (!odom.Process(&out)) {
      continue;
    }
    last_ratio = odom.last_lidar_dynamic_ratio();
    ++processed;
    std::cout << "[offline_kitti] processed=" << processed
              << " t=" << fr.timestamp << " reject=" << last_ratio
              << " flag=" << static_cast<int>(out.ekf_phase) << "\n";
  }

  if (processed == 0) {
    std::cerr << "[offline_kitti] no package processed\n";
    return 1;
  }
  if (expect_reject >= 0.0 && last_ratio + 1e-6 < expect_reject) {
    std::cerr << "[offline_kitti] reject ratio " << last_ratio
              << " < expected " << expect_reject << "\n";
    return 1;
  }

  std::filesystem::remove_all(cfg.mapping_output_dir);
  if (!odom.SaveBuiltMap()) {
    std::cerr << "[offline_kitti] SaveBuiltMap failed\n";
    return 1;
  }

  std::cout << "[offline_kitti] e2e pass processed=" << processed
            << " last_reject=" << last_ratio
            << " map=" << cfg.mapping_output_dir << "\n";
  return 0;
}
