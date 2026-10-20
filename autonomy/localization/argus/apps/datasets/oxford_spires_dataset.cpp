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

#include "apps/datasets/oxford_spires_dataset.hpp"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <system_error>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "apps/datasets/dataset_common.hpp"
#include "apps/datasets/pcd_io.hpp"

namespace argus {
namespace datasets {
namespace {

namespace fs = std::filesystem;

bool ParseTCamLidarYaml(const std::string& path, const std::string& camera,
                        Eigen::Matrix4d* T) {
  if (T == nullptr) {
    return false;
  }
  std::ifstream ifs(path);
  if (!ifs) {
    return false;
  }
  const std::string cam_key = camera + ":";
  bool in_cam = false;
  bool in_T = false;
  std::vector<double> vals;
  std::string line;
  while (std::getline(ifs, line)) {
    // Strip leading spaces for key match.
    std::size_t i = 0;
    while (i < line.size() && (line[i] == ' ' || line[i] == '\t')) {
      ++i;
    }
    const std::string trimmed = line.substr(i);
    if (!in_cam) {
      if (trimmed == cam_key || trimmed.rfind(cam_key, 0) == 0) {
        in_cam = true;
      }
      continue;
    }
    // Next top-level camN: ends this block.
    if (!trimmed.empty() && trimmed[0] != ' ' && trimmed.find(':') != std::string::npos &&
        trimmed.rfind("cam", 0) == 0 && trimmed != cam_key) {
      break;
    }
    if (trimmed.rfind("T_cam_lidar:", 0) == 0) {
      in_T = true;
      // DiffCal often puts the first matrix row on the same line — keep parsing.
    }
    if (!in_T) {
      continue;
    }
    // Collect floats from nested list lines (and same-line T_cam_lidar: [[...]).
    for (std::size_t p = 0; p < line.size();) {
      if (line[p] == '-' || line[p] == '.' ||
          (line[p] >= '0' && line[p] <= '9')) {
        char* end = nullptr;
        const double v = std::strtod(line.c_str() + p, &end);
        if (end != line.c_str() + p) {
          vals.push_back(v);
          p = static_cast<std::size_t>(end - line.c_str());
          continue;
        }
      }
      ++p;
    }
    if (vals.size() >= 16) {
      break;
    }
    // Blank / next key at same indent as T_cam_lidar ends matrix.
    if (in_T && trimmed.rfind("C_", 0) == 0) {
      break;
    }
  }
  if (vals.size() < 16) {
    return false;
  }
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      (*T)(r, c) = vals[static_cast<std::size_t>(r * 4 + c)];
    }
  }
  return true;
}

}  // namespace

bool OxfordSpiresDataset::LoadCalibration(const std::string& calibration_dir,
                                          const std::string& camera,
                                          OxfordSpiresCalib* out) {
  if (out == nullptr || calibration_dir.empty()) {
    return false;
  }
  const fs::path cam_yaml = fs::path(calibration_dir) / (camera + ".yaml");
  if (fs::exists(cam_yaml)) {
    try {
      cv::FileStorage fs(cam_yaml.string(), cv::FileStorage::READ);
      if (fs.isOpened()) {
        // Kalibr / DiffCal: camera_matrix: {rows, cols, data:[...]} — OpenCV 5
        // rejects `fs["camera_matrix"] >> Mat` on that layout.
        auto read_data_seq = [](const cv::FileNode& node,
                                std::vector<double>* vals) {
          vals->clear();
          cv::FileNode data = node["data"];
          if (!data.isSeq()) {
            data = node;
          }
          if (!data.isSeq()) {
            return;
          }
          for (auto it = data.begin(); it != data.end(); ++it) {
            double v = 0.0;
            (*it) >> v;
            vals->push_back(v);
          }
        };
        std::vector<double> kvals;
        read_data_seq(fs["camera_matrix"], &kvals);
        if (kvals.size() >= 9) {
          for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
              out->K(r, c) = kvals[static_cast<std::size_t>(r * 3 + c)];
            }
          }
          out->has_intrinsics = true;
        }
        std::vector<double> dvals;
        read_data_seq(fs["distortion_coefficients"], &dvals);
        for (int i = 0; i < std::min(4, static_cast<int>(dvals.size())); ++i) {
          out->D(i) = dvals[static_cast<std::size_t>(i)];
        }
        if (!fs["distortion_model"].empty()) {
          fs["distortion_model"] >> out->distortion_model;
        } else {
          out->distortion_model = "equidistant";
        }
        if (!fs["image_width"].empty()) {
          fs["image_width"] >> out->image_width;
        }
        if (!fs["image_height"].empty()) {
          fs["image_height"] >> out->image_height;
        }
      }
    } catch (const cv::Exception& e) {
      std::cerr << "[OxfordSpires] calib parse failed: " << e.what() << "\n";
    }
  }

  const fs::path ext_yaml = fs::path(calibration_dir) / "cam-lidar-imu.yaml";
  if (fs::exists(ext_yaml)) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    if (ParseTCamLidarYaml(ext_yaml.string(), camera, &T)) {
      out->T_cam_lidar = T;
      out->has_extrinsics = true;
    }
  }
  return out->has_intrinsics || out->has_extrinsics;
}

bool OxfordSpiresDataset::ResolveDirs(const std::string& root) {
  root_ = root;
  const fs::path r(root);
  if (fs::is_directory(r / "lidar-clouds") &&
      (fs::is_directory(r / "cam0") || fs::is_directory(r / options_.camera))) {
    raw_dir_ = r.string();
  } else if (fs::is_directory(r / "raw" / "lidar-clouds")) {
    raw_dir_ = (r / "raw").string();
    root_ = r.string();
  } else {
    return false;
  }

  const fs::path raw(raw_dir_);
  if (options_.prefer_undistorted) {
    const fs::path undist =
        fs::path(root_) / "processed" / "vilens-slam" / "undist-clouds";
    if (detail::CountExt(undist, ".pcd") > 0) {
      lidar_dir_ = undist.string();
    }
  }
  if (lidar_dir_.empty()) {
    lidar_dir_ = (raw / "lidar-clouds").string();
  }

  image_dir_ = detail::ResolveDataDir(raw / options_.camera).string();
  return detail::CountExt(lidar_dir_, ".pcd") > 0 &&
         fs::is_directory(image_dir_);
}

bool OxfordSpiresDataset::IndexSensors() {
  lidar_stems_ = detail::ListStems(lidar_dir_, ".pcd");
  if (lidar_stems_.empty()) {
    return false;
  }
  lidar_times_ = detail::StemsToTimes(lidar_stems_);

  image_stems_.clear();
  for (const char* ext : {".jpg", ".jpeg", ".png"}) {
    auto stems = detail::ListStems(image_dir_, ext);
    if (!stems.empty()) {
      image_stems_ = std::move(stems);
      break;
    }
  }
  image_times_ = detail::StemsToTimes(image_stems_);
  return !image_stems_.empty();
}

bool OxfordSpiresDataset::LoadCalibIfAvailable() {
  std::vector<fs::path> candidates;
  if (!options_.calibration_dir.empty()) {
    candidates.emplace_back(options_.calibration_dir);
  }
  candidates.push_back(fs::path(root_) / "calibration");
  candidates.push_back(fs::path(root_) / ".." / "calibration");
  candidates.push_back(fs::path(root_) / ".." / ".." / "calibration");
  candidates.push_back(fs::path(raw_dir_) / ".." / ".." / "calibration");
  for (const auto& c : candidates) {
    std::error_code ec;
    const fs::path canon = fs::weakly_canonical(c, ec);
    if (ec || !fs::is_directory(canon)) {
      continue;
    }
    if (LoadCalibration(canon.string(), options_.camera, &calib_)) {
      return true;
    }
  }
  return false;
}

bool OxfordSpiresDataset::Open(const std::string& root,
                               const OxfordSpiresOptions& options) {
  options_ = options;
  lidar_stems_.clear();
  lidar_times_.clear();
  image_stems_.clear();
  image_times_.clear();
  lidar_dir_.clear();
  image_dir_.clear();
  calib_ = OxfordSpiresCalib{};

  if (!ResolveDirs(root)) {
    std::cerr << "[oxford_spires] unsupported layout (need raw/lidar-clouds + "
              << options_.camera << "): " << root << std::endl;
    return false;
  }
  if (!IndexSensors()) {
    std::cerr << "[oxford_spires] no lidar/image stems under " << raw_dir_
              << std::endl;
    return false;
  }
  LoadCalibIfAvailable();
  std::cout << "[oxford_spires] opened raw=" << raw_dir_
            << " lidar=" << lidar_stems_.size()
            << " images=" << image_stems_.size() << " cam=" << options_.camera
            << " K=" << calib_.has_intrinsics
            << " T_cl=" << calib_.has_extrinsics << std::endl;
  return true;
}

bool OxfordSpiresDataset::Read(std::size_t index, OxfordSpiresFrame* out,
                               bool load_image) const {
  if (out == nullptr || index >= lidar_stems_.size()) {
    return false;
  }
  out->index = static_cast<int>(index);
  out->timestamp = lidar_times_[index];
  out->camera_id = options_.camera;
  out->image_timestamp = 0.0;
  out->image.release();

  const fs::path cloud_path =
      detail::FindCloudFile(lidar_dir_, lidar_stems_[index]);
  if (cloud_path.empty()) {
    return false;
  }
  out->cloud = LoadPcdXYZI(cloud_path.string());
  if (!out->cloud || out->cloud->empty()) {
    return false;
  }

  if (load_image) {
    const int j = detail::NearestIndex(image_times_, out->timestamp,
                                       options_.max_image_dt_sec);
    if (j >= 0) {
      out->image_timestamp = image_times_[static_cast<std::size_t>(j)];
      const fs::path img =
          detail::FindImageFile(image_dir_, image_stems_[static_cast<std::size_t>(j)]);
      if (!img.empty()) {
        out->image = cv::imread(img.string(), cv::IMREAD_COLOR);
      }
    }
  }
  return true;
}

}  // namespace datasets
}  // namespace argus
