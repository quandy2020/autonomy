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

#include "apps/datasets/kitti_dataset.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include <opencv2/imgcodecs.hpp>

namespace argus {
namespace datasets {
namespace {

namespace fs = std::filesystem;

bool ParseMatrixLine(const std::string& line, const char* key,
                     Eigen::Matrix<double, 3, 4>* M) {
  const std::string prefix = std::string(key) + ":";
  if (line.compare(0, prefix.size(), prefix) != 0) {
    return false;
  }
  std::istringstream iss(line.substr(prefix.size()));
  for (int i = 0; i < 12; ++i) {
    double v = 0;
    if (!(iss >> v)) {
      return false;
    }
    (*M)(i / 4, i % 4) = v;
  }
  return true;
}

bool ParseMatrixLineRaw(const std::string& line, const char* key,
                        Eigen::Matrix<double, 3, 4>* M) {
  // raw: "P_rect_02: ..." or "R: ..." / "T: ..."
  return ParseMatrixLine(line, key, M);
}

std::vector<std::string> ListStems(const fs::path& dir, const char* ext) {
  std::vector<std::string> stems;
  if (!fs::is_directory(dir)) {
    return stems;
  }
  for (const auto& e : fs::directory_iterator(dir)) {
    if (e.path().extension() == ext) {
      stems.push_back(e.path().stem().string());
    }
  }
  std::sort(stems.begin(), stems.end());
  return stems;
}

}  // namespace

bool KittiDataset::LoadCalibFile(const std::string& path, KittiCalib* out) {
  if (out == nullptr) {
    return false;
  }
  std::ifstream ifs(path);
  if (!ifs) {
    return false;
  }
  std::string line;
  while (std::getline(ifs, line)) {
    if (ParseMatrixLine(line, "P2", &out->P2)) {
      out->has_P2 = true;
    } else if (ParseMatrixLine(line, "Tr", &out->Tr)) {
      out->has_Tr = true;
    } else if (ParseMatrixLine(line, "Tr_velo_to_cam", &out->Tr)) {
      out->has_Tr = true;
    }
  }
  return out->has_P2 || out->has_Tr;
}

bool KittiDataset::LoadRawCalibDir(const std::string& date_dir, KittiCalib* out) {
  if (out == nullptr) {
    return false;
  }
  const fs::path cam = fs::path(date_dir) / "calib_cam_to_cam.txt";
  const fs::path velo = fs::path(date_dir) / "calib_velo_to_cam.txt";
  if (!fs::exists(cam) || !fs::exists(velo)) {
    return false;
  }
  {
    std::ifstream ifs(cam.string());
    std::string line;
    while (std::getline(ifs, line)) {
      if (ParseMatrixLineRaw(line, "P_rect_02", &out->P2) ||
          ParseMatrixLineRaw(line, "P2", &out->P2)) {
        out->has_P2 = true;
      }
    }
  }
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d T = Eigen::Vector3d::Zero();
  bool has_R = false;
  bool has_T = false;
  {
    std::ifstream ifs(velo.string());
    std::string line;
    while (std::getline(ifs, line)) {
      if (line.compare(0, 2, "R:") == 0) {
        std::istringstream iss(line.substr(2));
        for (int i = 0; i < 9; ++i) {
          iss >> R(i / 3, i % 3);
        }
        has_R = true;
      } else if (line.compare(0, 2, "T:") == 0) {
        std::istringstream iss(line.substr(2));
        iss >> T(0) >> T(1) >> T(2);
        has_T = true;
      }
    }
  }
  if (has_R && has_T) {
    out->Tr.setZero();
    out->Tr.block<3, 3>(0, 0) = R;
    out->Tr.block<3, 1>(0, 3) = T;
    out->has_Tr = true;
  }
  return out->has_P2 || out->has_Tr;
}

sensor::PointCloudXYZI::Ptr KittiDataset::LoadVelodyneBin(const std::string& path) {
  auto cloud = std::make_shared<sensor::PointCloudXYZI>();
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs) {
    return cloud;
  }
  ifs.seekg(0, std::ios::end);
  const auto nbytes = static_cast<std::size_t>(ifs.tellg());
  ifs.seekg(0, std::ios::beg);
  if (nbytes < 16 || nbytes % 16 != 0) {
    return cloud;
  }
  const std::size_t npts = nbytes / 16;
  std::vector<float> buf(npts * 4);
  ifs.read(reinterpret_cast<char*>(buf.data()),
           static_cast<std::streamsize>(nbytes));
  cloud->reserve(npts);
  for (std::size_t i = 0; i < npts; ++i) {
    sensor::PointType p;
    p.x = buf[4 * i + 0];
    p.y = buf[4 * i + 1];
    p.z = buf[4 * i + 2];
    p.intensity = buf[4 * i + 3];
    // Relative time within scan (ms) for LO undistort; KITTI bins have none.
    p.curvature = static_cast<float>(
        (static_cast<double>(i) / static_cast<double>(npts)) * 100.0);
    cloud->push_back(p);
  }
  return cloud;
}

bool KittiDataset::OpenOdometry(const std::string& root) {
  const fs::path velo_dir = fs::path(root) / "velodyne";
  if (!fs::is_directory(velo_dir)) {
    return false;
  }
  layout_ = KittiLayout::kOdometry;
  velo_rel_ = "velodyne";
  image_rel_ = "image_2";
  frame_stems_ = ListStems(velo_dir, ".bin");
  if (frame_stems_.empty()) {
    return false;
  }

  const fs::path calib_path = fs::path(root) / "calib.txt";
  if (fs::exists(calib_path)) {
    LoadCalibFile(calib_path.string(), &calib_);
  }

  timestamps_.assign(frame_stems_.size(), 0.0);
  const fs::path times_path = fs::path(root) / "times.txt";
  if (fs::exists(times_path)) {
    std::ifstream ifs(times_path);
    std::string line;
    std::size_t i = 0;
    while (std::getline(ifs, line) && i < timestamps_.size()) {
      if (!line.empty()) {
        timestamps_[i++] = std::stod(line);
      }
    }
    for (; i < timestamps_.size(); ++i) {
      timestamps_[i] = 0.1 * static_cast<double>(i + 1);
    }
  } else {
    for (std::size_t i = 0; i < timestamps_.size(); ++i) {
      timestamps_[i] = 0.1 * static_cast<double>(i + 1);
    }
  }
  return true;
}

bool KittiDataset::OpenRawSync(const std::string& root) {
  const fs::path velo_dir = fs::path(root) / "velodyne_points" / "data";
  const fs::path img_dir = fs::path(root) / "image_02" / "data";
  if (!fs::is_directory(velo_dir)) {
    return false;
  }
  layout_ = KittiLayout::kRawSync;
  velo_rel_ = "velodyne_points/data";
  image_rel_ = fs::is_directory(img_dir) ? "image_02/data" : "image_2/data";
  frame_stems_ = ListStems(velo_dir, ".bin");
  if (frame_stems_.empty()) {
    return false;
  }

  // Calib lives in parent date directory.
  const fs::path date_dir = fs::path(root).parent_path();
  LoadRawCalibDir(date_dir.string(), &calib_);

  timestamps_.assign(frame_stems_.size(), 0.0);
  const fs::path ts = fs::path(root) / "velodyne_points" / "timestamps.txt";
  if (fs::exists(ts)) {
    std::ifstream ifs(ts.string());
    std::string line;
    std::size_t i = 0;
    while (std::getline(ifs, line) && i < timestamps_.size()) {
      // "2011-09-26 13:02:XX.YYYYYYYYY" → use index*0.1 if parse hard
      timestamps_[i] = 0.1 * static_cast<double>(i + 1);
      ++i;
    }
  } else {
    for (std::size_t i = 0; i < timestamps_.size(); ++i) {
      timestamps_[i] = 0.1 * static_cast<double>(i + 1);
    }
  }
  return true;
}

bool KittiDataset::Open(const std::string& root) {
  root_ = root;
  timestamps_.clear();
  frame_stems_.clear();
  calib_ = KittiCalib{};
  layout_ = KittiLayout::kUnknown;

  if (OpenOdometry(root) || OpenRawSync(root)) {
    std::cout << "[kitti] opened " << root << " layout="
              << (layout_ == KittiLayout::kOdometry ? "odometry" : "raw_sync")
              << " frames=" << frame_stems_.size() << " P2=" << calib_.has_P2
              << " Tr=" << calib_.has_Tr << std::endl;
    return true;
  }
  std::cerr << "[kitti] unsupported layout (need odometry seq or raw sync): "
            << root << std::endl;
  return false;
}

std::string KittiDataset::VelodynePath(std::size_t index) const {
  return (fs::path(root_) / velo_rel_ / (frame_stems_[index] + ".bin")).string();
}

std::string KittiDataset::ImagePath(std::size_t index) const {
  const fs::path base = fs::path(root_) / image_rel_ / frame_stems_[index];
  for (const char* ext : {".png", ".jpg", ".ppm"}) {
    const fs::path p = fs::path(base.string() + ext);
    if (fs::exists(p)) {
      return p.string();
    }
  }
  return {};
}

bool KittiDataset::Read(std::size_t index, KittiFrame* out, bool load_image,
                        bool load_mask) const {
  if (out == nullptr || index >= frame_stems_.size()) {
    return false;
  }
  out->index = static_cast<int>(index);
  out->timestamp = timestamps_[index];
  out->cloud = LoadVelodyneBin(VelodynePath(index));
  if (!out->cloud || out->cloud->empty()) {
    return false;
  }

  if (load_image) {
    const std::string img_path = ImagePath(index);
    if (!img_path.empty()) {
      out->image = cv::imread(img_path, cv::IMREAD_COLOR);
    }
  }
  if (load_mask && layout_ == KittiLayout::kOdometry) {
    for (const char* ext : {".png", ".pgm"}) {
      const fs::path mask_path =
          fs::path(root_) / "masks" / (frame_stems_[index] + ext);
      if (!fs::exists(mask_path)) {
        continue;
      }
      out->keep_mask = cv::imread(mask_path.string(), cv::IMREAD_GRAYSCALE);
      break;
    }
  }
  return true;
}

}  // namespace datasets
}  // namespace argus
