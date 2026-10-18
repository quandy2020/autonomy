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

#include "apps/datasets/newer_college_dataset.hpp"

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

fs::path PickLidarDir(const fs::path& root) {
  const char* kCandidates[] = {
      "ouster_points", "ouster", "lidar", "os1_cloud", "os_cloud",
      "os1_cloud_node", "os_cloud_node", "lidar-clouds", "point_clouds",
  };
  fs::path best;
  std::size_t best_n = 0;
  for (const char* name : kCandidates) {
    const fs::path d = detail::ResolveDataDir(root / name);
    const std::size_t n = detail::CountExt(d, ".pcd");
    if (n > best_n) {
      best_n = n;
      best = d;
    }
  }
  // Fallback: any immediate child dir with many PCDs.
  if (best_n == 0 && fs::is_directory(root)) {
    for (const auto& e : fs::directory_iterator(root)) {
      if (!e.is_directory()) {
        continue;
      }
      const fs::path d = detail::ResolveDataDir(e.path());
      const std::size_t n = detail::CountExt(d, ".pcd");
      if (n > best_n) {
        best_n = n;
        best = d;
      }
    }
  }
  return best_n > 0 ? best : fs::path{};
}

fs::path PickCameraDir(const fs::path& root, const std::string& preferred,
                       NewerCollegeLayout* layout) {
  auto try_cam = [&](const std::string& name) -> fs::path {
    const fs::path d = detail::ResolveDataDir(root / name);
    if (detail::CountExt(d, ".png") + detail::CountExt(d, ".jpg") > 0) {
      return d;
    }
    return {};
  };

  if (!preferred.empty()) {
    const fs::path d = try_cam(preferred);
    if (!d.empty()) {
      return d;
    }
  }

  // Prefer requested / common camera folders; layout refined in DetectLayout.
  for (const char* name :
       {"cam0", "cam1", "cam3", "cam4", "infra1", "camera_left", "left"}) {
    const fs::path d = try_cam(name);
    if (!d.empty()) {
      if (layout != nullptr) {
        *layout = NewerCollegeLayout::kStereo;
      }
      return d;
    }
  }
  return {};
}

bool LoadOpenCvCameraYaml(const fs::path& path, NewerCollegeCalib* out) {
  cv::FileStorage fs(path.string(), cv::FileStorage::READ);
  if (!fs.isOpened()) {
    return false;
  }
  cv::Mat K;
  if (!fs["camera_matrix"].empty()) {
    fs["camera_matrix"] >> K;
  } else if (!fs["intrinsics"].empty()) {
    // Kalibr camchain style: intrinsics: [fx, fy, cx, cy]
    std::vector<double> intr;
    fs["intrinsics"] >> intr;
    if (intr.size() >= 4) {
      out->K.setIdentity();
      out->K(0, 0) = intr[0];
      out->K(1, 1) = intr[1];
      out->K(0, 2) = intr[2];
      out->K(1, 2) = intr[3];
      out->has_intrinsics = true;
    }
  }
  if (K.rows == 3 && K.cols == 3) {
    cv::Mat K64;
    K.convertTo(K64, CV_64F);
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        out->K(r, c) = K64.at<double>(r, c);
      }
    }
    out->has_intrinsics = true;
  }
  cv::Mat D;
  if (!fs["distortion_coefficients"].empty()) {
    fs["distortion_coefficients"] >> D;
  } else if (!fs["distortion_coeffs"].empty()) {
    fs["distortion_coeffs"] >> D;
  }
  if (!D.empty()) {
    cv::Mat D64;
    D.convertTo(D64, CV_64F);
    const int n = D64.cols * D64.rows;
    out->D = Eigen::VectorXd::Zero(std::max(n, 4));
    for (int i = 0; i < n; ++i) {
      out->D(i) = D64.at<double>(i);
    }
  }
  if (!fs["distortion_model"].empty()) {
    fs["distortion_model"] >> out->distortion_model;
  }
  if (!fs["image_width"].empty()) {
    fs["image_width"] >> out->image_width;
  }
  if (!fs["image_height"].empty()) {
    fs["image_height"] >> out->image_height;
  }
  // Optional 4x4 under common keys.
  cv::Mat T;
  if (!fs["T_cam_lidar"].empty()) {
    fs["T_cam_lidar"] >> T;
  } else if (!fs["T_cn_cnm1"].empty()) {
    fs["T_cn_cnm1"] >> T;
  }
  if (T.rows == 4 && T.cols == 4) {
    cv::Mat T64;
    T.convertTo(T64, CV_64F);
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        out->T_cam_lidar(r, c) = T64.at<double>(r, c);
      }
    }
    out->has_extrinsics = true;
  }
  return out->has_intrinsics || out->has_extrinsics;
}

}  // namespace

bool NewerCollegeDataset::LoadCalibration(const std::string& calibration_dir,
                                          const std::string& camera,
                                          NewerCollegeCalib* out) {
  if (out == nullptr || calibration_dir.empty()) {
    return false;
  }
  const fs::path dir(calibration_dir);
  for (const char* suffix : {".yaml", ".yml"}) {
    const fs::path p = dir / (camera + suffix);
    if (fs::exists(p) && LoadOpenCvCameraYaml(p, out)) {
      return true;
    }
  }
  if (fs::is_directory(dir)) {
    for (const auto& e : fs::directory_iterator(dir)) {
      if (!e.is_regular_file()) {
        continue;
      }
      const std::string name = e.path().filename().string();
      if (name.find("camchain") != std::string::npos ||
          (!camera.empty() && name.find(camera) != std::string::npos)) {
        if (LoadOpenCvCameraYaml(e.path(), out)) {
          return true;
        }
      }
    }
  }
  return out->has_intrinsics || out->has_extrinsics;
}

bool NewerCollegeDataset::DetectLayout() {
  layout_ = NewerCollegeLayout::kUnknown;
  lidar_dir_ = PickLidarDir(root_).string();
  if (lidar_dir_.empty()) {
    return false;
  }

  NewerCollegeLayout detected = NewerCollegeLayout::kUnknown;
  const fs::path cam_dir =
      PickCameraDir(root_, options_.camera, &detected);
  if (cam_dir.empty()) {
    return false;
  }
  image_dir_ = cam_dir.string();
  {
    fs::path cam_folder = cam_dir;
    if (cam_folder.filename() == "data") {
      cam_folder = cam_folder.parent_path();
    }
    camera_id_ = cam_folder.filename().string();
  }
  if (!options_.camera.empty()) {
    camera_id_ = options_.camera;
  }
  layout_ = detected != NewerCollegeLayout::kUnknown
                ? detected
                : NewerCollegeLayout::kStereo;

  // Refine multi-cam if sibling cam3/cam4 exist.
  if (fs::is_directory(fs::path(root_) / "cam3") ||
      fs::is_directory(fs::path(root_) / "cam4")) {
    layout_ = NewerCollegeLayout::kMultiCam;
  }
  return true;
}

bool NewerCollegeDataset::IndexSensors() {
  lidar_stems_ = detail::ListStems(lidar_dir_, ".pcd");
  if (lidar_stems_.empty()) {
    return false;
  }
  lidar_times_ = detail::StemsToTimes(lidar_stems_);

  image_stems_.clear();
  for (const char* ext : {".png", ".jpg", ".jpeg"}) {
    auto stems = detail::ListStems(image_dir_, ext);
    if (!stems.empty()) {
      image_stems_ = std::move(stems);
      break;
    }
  }
  image_times_ = detail::StemsToTimes(image_stems_);
  return !image_stems_.empty();
}

bool NewerCollegeDataset::LoadTimeOffsets() {
  time_offsets_.clear();
  const char* names[] = {"time_offset.csv", "time_offsets.csv",
                         "lidar_camera_time_offset.csv"};
  for (const char* name : names) {
    const fs::path p = fs::path(root_) / name;
    if (!fs::exists(p)) {
      continue;
    }
    std::ifstream ifs(p.string());
    std::string line;
    bool header = true;
    while (std::getline(ifs, line)) {
      if (line.empty() || line[0] == '#') {
        continue;
      }
      if (header && (line.find("sec") != std::string::npos ||
                     line.find("time_offset") != std::string::npos)) {
        header = false;
        continue;
      }
      header = false;
      // sec,nsec,time_offset_nsec  OR  t_sec,offset_sec
      for (char& c : line) {
        if (c == ',') {
          c = ' ';
        }
      }
      std::istringstream iss(line);
      double a = 0, b = 0, c = 0;
      if (!(iss >> a >> b)) {
        continue;
      }
      if (iss >> c) {
        // sec, nsec, offset_nsec
        const double t = a + b * 1e-9;
        time_offsets_.emplace_back(t, c * 1e-9);
      } else {
        time_offsets_.emplace_back(a, b);
      }
    }
    return !time_offsets_.empty();
  }
  return false;
}

bool NewerCollegeDataset::LoadCalibIfAvailable() {
  std::vector<fs::path> candidates;
  if (!options_.calibration_dir.empty()) {
    candidates.emplace_back(options_.calibration_dir);
  }
  candidates.push_back(fs::path(root_) / "calibration");
  candidates.push_back(fs::path(root_) / ".." / "calibration");
  for (const auto& c : candidates) {
    std::error_code ec;
    const fs::path canon = fs::weakly_canonical(c, ec);
    if (ec || !fs::is_directory(canon)) {
      continue;
    }
    if (LoadCalibration(canon.string(), camera_id_, &calib_)) {
      return true;
    }
  }
  return false;
}

bool NewerCollegeDataset::Open(const std::string& root,
                               const NewerCollegeOptions& options) {
  options_ = options;
  root_ = root;
  layout_ = NewerCollegeLayout::kUnknown;
  lidar_dir_.clear();
  image_dir_.clear();
  camera_id_.clear();
  lidar_stems_.clear();
  lidar_times_.clear();
  image_stems_.clear();
  image_times_.clear();
  time_offsets_.clear();
  calib_ = NewerCollegeCalib{};

  if (!DetectLayout()) {
    std::cerr << "[newer_college] unsupported layout (need PCD folder + "
                 "cam0|infra1|...): "
              << root << std::endl;
    return false;
  }
  if (!IndexSensors()) {
    std::cerr << "[newer_college] empty lidar/image lists under " << root
              << std::endl;
    return false;
  }
  LoadTimeOffsets();
  LoadCalibIfAvailable();
  std::cout << "[newer_college] opened " << root
            << " layout="
            << (layout_ == NewerCollegeLayout::kMultiCam ? "multi_cam"
                                                         : "stereo")
            << " lidar=" << lidar_stems_.size()
            << " images=" << image_stems_.size() << " cam=" << camera_id_
            << " K=" << calib_.has_intrinsics << std::endl;
  return true;
}

bool NewerCollegeDataset::Read(std::size_t index, NewerCollegeFrame* out,
                               bool load_image) const {
  if (out == nullptr || index >= lidar_stems_.size()) {
    return false;
  }
  out->index = static_cast<int>(index);
  out->timestamp = lidar_times_[index];
  out->camera_id = camera_id_;
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
    double t_query = out->timestamp;
    // Apply nearest time-offset sample if present.
    if (!time_offsets_.empty()) {
      // Find last offset with t <= lidar time (piecewise constant).
      double off = time_offsets_.front().second;
      for (const auto& kv : time_offsets_) {
        if (kv.first <= out->timestamp) {
          off = kv.second;
        } else {
          break;
        }
      }
      // Docs: time_camera_corrected = time_camera - time_offset
      // → match lidar to (image - offset) ⇔ query image with lidar+offset.
      t_query = out->timestamp + off;
    }
    const int j =
        detail::NearestIndex(image_times_, t_query, options_.max_image_dt_sec);
    if (j >= 0) {
      out->image_timestamp = image_times_[static_cast<std::size_t>(j)];
      const fs::path img = detail::FindImageFile(
          image_dir_, image_stems_[static_cast<std::size_t>(j)]);
      if (!img.empty()) {
        out->image = cv::imread(img.string(), cv::IMREAD_COLOR);
      }
    }
  }
  return true;
}

}  // namespace datasets
}  // namespace argus
