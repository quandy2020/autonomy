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
 * @file kitti_dataset.hpp
 * @brief App-side C++ reader for real KITTI odometry / raw synced layouts.
 *
 * Lives under apps/datasets/ (not libargus).
 *
 * Odometry sequence:
 *   <seq>/calib.txt times.txt velodyne/000000.bin image_2/000000.png
 *
 * Raw synced drive:
 *   <date>/<date>_drive_XXXX_sync/velodyne_points/data/…
 *   <date>/calib_cam_to_cam.txt + calib_velo_to_cam.txt
 */

#ifndef ARGUS_APPS_DATASETS_KITTI_DATASET_HPP_
#define ARGUS_APPS_DATASETS_KITTI_DATASET_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "argus/sensor/point_types.hpp"

namespace argus {
namespace datasets {

/** @brief Detected on-disk KITTI dataset layout. */
enum class KittiLayout {
  kUnknown = 0,
  kOdometry,
  kRawSync,
};

/** @brief Projection and velodyne-to-camera calibration from KITTI calib files. */
struct KittiCalib {
  Eigen::Matrix<double, 3, 4> P2 = Eigen::Matrix<double, 3, 4>::Zero();
  Eigen::Matrix<double, 3, 4> Tr = Eigen::Matrix<double, 3, 4>::Zero();
  bool has_P2 = false;
  bool has_Tr = false;

  double fx() const { return P2(0, 0); }
  double fy() const { return P2(1, 1); }
  double cx() const { return P2(0, 2); }
  double cy() const { return P2(1, 2); }

  Eigen::Matrix3d R_cl() const {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    if (has_Tr) {
      R = Tr.block<3, 3>(0, 0);
    }
    return R;
  }
  Eigen::Vector3d t_cl() const {
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    if (has_Tr) {
      t = Tr.block<3, 1>(0, 3);
    }
    return t;
  }
};

/** @brief One indexed KITTI frame (LiDAR, optional image, optional mask). */
struct KittiFrame {
  int index = 0;
  double timestamp = 0.0;
  sensor::PointCloudXYZI::Ptr cloud;
  /** @brief BGR image from image_2 / image_02 (empty if missing). */
  cv::Mat image;
  cv::Mat keep_mask;
};

/**
 * @brief Sequential reader for KITTI odometry sequences or raw synced drives.
 */
class KittiDataset {
 public:
  /**
   * @brief Open a sequence root and index frames.
   * @param root Odometry `<seq>/` or raw sync drive folder.
   * @return True if layout was recognized and at least one frame was indexed.
   */
  bool Open(const std::string& root);

  /** @return Number of indexed frames. */
  std::size_t size() const { return frame_stems_.size(); }

  /** @return True if no frames were indexed. */
  bool empty() const { return frame_stems_.empty(); }

  /** @return Dataset root path passed to Open(). */
  const std::string& root() const { return root_; }

  /** @return Loaded calibration (P2 / Tr when available). */
  const KittiCalib& calib() const { return calib_; }

  /** @return Detected KITTI layout variant. */
  KittiLayout layout() const { return layout_; }

  /**
   * @brief Load velodyne (and optionally image / semantic mask) for one index.
   * @param index Frame index in [0, size()).
   * @param[out] out Populated frame; cloud/image cleared on failure.
   * @param load_image When false, skip image I/O.
   * @param load_mask When false, skip keep_mask I/O.
   * @return True on success.
   */
  bool Read(std::size_t index, KittiFrame* out, bool load_image = true,
            bool load_mask = true) const;

  /**
   * @brief Load a KITTI velodyne .bin (x,y,z,reflectance).
   * @param path Path to the .bin file.
   * @return Point cloud; null on I/O or format error.
   */
  static sensor::PointCloudXYZI::Ptr LoadVelodyneBin(const std::string& path);

  /**
   * @brief Parse odometry-style calib.txt (P2, Tr).
   * @param path Path to calib.txt.
   * @param[out] out Filled calibration flags and matrices.
   * @return True if the file was read and at least one matrix was found.
   */
  static bool LoadCalibFile(const std::string& path, KittiCalib* out);

  /**
   * @brief Parse raw-date calib_cam_to_cam + calib_velo_to_cam.
   * @param date_dir KITTI raw date directory.
   * @param[out] out Filled calibration.
   * @return True on successful load.
   */
  static bool LoadRawCalibDir(const std::string& date_dir, KittiCalib* out);

 private:
  bool OpenOdometry(const std::string& root);
  bool OpenRawSync(const std::string& root);
  std::string VelodynePath(std::size_t index) const;
  std::string ImagePath(std::size_t index) const;

  std::string root_;
  KittiLayout layout_ = KittiLayout::kUnknown;
  std::vector<double> timestamps_;
  std::vector<std::string> frame_stems_;
  KittiCalib calib_;
  std::string velo_rel_ = "velodyne";
  std::string image_rel_ = "image_2";
};

}  // namespace datasets
}  // namespace argus

#endif  // ARGUS_APPS_DATASETS_KITTI_DATASET_HPP_
