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
 * @file oxford_spires_dataset.hpp
 * @brief App-side reader for Oxford Spires Dataset raw sequences.
 *
 * Official layout (https://dynamic.robots.ox.ac.uk/datasets/oxford-spires/):
 *   sequences/<name>/raw/
 *     cam0|cam1|cam2/  (20 Hz fisheye jpg, stem = timestamp)
 *     lidar-clouds/    (10 Hz Hesai QT64 pcd, sweep start time)
 *     imu.csv
 *   calibration/
 *     cam0.yaml … cam-lidar-imu.yaml
 *
 * Open() accepts sequence root, .../raw, or a folder that already contains
 * lidar-clouds/ plus cam0|cam1|cam2/.
 */

#ifndef ARGUS_APPS_DATASETS_OXFORD_SPIRES_DATASET_HPP_
#define ARGUS_APPS_DATASETS_OXFORD_SPIRES_DATASET_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "argus/sensor/point_types.hpp"
#include "oxford_spires_options.hpp"

namespace argus {
namespace datasets {

/** @brief Camera intrinsics and cam–LiDAR extrinsics for Oxford Spires. */
struct OxfordSpiresCalib {
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
  Eigen::Vector4d D = Eigen::Vector4d::Zero();  // equidistant k1..k4
  Eigen::Matrix4d T_cam_lidar = Eigen::Matrix4d::Identity();
  int image_width = 0;
  int image_height = 0;
  std::string distortion_model = "equidistant";
  bool has_intrinsics = false;
  bool has_extrinsics = false;

  double fx() const { return K(0, 0); }
  double fy() const { return K(1, 1); }
  double cx() const { return K(0, 2); }
  double cy() const { return K(1, 2); }

  Eigen::Matrix3d R_cl() const { return T_cam_lidar.block<3, 3>(0, 0); }
  Eigen::Vector3d t_cl() const { return T_cam_lidar.block<3, 1>(0, 3); }
};

/** @brief One LiDAR sweep with nearest associated camera frame. */
struct OxfordSpiresFrame {
  int index = 0;
  double timestamp = 0.0;       // LiDAR sweep start (sec)
  double image_timestamp = 0.0;  // nearest camera (sec)
  sensor::PointCloudXYZI::Ptr cloud;
  cv::Mat image;  // BGR
  std::string camera_id;
};

/**
 * @brief Sequential reader for Oxford Spires raw LiDAR / multi-camera sequences.
 */
class OxfordSpiresDataset {
 public:
  /**
   * @brief Open a sequence and index LiDAR (and camera) timestamps.
   * @param root Sequence root, .../raw, or a folder with lidar-clouds/ and cams.
   * @param options Camera id, sync threshold, optional calibration path.
   * @return True if LiDAR frames were indexed.
   */
  bool Open(const std::string& root,
            const OxfordSpiresOptions& options = OxfordSpiresOptions());

  /** @return Number of LiDAR-indexed frames. */
  std::size_t size() const { return lidar_stems_.size(); }

  /** @return True if no LiDAR frames were indexed. */
  bool empty() const { return lidar_stems_.empty(); }

  /** @return Path passed to Open(). */
  const std::string& root() const { return root_; }

  /** @return Resolved .../raw directory when found. */
  const std::string& raw_dir() const { return raw_dir_; }

  /** @return Loaded or default calibration. */
  const OxfordSpiresCalib& calib() const { return calib_; }

  /** @return Options used at Open(). */
  const OxfordSpiresOptions& options() const { return options_; }

  /**
   * @brief Load PCD cloud and optionally the time-aligned camera image.
   * @param index Frame index in [0, size()).
   * @param[out] out Populated frame.
   * @param load_image When false, skip image decode.
   * @return True on success.
   */
  bool Read(std::size_t index, OxfordSpiresFrame* out,
            bool load_image = true) const;

  /**
   * @brief Load Kalibr camN.yaml and DiffCal cam-lidar-imu.yaml.
   * @param calibration_dir Dataset-level calibration/ directory.
   * @param camera Camera id (cam0, cam1, cam2).
   * @param[out] out Filled intrinsics and extrinsics.
   * @return True when required YAML files parse successfully.
   */
  static bool LoadCalibration(const std::string& calibration_dir,
                              const std::string& camera,
                              OxfordSpiresCalib* out);

 private:
  bool ResolveDirs(const std::string& root);
  bool IndexSensors();
  bool LoadCalibIfAvailable();

  OxfordSpiresOptions options_;
  std::string root_;
  std::string raw_dir_;
  std::string lidar_dir_;
  std::string image_dir_;
  std::vector<std::string> lidar_stems_;
  std::vector<double> lidar_times_;
  std::vector<std::string> image_stems_;
  std::vector<double> image_times_;
  OxfordSpiresCalib calib_;
};

}  // namespace datasets
}  // namespace argus

#endif  // ARGUS_APPS_DATASETS_OXFORD_SPIRES_DATASET_HPP_
