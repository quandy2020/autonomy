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
 * @file newer_college_dataset.hpp
 * @brief App-side reader for Newer College Dataset raw (EuRoC-style) dumps.
 *
 * Official docs: https://ori-drs.github.io/newer-college-dataset/
 *
 * Supported roots (auto-detected):
 *   - Stereo 2020: cam0|infra1 + ouster/lidar PCD folders, imu.csv
 *   - Multi-cam 2021: cam0/cam1/cam3/cam4 + ouster PCD, imu.csv
 *   - EuRoC nesting: <sensor>/data/<timestamp>.{png,pcd}
 *
 * Images / clouds are timestamp-named (integer ns or decimal seconds).
 */

#ifndef ARGUS_APPS_DATASETS_NEWER_COLLEGE_DATASET_HPP_
#define ARGUS_APPS_DATASETS_NEWER_COLLEGE_DATASET_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "argus/sensor/point_types.hpp"
#include "newer_college_options.hpp"

namespace argus {
namespace datasets {

/** @brief Detected Newer College on-disk layout variant. */
enum class NewerCollegeLayout {
  kUnknown = 0,
  kStereo,
  kMultiCam,
};

/** @brief Camera intrinsics and cam–LiDAR extrinsics for Newer College. */
struct NewerCollegeCalib {
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
  Eigen::VectorXd D = Eigen::VectorXd::Zero(5);
  Eigen::Matrix4d T_cam_lidar = Eigen::Matrix4d::Identity();
  int image_width = 0;
  int image_height = 0;
  std::string distortion_model;
  bool has_intrinsics = false;
  bool has_extrinsics = false;

  double fx() const { return K(0, 0); }
  double fy() const { return K(1, 1); }
  double cx() const { return K(0, 2); }
  double cy() const { return K(1, 2); }

  Eigen::Matrix3d R_cl() const { return T_cam_lidar.block<3, 3>(0, 0); }
  Eigen::Vector3d t_cl() const { return T_cam_lidar.block<3, 1>(0, 3); }
};

/** @brief One Ouster sweep with nearest associated camera frame. */
struct NewerCollegeFrame {
  int index = 0;
  double timestamp = 0.0;
  double image_timestamp = 0.0;
  sensor::PointCloudXYZI::Ptr cloud;
  cv::Mat image;
  std::string camera_id;
};

/**
 * @brief Sequential reader for Newer College Dataset raw EuRoC-style dumps.
 */
class NewerCollegeDataset {
 public:
  /**
   * @brief Open a dataset root and index LiDAR / camera stems.
   * @param root Sequence or sensor root (layout auto-detected).
   * @param options Preferred camera, sync threshold, optional calibration path.
   * @return True if LiDAR frames were indexed.
   */
  bool Open(const std::string& root,
            const NewerCollegeOptions& options = NewerCollegeOptions());

  /** @return Number of LiDAR-indexed frames. */
  std::size_t size() const { return lidar_stems_.size(); }

  /** @return True if no LiDAR frames were indexed. */
  bool empty() const { return lidar_stems_.empty(); }

  /** @return Path passed to Open(). */
  const std::string& root() const { return root_; }

  /** @return Detected layout (stereo 2020 vs multi-cam 2021). */
  NewerCollegeLayout layout() const { return layout_; }

  /** @return Loaded or default calibration. */
  const NewerCollegeCalib& calib() const { return calib_; }

  /** @return Options used at Open(). */
  const NewerCollegeOptions& options() const { return options_; }

  /** @return Resolved camera folder id (e.g. cam0, infra1). */
  const std::string& camera_id() const { return camera_id_; }

  /**
   * @brief Load PCD cloud and optionally the time-aligned camera image.
   * @param index Frame index in [0, size()).
   * @param[out] out Populated frame.
   * @param load_image When false, skip image decode.
   * @return True on success.
   */
  bool Read(std::size_t index, NewerCollegeFrame* out,
            bool load_image = true) const;

  /**
   * @brief Best-effort load from Kalibr OpenCV YAML or camchain.
   * @param calibration_dir Directory containing calibration YAML.
   * @param camera Camera id folder name.
   * @param[out] out Filled intrinsics and extrinsics when found.
   * @return True if at least intrinsics were loaded.
   */
  static bool LoadCalibration(const std::string& calibration_dir,
                              const std::string& camera,
                              NewerCollegeCalib* out);

 private:
  bool DetectLayout();
  bool IndexSensors();
  bool LoadCalibIfAvailable();
  bool LoadTimeOffsets();

  NewerCollegeOptions options_;
  NewerCollegeLayout layout_ = NewerCollegeLayout::kUnknown;
  std::string root_;
  std::string camera_id_;
  std::string lidar_dir_;
  std::string image_dir_;
  std::vector<std::string> lidar_stems_;
  std::vector<double> lidar_times_;
  std::vector<std::string> image_stems_;
  std::vector<double> image_times_;
  /** @brief Optional camera time correction: t' = t - offset(t). */
  std::vector<std::pair<double, double>> time_offsets_;  // (t_sec, offset_sec)
  NewerCollegeCalib calib_;
};

}  // namespace datasets
}  // namespace argus

#endif  // ARGUS_APPS_DATASETS_NEWER_COLLEGE_DATASET_HPP_
