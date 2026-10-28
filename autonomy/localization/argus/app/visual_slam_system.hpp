/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#ifndef ARGUS_APP_VISUAL_SLAM_SYSTEM_HPP_
#define ARGUS_APP_VISUAL_SLAM_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "common/transform/rigid_transform.hpp"
#include "sensor/imu_types.hpp"

namespace argus {
namespace app {

enum class VisualSensorMode {
  kMonocular,
  kStereo,
  kRgbd,
  kMonocularInertial,
  kStereoInertial,
  kRgbdInertial,
};

struct VisualSlamConfig {
  std::string vocabulary_path;
  std::string settings_path;
  VisualSensorMode sensor = VisualSensorMode::kMonocular;
  bool enable_viewer = false;
};

struct VisualImage {
  double time = 0.0;
  std::string frame_id;
  cv::Mat pixels;
};

struct VisualTrackingResult {
  double time = 0.0;
  transform::Rigid3d pose_world_camera = transform::Rigid3d::Identity();
  int tracking_state = 0;
  bool valid = false;
};

class VisualSlamSystem {
 public:
  explicit VisualSlamSystem(VisualSlamConfig config);
  ~VisualSlamSystem();

  VisualSlamSystem(const VisualSlamSystem&) = delete;
  VisualSlamSystem& operator=(const VisualSlamSystem&) = delete;

  bool Start();
  void Stop();
  bool started() const;

  bool AddImu(const sensor::ImuData& imu);
  bool TrackMonocular(const VisualImage& image,
                      VisualTrackingResult* result = nullptr);
  bool TrackStereo(const VisualImage& left, const VisualImage& right,
                   VisualTrackingResult* result = nullptr);
  bool TrackRgbd(const VisualImage& color, const VisualImage& depth,
                 VisualTrackingResult* result = nullptr);

  bool LatestResult(VisualTrackingResult* result) const;
  std::vector<Eigen::Vector3f> TrackedMapPoints() const;

  void ActivateLocalizationMode();
  void DeactivateLocalizationMode();
  void Reset();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace app
}  // namespace argus

#endif  // ARGUS_APP_VISUAL_SLAM_SYSTEM_HPP_
