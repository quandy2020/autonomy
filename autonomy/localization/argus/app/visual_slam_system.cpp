/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#include "visual_slam_system.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <exception>
#include <fstream>
#include <iostream>
#include <limits>
#include <mutex>
#include <utility>

#include "MapPoint.h"
#include "System.h"
#include "orb_vocabulary_cache.hpp"

namespace argus {
namespace app {
namespace {

ORB_SLAM3::System::eSensor ToOrbSensor(VisualSensorMode sensor) {
  switch (sensor) {
    case VisualSensorMode::kMonocular:
      return ORB_SLAM3::System::MONOCULAR;
    case VisualSensorMode::kStereo:
      return ORB_SLAM3::System::STEREO;
    case VisualSensorMode::kRgbd:
      return ORB_SLAM3::System::RGBD;
    case VisualSensorMode::kMonocularInertial:
      return ORB_SLAM3::System::IMU_MONOCULAR;
    case VisualSensorMode::kStereoInertial:
      return ORB_SLAM3::System::IMU_STEREO;
    case VisualSensorMode::kRgbdInertial:
      return ORB_SLAM3::System::IMU_RGBD;
  }
  return ORB_SLAM3::System::MONOCULAR;
}

bool IsInertial(VisualSensorMode sensor) {
  return sensor == VisualSensorMode::kMonocularInertial ||
         sensor == VisualSensorMode::kStereoInertial ||
         sensor == VisualSensorMode::kRgbdInertial;
}

bool IsReadableFile(const std::string& path) {
  if (path.empty()) return false;
  std::ifstream input(path);
  return static_cast<bool>(input);
}

bool ValidImage(const VisualImage& image) {
  return std::isfinite(image.time) && !image.pixels.empty() &&
         image.pixels.rows > 0 && image.pixels.cols > 0;
}

transform::Rigid3d ToWorldCamera(const Sophus::SE3f& camera_from_world) {
  const Sophus::SE3f world_from_camera = camera_from_world.inverse();
  const Eigen::Vector3d translation =
      world_from_camera.translation().cast<double>();
  Eigen::Quaterniond rotation(
      world_from_camera.unit_quaternion().cast<double>());
  rotation.normalize();
  return transform::Rigid3d(translation, rotation);
}

}  // namespace

class VisualSlamSystem::Impl {
 public:
  explicit Impl(VisualSlamConfig value) : config(std::move(value)) {}

  std::vector<ORB_SLAM3::IMU::Point> TakeImu(double image_time) {
    std::vector<ORB_SLAM3::IMU::Point> points;
    if (!IsInertial(config.sensor)) return points;
    while (!imu.empty() && imu.front().time <= image_time) {
      const sensor::ImuData sample = imu.front();
      imu.pop_front();
      points.emplace_back(
          static_cast<float>(sample.linear_acceleration.x()),
          static_cast<float>(sample.linear_acceleration.y()),
          static_cast<float>(sample.linear_acceleration.z()),
          static_cast<float>(sample.angular_velocity.x()),
          static_cast<float>(sample.angular_velocity.y()),
          static_cast<float>(sample.angular_velocity.z()), sample.time);
    }
    return points;
  }

  void UpdateResult(double time, const Sophus::SE3f& camera_from_world,
                    VisualTrackingResult* output) {
    latest.time = time;
    latest.tracking_state = system ? system->GetTrackingState() : 0;
    latest.valid = latest.tracking_state == 2 || latest.tracking_state == 5;
    if (latest.valid && camera_from_world.matrix().allFinite()) {
      latest.pose_world_camera = ToWorldCamera(camera_from_world);
    } else {
      latest.valid = false;
    }
    if (output != nullptr) *output = latest;
  }

  VisualSlamConfig config;
  mutable std::mutex mutex;
  std::unique_ptr<ORB_SLAM3::System> system;
  std::deque<sensor::ImuData> imu;
  VisualTrackingResult latest;
  double last_imu_time = -std::numeric_limits<double>::infinity();
};

VisualSlamSystem::VisualSlamSystem(VisualSlamConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

VisualSlamSystem::~VisualSlamSystem() { Stop(); }

bool VisualSlamSystem::Start() {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->system) return true;
  if (!IsReadableFile(impl_->config.settings_path)) {
    std::cerr << "Argus visual SLAM settings file is not readable: "
              << impl_->config.settings_path << std::endl;
    return false;
  }

  const std::string vocabulary_path =
      PrepareOrbVocabulary(impl_->config.vocabulary_path);
  if (vocabulary_path.empty() || !IsReadableFile(vocabulary_path)) {
    std::cerr << "Argus visual SLAM vocabulary could not be prepared: "
              << impl_->config.vocabulary_path << std::endl;
    return false;
  }
  try {
    impl_->system = std::make_unique<ORB_SLAM3::System>(
        vocabulary_path, impl_->config.settings_path,
        ToOrbSensor(impl_->config.sensor), impl_->config.enable_viewer);
  } catch (const std::exception& error) {
    std::cerr << "Argus visual SLAM failed to start: " << error.what()
              << std::endl;
    impl_->system.reset();
    return false;
  } catch (...) {
    std::cerr << "Argus visual SLAM failed to start with an unknown error"
              << std::endl;
    impl_->system.reset();
    return false;
  }
  return true;
}

void VisualSlamSystem::Stop() {
  std::unique_ptr<ORB_SLAM3::System> system;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    system = std::move(impl_->system);
    impl_->imu.clear();
  }
  if (system) system->Shutdown();
}

bool VisualSlamSystem::started() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->system != nullptr;
}

bool VisualSlamSystem::AddImu(const sensor::ImuData& sample) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->system || !IsInertial(impl_->config.sensor) ||
      !std::isfinite(sample.time) ||
      !sample.linear_acceleration.allFinite() ||
      !sample.angular_velocity.allFinite() ||
      sample.time <= impl_->last_imu_time) {
    return false;
  }
  impl_->imu.push_back(sample);
  impl_->last_imu_time = sample.time;
  return true;
}

bool VisualSlamSystem::TrackMonocular(const VisualImage& image,
                                      VisualTrackingResult* result) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->system || !ValidImage(image) ||
      (impl_->config.sensor != VisualSensorMode::kMonocular &&
       impl_->config.sensor != VisualSensorMode::kMonocularInertial)) {
    return false;
  }
  const Sophus::SE3f pose = impl_->system->TrackMonocular(
      image.pixels, image.time, impl_->TakeImu(image.time));
  impl_->UpdateResult(image.time, pose, result);
  return true;
}

bool VisualSlamSystem::TrackStereo(const VisualImage& left,
                                   const VisualImage& right,
                                   VisualTrackingResult* result) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->system || !ValidImage(left) || !ValidImage(right) ||
      std::abs(left.time - right.time) > 0.01 ||
      (impl_->config.sensor != VisualSensorMode::kStereo &&
       impl_->config.sensor != VisualSensorMode::kStereoInertial)) {
    return false;
  }
  const double time = std::max(left.time, right.time);
  const Sophus::SE3f pose = impl_->system->TrackStereo(
      left.pixels, right.pixels, time, impl_->TakeImu(time));
  impl_->UpdateResult(time, pose, result);
  return true;
}

bool VisualSlamSystem::TrackRgbd(const VisualImage& color,
                                 const VisualImage& depth,
                                 VisualTrackingResult* result) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->system || !ValidImage(color) || !ValidImage(depth) ||
      std::abs(color.time - depth.time) > 0.03 ||
      (impl_->config.sensor != VisualSensorMode::kRgbd &&
       impl_->config.sensor != VisualSensorMode::kRgbdInertial)) {
    return false;
  }
  const double time = std::max(color.time, depth.time);
  const Sophus::SE3f pose = impl_->system->TrackRGBD(
      color.pixels, depth.pixels, time, impl_->TakeImu(time));
  impl_->UpdateResult(time, pose, result);
  return true;
}

bool VisualSlamSystem::LatestResult(VisualTrackingResult* result) const {
  if (result == nullptr) return false;
  std::lock_guard<std::mutex> lock(impl_->mutex);
  *result = impl_->latest;
  return result->valid;
}

std::vector<Eigen::Vector3f> VisualSlamSystem::TrackedMapPoints() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  std::vector<Eigen::Vector3f> points;
  if (!impl_->system) return points;
  const auto tracked = impl_->system->GetTrackedMapPoints();
  points.reserve(tracked.size());
  for (ORB_SLAM3::MapPoint* point : tracked) {
    if (point == nullptr || point->isBad()) continue;
    const Eigen::Vector3f position = point->GetWorldPos();
    if (position.allFinite()) points.push_back(position);
  }
  return points;
}

void VisualSlamSystem::ActivateLocalizationMode() {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->system) impl_->system->ActivateLocalizationMode();
}

void VisualSlamSystem::DeactivateLocalizationMode() {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->system) impl_->system->DeactivateLocalizationMode();
}

void VisualSlamSystem::Reset() {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->system) impl_->system->Reset();
  impl_->imu.clear();
  impl_->latest = VisualTrackingResult();
  impl_->last_imu_time = -std::numeric_limits<double>::infinity();
}

}  // namespace app
}  // namespace argus
