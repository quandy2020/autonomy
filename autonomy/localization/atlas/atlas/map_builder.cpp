/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#include "atlas/map_builder.hpp"
#include "autolink/common/log.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <exception>
#include <fstream>
#include <iostream>
#include <limits>
#include <mutex>
#include <utility>

#include "atlas/sensor/map_point.hpp"
#include "atlas/system.hpp"

namespace autonomy::localization::atlas {
namespace {

::autonomy::localization::atlas::System::eSensor ToOrbSensor(
    SensorMode sensor) {
    switch (sensor) {
        case SensorMode::kMonocular:
            return ::autonomy::localization::atlas::System::MONOCULAR;
        case SensorMode::kStereo:
            return ::autonomy::localization::atlas::System::STEREO;
        case SensorMode::kRgbd:
            return ::autonomy::localization::atlas::System::RGBD;
        case SensorMode::kMonocularInertial:
            return ::autonomy::localization::atlas::System::IMU_MONOCULAR;
        case SensorMode::kStereoInertial:
            return ::autonomy::localization::atlas::System::IMU_STEREO;
        case SensorMode::kRgbdInertial:
            return ::autonomy::localization::atlas::System::IMU_RGBD;
    }
    return ::autonomy::localization::atlas::System::MONOCULAR;
}

bool IsInertial(SensorMode sensor) {
    return sensor == SensorMode::kMonocularInertial ||
           sensor == SensorMode::kStereoInertial ||
           sensor == SensorMode::kRgbdInertial;
}

bool IsReadableFile(const std::string& path) {
    if (path.empty())
        return false;
    std::ifstream input(path);
    return static_cast<bool>(input);
}

bool ValidImage(const ImageFrame& image) {
    return std::isfinite(image.time) && !image.pixels.empty() &&
           image.pixels.rows > 0 && image.pixels.cols > 0;
}

Pose3d ToWorldCamera(const autonomy::common::math::SE3f& camera_from_world) {
    const autonomy::common::math::SE3f world_from_camera =
        camera_from_world.inverse();
    Pose3d pose;
    pose.translation = world_from_camera.translation().cast<double>();
    pose.rotation =
        Eigen::Quaterniond(world_from_camera.unit_quaternion().cast<double>());
    pose.rotation.normalize();
    return pose;
}

}  // namespace

class MapBuilder::Impl
{
public:
    explicit Impl(SystemConfig value) : config(std::move(value)) {}

    std::vector<::autonomy::localization::atlas::IMU::Point> TakeImu(
        double image_time) {
        std::vector<::autonomy::localization::atlas::IMU::Point> points;
        if (!IsInertial(config.sensor))
            return points;
        while (!imu.empty() && imu.front().time <= image_time) {
            const ImuSample sample = imu.front();
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

    void UpdateResult(double time,
                      const autonomy::common::math::SE3f& camera_from_world,
                      TrackingResult* output) {
        latest.time = time;
        latest.tracking_state = system ? system->GetTrackingState() : 0;
        latest.valid = latest.tracking_state == 2 || latest.tracking_state == 5;
        if (latest.valid && camera_from_world.matrix().allFinite()) {
            latest.pose_world_camera = ToWorldCamera(camera_from_world);
        } else {
            latest.valid = false;
        }
        if (output != nullptr)
            *output = latest;
    }

    SystemConfig config;
    mutable std::mutex mutex;
    std::unique_ptr<::autonomy::localization::atlas::System> system;
    std::deque<ImuSample> imu;
    TrackingResult latest;
    double last_imu_time = -std::numeric_limits<double>::infinity();
};

MapBuilder::MapBuilder(SystemConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

MapBuilder::~MapBuilder() {
    Stop();
}

bool MapBuilder::Start() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->system)
        return true;
    if (!IsReadableFile(impl_->config.settings_path)) {
        AERROR << "autonomy atlas settings file is not readable: "
               << impl_->config.settings_path << std::endl;
        return false;
    }

    if (!IsReadableFile(impl_->config.vocabulary_path)) {
        AERROR << "autonomy atlas vocabulary could not be prepared: "
               << impl_->config.vocabulary_path << std::endl;
        return false;
    }
    try {
        impl_->system =
            std::make_unique<::autonomy::localization::atlas::System>(
                impl_->config.vocabulary_path, impl_->config.settings_path,
                ToOrbSensor(impl_->config.sensor), impl_->config.enable_viewer);
    } catch (const std::exception& error) {
        AERROR << "autonomy atlas failed to start: " << error.what()
               << std::endl;
        impl_->system.reset();
        return false;
    } catch (...) {
        AERROR << "autonomy atlas failed to start with an unknown error"
               << std::endl;
        impl_->system.reset();
        return false;
    }
    return true;
}

void MapBuilder::Stop() {
    std::unique_ptr<::autonomy::localization::atlas::System> system;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        system = std::move(impl_->system);
        impl_->imu.clear();
    }
    if (system)
        system->Shutdown();
}

bool MapBuilder::ShutdownAndSaveTumTrajectory(const std::string& path) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->system || path.empty())
        return false;
    impl_->system->Shutdown();
    impl_->system->SaveTrajectoryTUM(path);
    std::ifstream saved(path);
    return saved.good();
}

bool MapBuilder::ShutdownAndSaveEurocTrajectory(const std::string& path) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->system || path.empty())
        return false;
    impl_->system->Shutdown();
    impl_->system->SaveTrajectoryEuRoC(path);
    std::ifstream saved(path);
    return saved.good() && saved.peek() != std::ifstream::traits_type::eof();
}

bool MapBuilder::ShutdownAndSaveKittiTrajectory(const std::string& path) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->system || path.empty())
        return false;
    impl_->system->Shutdown();
    impl_->system->SaveTrajectoryKITTI(path);
    std::ifstream saved(path);
    return saved.good() && saved.peek() != std::ifstream::traits_type::eof();
}

bool MapBuilder::started() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->system != nullptr;
}

bool MapBuilder::AddImu(const ImuSample& sample) {
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

bool MapBuilder::TrackMonocular(const ImageFrame& image,
                                TrackingResult* result) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->system || !ValidImage(image) ||
        (impl_->config.sensor != SensorMode::kMonocular &&
         impl_->config.sensor != SensorMode::kMonocularInertial)) {
        return false;
    }
    const autonomy::common::math::SE3f pose = impl_->system->TrackMonocular(
        image.pixels, image.time, impl_->TakeImu(image.time));
    impl_->UpdateResult(image.time, pose, result);
    return true;
}

bool MapBuilder::TrackStereo(const ImageFrame& left, const ImageFrame& right,
                             TrackingResult* result) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->system || !ValidImage(left) || !ValidImage(right) ||
        std::abs(left.time - right.time) > 0.01 ||
        (impl_->config.sensor != SensorMode::kStereo &&
         impl_->config.sensor != SensorMode::kStereoInertial)) {
        return false;
    }
    const double time = std::max(left.time, right.time);
    const autonomy::common::math::SE3f pose = impl_->system->TrackStereo(
        left.pixels, right.pixels, time, impl_->TakeImu(time));
    impl_->UpdateResult(time, pose, result);
    return true;
}

bool MapBuilder::TrackRgbd(const ImageFrame& color, const ImageFrame& depth,
                           TrackingResult* result) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->system || !ValidImage(color) || !ValidImage(depth) ||
        std::abs(color.time - depth.time) > 0.03 ||
        (impl_->config.sensor != SensorMode::kRgbd &&
         impl_->config.sensor != SensorMode::kRgbdInertial)) {
        return false;
    }
    const double time = std::max(color.time, depth.time);
    const autonomy::common::math::SE3f pose = impl_->system->TrackRGBD(
        color.pixels, depth.pixels, time, impl_->TakeImu(time));
    impl_->UpdateResult(time, pose, result);
    return true;
}

bool MapBuilder::LatestResult(TrackingResult* result) const {
    if (result == nullptr)
        return false;
    std::lock_guard<std::mutex> lock(impl_->mutex);
    *result = impl_->latest;
    return result->valid;
}

std::vector<Eigen::Vector3f> MapBuilder::TrackedMapPoints() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    std::vector<Eigen::Vector3f> points;
    if (!impl_->system)
        return points;
    const auto tracked = impl_->system->GetTrackedMapPoints();
    points.reserve(tracked.size());
    for (::autonomy::localization::atlas::MapPoint* point : tracked) {
        if (point == nullptr || point->isBad())
            continue;
        const Eigen::Vector3f position = point->GetWorldPos();
        if (position.allFinite())
            points.push_back(position);
    }
    return points;
}

void MapBuilder::ActivateLocalizationMode() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->system)
        impl_->system->ActivateLocalizationMode();
}

void MapBuilder::DeactivateLocalizationMode() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->system)
        impl_->system->DeactivateLocalizationMode();
}

void MapBuilder::Reset() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->system)
        impl_->system->Reset();
    impl_->imu.clear();
    impl_->latest = TrackingResult();
    impl_->last_imu_time = -std::numeric_limits<double>::infinity();
}

}  // namespace autonomy::localization::atlas
