/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#pragma once

#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/mat.hpp>

namespace autonomy::localization::atlas {

/** @brief Physical sensor families understood by Atlas adapters. */
enum class SensorType {
    kCamera,
    kImu,
    kPointCloud,
};

/** @brief Stable sensor identity within a configured rig. */
struct SensorId {
    /** Sensor family. */
    SensorType type = SensorType::kCamera;
    /** Coordinate frame carried by measurements from this sensor. */
    std::string frame_id;
    /** Zero-based index among sensors of the same family. */
    int index = 0;
};

/** @brief Tracking pipeline selected for one MapBuilder session. */
enum class SensorMode {
    kMonocular,
    kStereo,
    kRgbd,
    kMonocularInertial,
    kStereoInertial,
    kRgbdInertial,
};

/** @brief Immutable startup configuration for MapBuilder. */
struct SystemConfig {
    /** Path to the binary FBoW vocabulary. */
    std::string vocabulary_path;
    /** Path to the ORB-SLAM3 camera and tracker settings. */
    std::string settings_path;
    /** Sensor combination accepted by tracking calls. */
    SensorMode sensor = SensorMode::kMonocular;
    /** Enables the optional viewer when supported by the build. */
    bool enable_viewer = false;
};

/** @brief One decoded image and its timestamp in seconds. */
struct ImageFrame {
    double time = 0.0;
    std::string frame_id;
    cv::Mat pixels;
};

/** @brief One body-frame inertial sample in SI units. */
struct ImuSample {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double time = 0.0;
    Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};

/** @brief Rigid pose represented as translation plus unit quaternion. */
struct Pose3d {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
};

/** @brief Result produced for the latest accepted camera observation. */
struct TrackingResult {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double time = 0.0;
    Pose3d pose_world_camera;
    /** Legacy ORB-SLAM3 state value; see TrackingState. */
    int tracking_state = 0;
    /** Whether pose_world_camera contains a usable finite pose. */
    bool valid = false;
};

}  // namespace autonomy::localization::atlas
