/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/mat.hpp>

#include "atlas/map/map_types.hpp"

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
    /** Enables asynchronous dense-cloud and occupancy-grid construction. */
    bool enable_dense_mapping = true;
    /** Performance and probability settings for the asynchronous map. */
    map::MappingPipelineOptions mapping_options;
    /** How often the optimized keyframe list is sampled for new frames. */
    double mapping_keyframe_poll_period_sec = 0.50;
    /** Maximum keyframes submitted by one optimized-trajectory refresh. */
    std::size_t maximum_mapping_keyframes_per_refresh = 4;
    /** Bounds image memory while local mapping creates a keyframe. */
    std::size_t maximum_pending_mapping_frames = 45;
};

/** @brief One decoded image and its timestamp in seconds. */
struct ImageFrame {
    double time = 0.0;
    std::string frame_id;
    cv::Mat pixels;
    /** Original AutoMsgs encoding, used to preserve RGB/BGR channel order. */
    std::string encoding = "bgr8";
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
    /** Stable identity of the active Atlas map containing this result. */
    std::uint64_t map_id = 0;
    /** False only when no active Atlas map identity could be obtained. */
    bool map_id_valid = false;
    /** Legacy ORB-SLAM3 state value; see TrackingState. */
    int tracking_state = 0;
    /** Whether pose_world_camera contains a usable finite pose. */
    bool valid = false;
    /**
     * Monotonic identity of the active map's coordinate representation. This
     * advances when a reset clears the map or inertial initialization applies
     * a global scaled rotation, even if Atlas retains the same map_id.
     */
    std::uint64_t coordinate_epoch = 0;
    /** Advances only when Reset or ResetActiveMap destroys map contents. */
    std::uint64_t destructive_epoch = 0;
};

/** @brief One optimized camera pose sampled at its source image timestamp. */
struct TrajectoryPose {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double time = 0.0;
    Pose3d pose_world_camera;
};

/** @brief One loop-closure constraint represented by its keyframe endpoints. */
struct LoopClosureEdge {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d first = Eigen::Vector3d::Zero();
    Eigen::Vector3d second = Eigen::Vector3d::Zero();
};

}  // namespace autonomy::localization::atlas
