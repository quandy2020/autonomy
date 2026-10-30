/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <opencv2/core/mat.hpp>

namespace autonomy::localization::atlas::map {

enum class CameraProjection {
    kUnknown,
    /** Images are already stereo-rectified and pixels follow a pinhole K. */
    kRectified,
    kPinhole,
    kKannalaBrandt8,
};

/** Camera projection and depth/stereo calibration used for dense mapping. */
struct CameraModel {
    CameraProjection projection = CameraProjection::kRectified;
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    /** Raw depth units per metre. A value of one means depth is in metres. */
    double depth_scale = 1.0;
    /** Metric distance between the left and right optical centres. */
    double stereo_baseline = 0.0;
    /** OpenCV pinhole distortion order: k1, k2, p1, p2, k3. */
    std::array<double, 5> distortion = {0.0, 0.0, 0.0, 0.0, 0.0};

    bool IsValid() const;
};

enum class ColorEncoding {
    kBgr,
    kRgb,
    kMono,
};

enum class DepthSource {
    kRgbd,
    kStereo,
    /** Current/reference monocular frames with a known relative pose. */
    kMonocularTemporal,
};

/** One tracking result with image ownership transferred to the map worker. */
struct MappingFrame {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double timestamp = 0.0;
    /** Stable source/keyframe id. Zero lets the pipeline allocate one. */
    std::uint64_t frame_id = 0;
    /** Source id of reference for monocular temporal reconstruction. */
    std::uint64_t reference_frame_id = 0;
    Eigen::Isometry3d world_from_camera = Eigen::Isometry3d::Identity();
    CameraModel camera;
    DepthSource depth_source = DepthSource::kRgbd;
    ColorEncoding color_encoding = ColorEncoding::kBgr;
    cv::Mat color;
    cv::Mat depth;
    cv::Mat right;
    cv::Mat reference;
    CameraModel secondary_camera;
    /** Transforms primary/current-camera coordinates into secondary-camera. */
    Eigen::Isometry3d secondary_from_camera = Eigen::Isometry3d::Identity();
    /** Transforms current-camera coordinates into reference-camera ones. */
    Eigen::Isometry3d reference_from_camera = Eigen::Isometry3d::Identity();
};

/** A coloured point in the map frame. */
struct ColorPoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3f position = Eigen::Vector3f::Zero();
    std::uint8_t red = 0;
    std::uint8_t green = 0;
    std::uint8_t blue = 0;
};

/** Semantic used only by the navigation occupancy layer. */
enum class PointSemantic : std::uint8_t {
    kUnknown,
    kGround,
    kObstacle,
};

struct SemanticPoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3f position = Eigen::Vector3f::Zero();
    PointSemantic semantic = PointSemantic::kUnknown;
};

using SemanticPointCloud =
    std::vector<SemanticPoint, Eigen::aligned_allocator<SemanticPoint>>;

struct GroundPlane {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    double offset = 0.0;
    bool valid = false;
    bool estimated = false;
};

struct ProjectionOptions {
    int pixel_stride = 4;
    double minimum_depth = 0.2;
    double maximum_depth = 5.0;
    std::size_t maximum_points_per_frame = 150000;
    int stereo_minimum_disparity = 0;
    int stereo_number_of_disparities = 128;
    int stereo_block_size = 5;
    double minimum_temporal_baseline = 0.02;
    double maximum_temporal_baseline = 2.0;
};

struct DenseMapOptions {
    double voxel_size = 0.05;
    std::size_t maximum_voxels = 1000000;
    std::size_t reserve_voxels = 200000;
};

struct OccupancyGridOptions {
    enum class ProjectionPlane {
        kXy,
        kXz,
        kYz,
    };

    enum class UpAxis {
        kPositiveX,
        kNegativeX,
        kPositiveY,
        kNegativeY,
        kPositiveZ,
        kNegativeZ,
    };

    ProjectionPlane projection_plane = ProjectionPlane::kXy;
    UpAxis up_axis = UpAxis::kPositiveZ;
    double resolution = 0.05;
    double minimum_obstacle_height = -0.5;
    double maximum_obstacle_height = 2.0;
    double hit_probability = 0.70;
    double miss_probability = 0.40;
    double minimum_probability = 0.1192;
    double maximum_probability = 0.971;
    double occupied_probability = 0.50;
    double maximum_extent = 250.0;
    std::size_t maximum_rays_per_frame = 50000;
    /** Angular buckets used to stop free-space rays at the nearest obstacle. */
    std::size_t ray_angle_bins = 360;
    std::size_t maximum_cells = 4000000;
    /** Upper bound for the contiguous grid published in one message. */
    std::size_t maximum_snapshot_cells = 2500000;
    /** Circular robot footprint radius. Zero disables footprint inflation. */
    double robot_radius = 0.0;
    /** Total graded-cost radius from an obstacle; zero disables the band. */
    double inflation_radius = 0.0;
    /** Exponential cost decay per metre outside the robot footprint. */
    double inflation_cost_scaling = 0.0;
    /** Camera/sensor height above the local ground plane in metres. */
    double sensor_height = 0.0;
    /** Interpret obstacle-height limits relative to the ground plane. */
    bool use_sensor_height = false;
    /** Fit a gravity-constrained plane before using SensorHeight fallback. */
    bool estimate_ground_plane = false;
    /** Symmetric band around the plane classified as traversable ground. */
    double ground_distance_threshold = 0.10;
    /** Flat-ground prior margin used to select RANSAC candidates. */
    double ground_candidate_height_margin = 0.25;
    /** Maximum plane tilt relative to the configured UpAxis. */
    double maximum_ground_slope_radians = 0.3490658503988659;
    /** Allowed difference between fitted plane distance and SensorHeight. */
    double ground_sensor_height_tolerance = 0.40;
    /** Only nearby points participate in plane fitting. */
    double maximum_ground_estimation_range = 12.0;
    std::size_t ground_ransac_iterations = 64;
    std::size_t minimum_ground_inliers = 40;
    std::size_t maximum_ground_estimation_points = 4096;
    /** Published navigation-cloud height band relative to local ground. */
    double minimum_navigation_height = -1.0;
    double maximum_navigation_height = 1.0;
};

struct MappingPipelineOptions {
    ProjectionOptions projection;
    DenseMapOptions dense_map;
    /** Independent hard bound for the ground-relative navigation cloud. */
    std::size_t maximum_navigation_voxels = 1500000;
    /** Radius of the rolling navigation cloud; zero keeps the global cloud. */
    double navigation_radius = 0.0;
    /** Distance travelled before re-centering an enabled rolling cloud. */
    double navigation_recenter_distance = 5.0;
    OccupancyGridOptions occupancy_grid;
    std::size_t queue_capacity = 2;
    double minimum_translation = 0.10;
    double minimum_rotation_radians = 0.08726646259971647;
    double maximum_frame_interval = 1.0;
    double snapshot_period = 1.0;
    /** Maximum number of local clouds/images kept resident in RAM. */
    std::size_t maximum_cached_frames = 400;
    /** Hard limit for this instance's disk-backed cold history. */
    std::uint64_t maximum_history_cache_bytes = 8ULL * 1024ULL * 1024ULL * 1024ULL;
    /** Optional existing parent directory for the unique history directory. */
    std::string history_cache_directory;
    double rebuild_translation_threshold = 0.02;
    double rebuild_rotation_threshold_radians = 0.017453292519943295;
};

/**
 * Optimized pose used to move a cached camera-frame cloud after loop closure.
 */
struct TimedPose {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::uint64_t frame_id = 0;
    double timestamp = 0.0;
    Eigen::Isometry3d world_from_camera = Eigen::Isometry3d::Identity();
};

struct DenseMapSnapshot {
    double timestamp = 0.0;
    std::uint64_t generation = 0;
    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>> points;
};

struct OccupancyGridSnapshot {
    double timestamp = 0.0;
    std::uint64_t generation = 0;
    double resolution = 0.0;
    double origin_x = 0.0;
    double origin_y = 0.0;
    /** World pose of the grid plane's local (origin_x, origin_y, 0). */
    Eigen::Vector3d origin_translation = Eigen::Vector3d::Zero();
    Eigen::Quaterniond origin_rotation = Eigen::Quaterniond::Identity();
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    /** Row-major values: -1 unknown, 0 free, 1..99 inflated, 100 occupied. */
    std::vector<std::int8_t> data;
};

struct MapSnapshot {
    DenseMapSnapshot dense_map;
    /** Ground-relative [-1, 1] m view used by the navigation publisher. */
    DenseMapSnapshot navigation_map;
    OccupancyGridSnapshot occupancy_grid;
};

/** Completion records drained by the producer after asynchronous culling. */
struct MappingFrameRemovalResults {
    /** Requests fully reflected in the published dense/navigation maps. */
    std::vector<std::uint64_t> removed_frame_ids;
    /** Requests retained because removing them could corrupt the valid map. */
    std::vector<std::uint64_t> retained_frame_ids;
};

struct MappingPipelineStats {
    std::uint64_t submitted_frames = 0;
    std::uint64_t dropped_frames = 0;
    /** Non-blocking submissions deferred because the input queue was full. */
    std::uint64_t backpressured_frames = 0;
    /** Accepted queued frames explicitly discarded by Reset(). */
    std::uint64_t reset_discarded_frames = 0;
    std::uint64_t gated_frames = 0;
    std::uint64_t processed_frames = 0;
    std::uint64_t rejected_frames = 0;
    /** Queued or fused frame records removed after keyframe culling. */
    std::uint64_t removed_frames = 0;
    /** Safe removals refused because a complete rebuild was unavailable. */
    std::uint64_t removal_rebuild_failures = 0;
    /** Frames rejected because their stable id was already fused. */
    std::uint64_t duplicate_frames = 0;
    /** Monocular frames reprojected after a relative-pose change. */
    std::uint64_t retriangulated_frames = 0;
    /** Rebuilds refused because their bounded cache was not full history. */
    std::uint64_t skipped_incomplete_rebuilds = 0;
    /** Cold-history write/read/decode failures, including quota exhaustion. */
    std::uint64_t history_archive_failures = 0;
};

}  // namespace autonomy::localization::atlas::map
