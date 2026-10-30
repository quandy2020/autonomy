/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#include "atlas/map_builder.hpp"
#include "autolink/common/log.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <deque>
#include <exception>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <opencv2/core/persistence.hpp>

#include "atlas/map/mapping_pipeline.hpp"
#include "atlas/sensor/map_point.hpp"
#include "atlas/system.hpp"

namespace autonomy::localization::atlas {
namespace {

constexpr std::int64_t kInvalidTimestampKey =
    std::numeric_limits<std::int64_t>::min();

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

bool IsStereo(SensorMode sensor) {
    return sensor == SensorMode::kStereo ||
           sensor == SensorMode::kStereoInertial;
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

Eigen::Isometry3d ToIsometry(const Pose3d& pose) {
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond rotation = pose.rotation;
    if (rotation.coeffs().allFinite() && rotation.norm() > 1e-12)
        rotation.normalize();
    result.linear() = rotation.toRotationMatrix();
    result.translation() = pose.translation;
    return result;
}

map::ColorEncoding ToColorEncoding(const std::string& encoding) {
    if (encoding == "rgb8" || encoding == "rgba8")
        return map::ColorEncoding::kRgb;
    if (encoding == "mono8" || encoding == "mono16" ||
        encoding == "8UC1" || encoding == "16UC1") {
        return map::ColorEncoding::kMono;
    }
    return map::ColorEncoding::kBgr;
}

bool ReadNumber(const cv::FileStorage& settings,
                std::initializer_list<const char*> names, double* value) {
    if (value == nullptr)
        return false;
    for (const char* name : names) {
        const cv::FileNode node = settings[name];
        if (node.empty() || (!node.isInt() && !node.isReal()))
            continue;
        const double candidate = node.real();
        if (!std::isfinite(candidate))
            continue;
        *value = candidate;
        return true;
    }
    return false;
}

bool ReadString(const cv::FileStorage& settings,
                std::initializer_list<const char*> names,
                std::string* value) {
    if (value == nullptr)
        return false;
    for (const char* name : names) {
        const cv::FileNode node = settings[name];
        if (node.empty() || !node.isString())
            continue;
        *value = static_cast<std::string>(node);
        return true;
    }
    return false;
}

std::string Uppercase(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char character) {
                       return static_cast<char>(std::toupper(character));
                   });
    return value;
}

bool ReadTransform(const cv::FileStorage& settings,
                   std::initializer_list<const char*> names,
                   bool invert,
                   Eigen::Isometry3d* transform) {
    if (transform == nullptr)
        return false;
    for (const char* name : names) {
        const cv::FileNode node = settings[name];
        if (node.empty())
            continue;
        const cv::Mat encoded = node.mat();
        if (encoded.empty() || encoded.channels() != 1 ||
            !((encoded.rows == 3 && encoded.cols == 4) ||
              (encoded.rows == 4 && encoded.cols == 4))) {
            continue;
        }
        cv::Mat matrix;
        encoded.convertTo(matrix, CV_64F);
        Eigen::Matrix3d rotation;
        Eigen::Vector3d translation;
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column)
                rotation(row, column) = matrix.at<double>(row, column);
            translation(row) = matrix.at<double>(row, 3);
        }
        Eigen::Quaterniond quaternion(rotation);
        if (!rotation.allFinite() || !translation.allFinite() ||
            !quaternion.coeffs().allFinite() || quaternion.norm() < 1e-12) {
            continue;
        }
        quaternion.normalize();
        Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
        result.linear() = quaternion.toRotationMatrix();
        result.translation() = translation;
        *transform = invert ? result.inverse() : result;
        return true;
    }
    return false;
}

bool ReadPositive(const cv::FileStorage& settings,
                  std::initializer_list<const char*> names, double* value) {
    double candidate = 0.0;
    if (!ReadNumber(settings, names, &candidate) || candidate <= 0.0)
        return false;
    *value = candidate;
    return true;
}

void ReadOptionalPositive(const cv::FileStorage& settings,
                          std::initializer_list<const char*> names,
                          double* value) {
    double candidate = 0.0;
    if (ReadPositive(settings, names, &candidate))
        *value = candidate;
}

void ReadOptionalNonnegative(const cv::FileStorage& settings,
                             std::initializer_list<const char*> names,
                             double* value) {
    double candidate = 0.0;
    if (ReadNumber(settings, names, &candidate) && candidate >= 0.0)
        *value = candidate;
}

bool ReadNonnegativeIfPresent(const cv::FileStorage& settings,
                              std::initializer_list<const char*> names,
                              double* value) {
    if (value == nullptr)
        return false;
    for (const char* name : names) {
        const cv::FileNode node = settings[name];
        if (node.empty())
            continue;
        if (!node.isInt() && !node.isReal())
            return false;
        const double candidate = node.real();
        if (!std::isfinite(candidate) || candidate < 0.0)
            return false;
        *value = candidate;
        return true;
    }
    return true;
}

void ReadOptionalSize(const cv::FileStorage& settings,
                      std::initializer_list<const char*> names,
                      std::size_t* value) {
    double candidate = 0.0;
    if (!ReadNumber(settings, names, &candidate) || candidate < 1.0 ||
        candidate > static_cast<double>(
                        std::numeric_limits<std::size_t>::max())) {
        return;
    }
    *value = static_cast<std::size_t>(candidate);
}

void ReadOptionalByteCount(const cv::FileStorage& settings,
                           std::initializer_list<const char*> names,
                           std::uint64_t* value) {
    constexpr double kUint64UpperBound = 18446744073709551616.0;
    double candidate = 0.0;
    if (value == nullptr || !ReadNumber(settings, names, &candidate) ||
        candidate < 0.0 || candidate >= kUint64UpperBound ||
        candidate != std::floor(candidate)) {
        return;
    }
    *value = static_cast<std::uint64_t>(candidate);
}

bool LoadMappingSettings(const SystemConfig& config,
                         map::CameraModel* camera,
                         map::CameraModel* secondary_camera,
                         Eigen::Isometry3d* secondary_from_camera,
                         map::MappingPipelineOptions* options) {
    if (camera == nullptr || secondary_camera == nullptr ||
        secondary_from_camera == nullptr || options == nullptr) {
        return false;
    }
    cv::FileStorage settings(config.settings_path, cv::FileStorage::READ);
    if (!settings.isOpened())
        return false;

    if (!ReadPositive(settings, {"Camera1.fx", "Camera.fx"}, &camera->fx) ||
        !ReadPositive(settings, {"Camera1.fy", "Camera.fy"}, &camera->fy) ||
        !ReadNumber(settings, {"Camera1.cx", "Camera.cx"}, &camera->cx) ||
        !ReadNumber(settings, {"Camera1.cy", "Camera.cy"}, &camera->cy)) {
        return false;
    }

    std::string camera_type;
    ReadString(settings, {"Camera.type", "Camera1.type"}, &camera_type);
    camera_type = Uppercase(camera_type);
    if (camera_type.find("RECTIFIED") != std::string::npos) {
        camera->projection = map::CameraProjection::kRectified;
    } else if (camera_type.find("KANNALA") != std::string::npos ||
               camera_type.find("FISHEYE") != std::string::npos) {
        camera->projection = map::CameraProjection::kKannalaBrandt8;
    } else {
        camera->projection = map::CameraProjection::kPinhole;
    }

    camera->depth_scale = 1.0;
    ReadOptionalPositive(settings,
                         {"RGBD.DepthMapFactor", "DepthMapFactor"},
                         &camera->depth_scale);
    camera->stereo_baseline = 0.0;
    ReadOptionalPositive(settings, {"Stereo.b"}, &camera->stereo_baseline);
    if (camera->stereo_baseline <= 0.0) {
        double baseline_times_focal = 0.0;
        if (ReadPositive(settings, {"Camera.bf"}, &baseline_times_focal)) {
            camera->stereo_baseline = baseline_times_focal / camera->fx;
        }
    }
    camera->distortion.fill(0.0);
    ReadNumber(settings, {"Camera1.k1", "Camera.k1"},
               &camera->distortion[0]);
    ReadNumber(settings, {"Camera1.k2", "Camera.k2"},
               &camera->distortion[1]);
    if (camera->projection == map::CameraProjection::kKannalaBrandt8) {
        ReadNumber(settings, {"Camera1.k3", "Camera.k3"},
                   &camera->distortion[2]);
        ReadNumber(settings, {"Camera1.k4", "Camera.k4"},
                   &camera->distortion[3]);
    } else {
        ReadNumber(settings, {"Camera1.p1", "Camera.p1"},
                   &camera->distortion[2]);
        ReadNumber(settings, {"Camera1.p2", "Camera.p2"},
                   &camera->distortion[3]);
        ReadNumber(settings, {"Camera1.k3", "Camera.k3"},
                   &camera->distortion[4]);
    }

    *secondary_camera = *camera;
    const bool have_secondary_intrinsics =
        ReadPositive(settings, {"Camera2.fx"}, &secondary_camera->fx) &&
        ReadPositive(settings, {"Camera2.fy"}, &secondary_camera->fy) &&
        ReadNumber(settings, {"Camera2.cx"}, &secondary_camera->cx) &&
        ReadNumber(settings, {"Camera2.cy"}, &secondary_camera->cy);
    if (!have_secondary_intrinsics) {
        *secondary_camera = *camera;
    } else {
        secondary_camera->distortion.fill(0.0);
        ReadNumber(settings, {"Camera2.k1"},
                   &secondary_camera->distortion[0]);
        ReadNumber(settings, {"Camera2.k2"},
                   &secondary_camera->distortion[1]);
        if (secondary_camera->projection ==
            map::CameraProjection::kKannalaBrandt8) {
            ReadNumber(settings, {"Camera2.k3"},
                       &secondary_camera->distortion[2]);
            ReadNumber(settings, {"Camera2.k4"},
                       &secondary_camera->distortion[3]);
        } else {
            ReadNumber(settings, {"Camera2.p1"},
                       &secondary_camera->distortion[2]);
            ReadNumber(settings, {"Camera2.p2"},
                       &secondary_camera->distortion[3]);
            ReadNumber(settings, {"Camera2.k3"},
                       &secondary_camera->distortion[4]);
        }
    }

    *secondary_from_camera = Eigen::Isometry3d::Identity();
    bool have_stereo_transform =
        ReadTransform(settings, {"Tlr"}, false, secondary_from_camera);
    if (!have_stereo_transform) {
        // T_c1_c2 maps camera-2 points to camera-1, the inverse of the map
        // pipeline's secondary-from-primary convention.
        have_stereo_transform = ReadTransform(
            settings, {"Stereo.T_c1_c2"}, true, secondary_from_camera);
    }
    if (have_stereo_transform) {
        camera->stereo_baseline =
            secondary_from_camera->translation().norm();
    } else if (camera->stereo_baseline > 0.0) {
        secondary_from_camera->translation().x() =
            -camera->stereo_baseline;
    }
    if (IsStereo(config.sensor) && !have_secondary_intrinsics &&
        !have_stereo_transform) {
        camera->projection = map::CameraProjection::kRectified;
    }
    secondary_camera->projection = camera->projection;
    secondary_camera->stereo_baseline = camera->stereo_baseline;

    ReadOptionalPositive(settings, {"Atlas.Map.RangeMin"},
                         &options->projection.minimum_depth);
    ReadOptionalPositive(settings, {"Atlas.Map.RangeMax"},
                         &options->projection.maximum_depth);
    ReadOptionalPositive(settings, {"Atlas.Map.VoxelSize"},
                         &options->dense_map.voxel_size);
    ReadOptionalPositive(settings, {"Atlas.Map.GridResolution"},
                         &options->occupancy_grid.resolution);
    if (!ReadNonnegativeIfPresent(
            settings, {"Atlas.Map.RobotRadius"},
            &options->occupancy_grid.robot_radius) ||
        !ReadNonnegativeIfPresent(
            settings, {"Atlas.Map.InflationRadius"},
            &options->occupancy_grid.inflation_radius) ||
        !ReadNonnegativeIfPresent(
            settings, {"Atlas.Map.InflationCostScaling"},
            &options->occupancy_grid.inflation_cost_scaling) ||
        !ReadNonnegativeIfPresent(
            settings, {"Atlas.Map.NavigationRadius"},
            &options->navigation_radius) ||
        !ReadNonnegativeIfPresent(
            settings, {"Atlas.Map.NavigationRecenterDistance"},
            &options->navigation_recenter_distance)) {
        return false;
    }
    double sensor_height = options->occupancy_grid.sensor_height;
    if (ReadPositive(settings, {"Atlas.Map.SensorHeight"}, &sensor_height)) {
        options->occupancy_grid.sensor_height = sensor_height;
        options->occupancy_grid.use_sensor_height = true;
    }
    double estimate_ground =
        options->occupancy_grid.estimate_ground_plane ? 1.0 : 0.0;
    if (ReadNumber(settings, {"Atlas.Map.EstimateGroundPlane"},
                   &estimate_ground)) {
        if (estimate_ground != 0.0 && estimate_ground != 1.0)
            return false;
        options->occupancy_grid.estimate_ground_plane =
            estimate_ground == 1.0;
    }
    ReadOptionalPositive(settings, {"Atlas.Map.GroundDistanceThreshold"},
                         &options->occupancy_grid.ground_distance_threshold);
    ReadOptionalNonnegative(
        settings, {"Atlas.Map.GroundCandidateHeightMargin"},
        &options->occupancy_grid.ground_candidate_height_margin);
    ReadOptionalNonnegative(
        settings, {"Atlas.Map.GroundSensorHeightTolerance"},
        &options->occupancy_grid.ground_sensor_height_tolerance);
    ReadOptionalPositive(
        settings, {"Atlas.Map.MaximumGroundEstimationRange"},
        &options->occupancy_grid.maximum_ground_estimation_range);
    double ground_slope_degrees =
        options->occupancy_grid.maximum_ground_slope_radians *
        180.0 / 3.14159265358979323846;
    if (ReadNumber(settings, {"Atlas.Map.MaximumGroundSlopeDegrees"},
                   &ground_slope_degrees)) {
        if (ground_slope_degrees < 0.0 || ground_slope_degrees >= 90.0)
            return false;
        options->occupancy_grid.maximum_ground_slope_radians =
            ground_slope_degrees * 3.14159265358979323846 / 180.0;
    }
    double minimum_height = options->occupancy_grid.minimum_obstacle_height;
    if (ReadNumber(settings, {"Atlas.Map.MinHeight"}, &minimum_height))
        options->occupancy_grid.minimum_obstacle_height = minimum_height;
    double maximum_height = options->occupancy_grid.maximum_obstacle_height;
    if (ReadNumber(settings, {"Atlas.Map.MaxHeight"}, &maximum_height))
        options->occupancy_grid.maximum_obstacle_height = maximum_height;
    ReadNumber(settings, {"Atlas.Map.NavigationMinHeight"},
               &options->occupancy_grid.minimum_navigation_height);
    ReadNumber(settings, {"Atlas.Map.NavigationMaxHeight"},
               &options->occupancy_grid.maximum_navigation_height);
    if (options->occupancy_grid.minimum_obstacle_height >
            options->occupancy_grid.maximum_obstacle_height ||
        options->occupancy_grid.minimum_navigation_height >
            options->occupancy_grid.maximum_navigation_height) {
        return false;
    }
    ReadOptionalPositive(settings, {"Atlas.Map.MaximumExtent"},
                         &options->occupancy_grid.maximum_extent);
    ReadOptionalNonnegative(settings, {"Atlas.Map.MinimumTranslation"},
                            &options->minimum_translation);
    ReadOptionalPositive(settings, {"Atlas.Map.SnapshotPeriod"},
                         &options->snapshot_period);
    ReadOptionalPositive(settings, {"Atlas.Map.MinimumTemporalBaseline"},
                         &options->projection.minimum_temporal_baseline);
    ReadOptionalPositive(settings, {"Atlas.Map.MaximumTemporalBaseline"},
                         &options->projection.maximum_temporal_baseline);

    double degrees = 0.0;
    if (ReadNumber(settings, {"Atlas.Map.MinimumRotationDegrees"}, &degrees) &&
        degrees >= 0.0) {
        options->minimum_rotation_radians =
            degrees * 3.14159265358979323846 / 180.0;
    }

    std::size_t size_value = options->queue_capacity;
    ReadOptionalSize(settings, {"Atlas.Map.QueueCapacity"}, &size_value);
    options->queue_capacity = size_value;
    size_value = options->maximum_cached_frames;
    ReadOptionalSize(settings, {"Atlas.Map.MaximumCachedFrames"},
                     &size_value);
    options->maximum_cached_frames = size_value;
    ReadOptionalByteCount(settings, {"Atlas.Map.MaximumHistoryCacheBytes"},
                          &options->maximum_history_cache_bytes);
    std::string history_cache_directory = options->history_cache_directory;
    if (ReadString(settings, {"Atlas.Map.HistoryCacheDirectory"},
                   &history_cache_directory))
        options->history_cache_directory = history_cache_directory;
    size_value = options->dense_map.maximum_voxels;
    ReadOptionalSize(settings, {"Atlas.Map.MaximumVoxels"}, &size_value);
    options->dense_map.maximum_voxels = size_value;
    size_value = options->maximum_navigation_voxels;
    ReadOptionalSize(settings, {"Atlas.Map.MaximumNavigationVoxels"},
                     &size_value);
    options->maximum_navigation_voxels = size_value;
    size_value = options->occupancy_grid.maximum_cells;
    ReadOptionalSize(settings, {"Atlas.Map.MaximumCells"}, &size_value);
    options->occupancy_grid.maximum_cells = size_value;
    size_value = options->occupancy_grid.maximum_snapshot_cells;
    ReadOptionalSize(settings, {"Atlas.Map.MaximumSnapshotCells"},
                     &size_value);
    options->occupancy_grid.maximum_snapshot_cells = size_value;
    size_value = options->occupancy_grid.maximum_rays_per_frame;
    ReadOptionalSize(settings, {"Atlas.Map.MaximumRaysPerFrame"},
                     &size_value);
    options->occupancy_grid.maximum_rays_per_frame = size_value;
    size_value = options->occupancy_grid.ray_angle_bins;
    ReadOptionalSize(settings, {"Atlas.Map.RayAngleBins"}, &size_value);
    options->occupancy_grid.ray_angle_bins = size_value;
    size_value = options->occupancy_grid.ground_ransac_iterations;
    ReadOptionalSize(settings, {"Atlas.Map.GroundRansacIterations"},
                     &size_value);
    options->occupancy_grid.ground_ransac_iterations = size_value;
    size_value = options->occupancy_grid.minimum_ground_inliers;
    ReadOptionalSize(settings, {"Atlas.Map.MinimumGroundInliers"},
                     &size_value);
    options->occupancy_grid.minimum_ground_inliers = size_value;
    size_value = options->occupancy_grid.maximum_ground_estimation_points;
    ReadOptionalSize(settings,
                     {"Atlas.Map.MaximumGroundEstimationPoints"},
                     &size_value);
    options->occupancy_grid.maximum_ground_estimation_points = size_value;
    size_value = options->projection.maximum_points_per_frame;
    ReadOptionalSize(settings, {"Atlas.Map.MaximumPointsPerFrame"},
                     &size_value);
    options->projection.maximum_points_per_frame = size_value;
    double stride = options->projection.pixel_stride;
    if (ReadPositive(settings, {"Atlas.Map.PixelStride"}, &stride)) {
        options->projection.pixel_stride =
            std::max(1, static_cast<int>(std::llround(stride)));
    }
    double stereo_minimum_disparity =
        options->projection.stereo_minimum_disparity;
    if (ReadNumber(settings, {"Atlas.Map.StereoMinimumDisparity"},
                   &stereo_minimum_disparity)) {
        if (stereo_minimum_disparity <
                static_cast<double>(std::numeric_limits<int>::min()) ||
            stereo_minimum_disparity >
                static_cast<double>(std::numeric_limits<int>::max()) ||
            std::abs(stereo_minimum_disparity -
                     std::round(stereo_minimum_disparity)) > 1e-9) {
            return false;
        }
        options->projection.stereo_minimum_disparity =
            static_cast<int>(std::llround(stereo_minimum_disparity));
    }
    double stereo_disparities =
        options->projection.stereo_number_of_disparities;
    if (ReadNumber(settings,
                   {"Atlas.Map.StereoNumberOfDisparities",
                    "Atlas.Map.StereoNumDisparities"},
                   &stereo_disparities)) {
        if (stereo_disparities < 16.0 ||
            stereo_disparities >
                static_cast<double>(std::numeric_limits<int>::max()) ||
            std::abs(stereo_disparities - std::round(stereo_disparities)) >
                1e-9) {
            return false;
        }
        options->projection.stereo_number_of_disparities =
            static_cast<int>(std::llround(stereo_disparities));
    }
    double stereo_block_size = options->projection.stereo_block_size;
    if (ReadNumber(settings, {"Atlas.Map.StereoBlockSize"},
                   &stereo_block_size)) {
        if (stereo_block_size < 3.0 ||
            stereo_block_size >
                static_cast<double>(std::numeric_limits<int>::max()) ||
            std::abs(stereo_block_size - std::round(stereo_block_size)) >
                1e-9) {
            return false;
        }
        options->projection.stereo_block_size =
            static_cast<int>(std::llround(stereo_block_size));
    }

    std::string grid_plane;
    if (ReadString(settings, {"Atlas.Map.GridPlane"}, &grid_plane)) {
        grid_plane = Uppercase(grid_plane);
        if (grid_plane == "XY") {
            options->occupancy_grid.projection_plane =
                map::OccupancyGridOptions::ProjectionPlane::kXy;
            if (options->occupancy_grid.up_axis !=
                    map::OccupancyGridOptions::UpAxis::kPositiveZ &&
                options->occupancy_grid.up_axis !=
                    map::OccupancyGridOptions::UpAxis::kNegativeZ) {
                options->occupancy_grid.up_axis =
                    map::OccupancyGridOptions::UpAxis::kPositiveZ;
            }
        } else if (grid_plane == "XZ") {
            options->occupancy_grid.projection_plane =
                map::OccupancyGridOptions::ProjectionPlane::kXz;
            if (options->occupancy_grid.up_axis !=
                    map::OccupancyGridOptions::UpAxis::kPositiveY &&
                options->occupancy_grid.up_axis !=
                    map::OccupancyGridOptions::UpAxis::kNegativeY) {
                options->occupancy_grid.up_axis =
                    map::OccupancyGridOptions::UpAxis::kPositiveY;
            }
        } else if (grid_plane == "YZ") {
            options->occupancy_grid.projection_plane =
                map::OccupancyGridOptions::ProjectionPlane::kYz;
            if (options->occupancy_grid.up_axis !=
                    map::OccupancyGridOptions::UpAxis::kPositiveX &&
                options->occupancy_grid.up_axis !=
                    map::OccupancyGridOptions::UpAxis::kNegativeX) {
                options->occupancy_grid.up_axis =
                    map::OccupancyGridOptions::UpAxis::kPositiveX;
            }
        }
    }
    std::string up_axis;
    if (ReadString(settings, {"Atlas.Map.UpAxis"}, &up_axis)) {
        up_axis = Uppercase(up_axis);
        if (up_axis == "+X" || up_axis == "X") {
            options->occupancy_grid.projection_plane =
                map::OccupancyGridOptions::ProjectionPlane::kYz;
            options->occupancy_grid.up_axis =
                map::OccupancyGridOptions::UpAxis::kPositiveX;
        } else if (up_axis == "-X") {
            options->occupancy_grid.projection_plane =
                map::OccupancyGridOptions::ProjectionPlane::kYz;
            options->occupancy_grid.up_axis =
                map::OccupancyGridOptions::UpAxis::kNegativeX;
        } else if (up_axis == "+Y" || up_axis == "Y") {
            options->occupancy_grid.projection_plane =
                map::OccupancyGridOptions::ProjectionPlane::kXz;
            options->occupancy_grid.up_axis =
                map::OccupancyGridOptions::UpAxis::kPositiveY;
        } else if (up_axis == "-Y") {
            options->occupancy_grid.projection_plane =
                map::OccupancyGridOptions::ProjectionPlane::kXz;
            options->occupancy_grid.up_axis =
                map::OccupancyGridOptions::UpAxis::kNegativeY;
        } else if (up_axis == "+Z" || up_axis == "Z") {
            options->occupancy_grid.projection_plane =
                map::OccupancyGridOptions::ProjectionPlane::kXy;
            options->occupancy_grid.up_axis =
                map::OccupancyGridOptions::UpAxis::kPositiveZ;
        } else if (up_axis == "-Z") {
            options->occupancy_grid.projection_plane =
                map::OccupancyGridOptions::ProjectionPlane::kXy;
            options->occupancy_grid.up_axis =
                map::OccupancyGridOptions::UpAxis::kNegativeZ;
        }
    }
    return camera->IsValid();
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

std::int64_t TimestampKey(double timestamp) {
    constexpr long double kNanosecondsPerSecond = 1000000000.0L;
    if (!std::isfinite(timestamp))
        return kInvalidTimestampKey;
    const long double value =
        std::round(static_cast<long double>(timestamp) *
                   kNanosecondsPerSecond);
    if (value <= static_cast<long double>(
                     std::numeric_limits<std::int64_t>::min()) ||
        value >= static_cast<long double>(
                     std::numeric_limits<std::int64_t>::max())) {
        return kInvalidTimestampKey;
    }
    return static_cast<std::int64_t>(value);
}

std::vector<map::TimedPose> BuildMappingPoseUpdates(
    const std::vector<TrajectoryPose>& trajectory,
    const std::unordered_map<std::int64_t, std::uint64_t>&
        submitted_frame_ids) {
    std::vector<map::TimedPose> optimized_poses;
    optimized_poses.reserve(
        std::min(trajectory.size(), submitted_frame_ids.size()));
    for (const TrajectoryPose& sample : trajectory) {
        const auto frame_id =
            submitted_frame_ids.find(TimestampKey(sample.time));
        if (frame_id == submitted_frame_ids.end())
            continue;
        map::TimedPose pose;
        pose.frame_id = frame_id->second;
        pose.timestamp = sample.time;
        pose.world_from_camera = ToIsometry(sample.pose_world_camera);
        optimized_poses.push_back(std::move(pose));
    }
    return optimized_poses;
}

struct MappingSubmissionResult {
    std::size_t matched_frames = 0;
    std::size_t submitted_frames = 0;
    std::size_t rejected_frames = 0;
    std::size_t new_unmatched_keyframes = 0;
    bool submission_blocked = false;
};

std::size_t MappingSubmissionBatchLimit(const SystemConfig& config) {
    return std::min(
        std::max<std::size_t>(
            1, config.maximum_mapping_keyframes_per_refresh),
        std::max<std::size_t>(1, config.mapping_options.queue_capacity));
}

void UpdateMonocularReference(
    const std::vector<TrajectoryPose>& trajectory, bool reset_if_missing,
    map::MappingFrame* reference, bool* have_reference) {
    if (reference == nullptr || have_reference == nullptr ||
        !*have_reference) {
        return;
    }
    const std::int64_t reference_key = TimestampKey(reference->timestamp);
    const auto updated_reference = std::find_if(
        trajectory.begin(), trajectory.end(),
        [reference_key](const TrajectoryPose& sample) {
            return TimestampKey(sample.time) == reference_key;
        });
    if (updated_reference != trajectory.end()) {
        reference->world_from_camera =
            ToIsometry(updated_reference->pose_world_camera);
    } else if (reset_if_missing) {
        *reference = map::MappingFrame();
        *have_reference = false;
    }
}

MappingSubmissionResult SubmitMatchingMappingKeyframes(
    const std::vector<TrajectoryPose>& trajectory,
    map::MappingPipeline* mapping,
    std::deque<map::MappingFrame>* pending_frames,
    std::unordered_map<std::int64_t, std::uint64_t>* submitted_frame_ids,
    map::MappingFrame* monocular_reference, bool* have_monocular_reference,
    std::uint64_t* next_monocular_frame_id,
    std::unordered_set<std::int64_t>* reported_unmatched_keys,
    std::size_t maximum_submissions) {
    MappingSubmissionResult result;
    if (mapping == nullptr || pending_frames == nullptr ||
        submitted_frame_ids == nullptr || monocular_reference == nullptr ||
        have_monocular_reference == nullptr ||
        next_monocular_frame_id == nullptr ||
        reported_unmatched_keys == nullptr) {
        return result;
    }
    maximum_submissions = std::max<std::size_t>(1, maximum_submissions);
    for (const TrajectoryPose& sample : trajectory) {
        if (result.submitted_frames >= maximum_submissions)
            break;
        const std::int64_t key = TimestampKey(sample.time);
        if (key == kInvalidTimestampKey ||
            submitted_frame_ids->count(key) != 0) {
            continue;
        }
        const auto pending = std::find_if(
            pending_frames->begin(), pending_frames->end(),
            [key](const map::MappingFrame& candidate) {
                return TimestampKey(candidate.timestamp) == key;
            });
        if (pending == pending_frames->end()) {
            if (reported_unmatched_keys->insert(key).second)
                ++result.new_unmatched_keyframes;
            continue;
        }

        // Keep the buffered candidate intact until the downstream pipeline
        // confirms that it accepted the frame. cv::Mat copies are shallow.
        map::MappingFrame frame = *pending;
        frame.world_from_camera = ToIsometry(sample.pose_world_camera);
        if (frame.depth_source == map::DepthSource::kMonocularTemporal) {
            if (!*have_monocular_reference) {
                frame.frame_id = *next_monocular_frame_id;
                *monocular_reference = frame;
                *have_monocular_reference = true;
                (*submitted_frame_ids)[key] = frame.frame_id;
                pending_frames->erase(pending);
                ++result.matched_frames;
                ++(*next_monocular_frame_id);
                continue;
            }
            frame.frame_id = *next_monocular_frame_id;
            frame.reference_frame_id = monocular_reference->frame_id;
            frame.reference = monocular_reference->color;
            frame.reference_from_camera =
                monocular_reference->world_from_camera.inverse() *
                frame.world_from_camera;
        }

        const bool monocular_temporal =
            frame.depth_source == map::DepthSource::kMonocularTemporal;
        map::MappingFrame next_monocular_reference;
        if (monocular_temporal)
            next_monocular_reference = frame;
        std::uint64_t assigned_frame_id = 0;
        const map::MappingPipeline::SubmitResult status =
            mapping->TrySubmit(&frame, &assigned_frame_id);
        if (status == map::MappingPipeline::SubmitResult::kQueueFull ||
            status == map::MappingPipeline::SubmitResult::kStopped) {
            result.submission_blocked = true;
            break;
        }
        if (status == map::MappingPipeline::SubmitResult::kInvalid) {
            pending_frames->erase(pending);
            ++result.matched_frames;
            ++result.rejected_frames;
            continue;
        }
        pending_frames->erase(pending);
        ++result.matched_frames;
        (*submitted_frame_ids)[key] = assigned_frame_id;
        if (monocular_temporal) {
            ++(*next_monocular_frame_id);
            next_monocular_reference.frame_id = assigned_frame_id;
            *monocular_reference = std::move(next_monocular_reference);
        }
        ++result.submitted_frames;
    }
    return result;
}

}  // namespace

namespace internal {

std::unordered_set<std::int64_t> UnsubmittedMappingKeyframeKeys(
    const std::vector<TrajectoryPose>& trajectory,
    const std::unordered_map<std::int64_t, std::uint64_t>&
        submitted_frame_ids) {
    std::unordered_set<std::int64_t> result;
    result.reserve(trajectory.size());
    for (const TrajectoryPose& sample : trajectory) {
        const std::int64_t key = TimestampKey(sample.time);
        if (key != kInvalidTimestampKey &&
            submitted_frame_ids.count(key) == 0) {
            result.insert(key);
        }
    }
    return result;
}

void ProtectPendingMappingKeyframes(
    const std::vector<double>& pending_timestamps,
    const std::unordered_map<std::int64_t, std::uint64_t>&
        submitted_frame_ids,
    std::unordered_set<std::int64_t>* protected_keys) {
    if (protected_keys == nullptr)
        return;
    protected_keys->reserve(protected_keys->size() +
                            pending_timestamps.size());
    for (const double timestamp : pending_timestamps) {
        const std::int64_t key = TimestampKey(timestamp);
        if (key != kInvalidTimestampKey &&
            submitted_frame_ids.count(key) == 0) {
            protected_keys->insert(key);
        }
    }
}

std::size_t DiscardStaleMappingCandidates(
    double oldest_useful_time,
    const std::unordered_set<std::int64_t>& protected_keys,
    std::deque<map::MappingFrame>* pending_frames) {
    if (pending_frames == nullptr)
        return 0;
    std::size_t discarded = 0;
    auto candidate = pending_frames->begin();
    while (candidate != pending_frames->end()) {
        const bool stale = candidate->timestamp < oldest_useful_time;
        const bool pending_keyframe =
            protected_keys.count(TimestampKey(candidate->timestamp)) != 0;
        if (!stale || pending_keyframe) {
            ++candidate;
            continue;
        }
        candidate = pending_frames->erase(candidate);
        ++discarded;
    }
    return discarded;
}

MappingCandidateSlotResult PrepareMappingCandidateSlot(
    std::size_t maximum_pending, double incoming_timestamp,
    const std::unordered_set<std::int64_t>& protected_keys,
    std::deque<map::MappingFrame>* pending_frames) {
    MappingCandidateSlotResult result;
    result.incoming_is_keyframe =
        protected_keys.count(TimestampKey(incoming_timestamp)) != 0;
    if (pending_frames == nullptr)
        return result;

    maximum_pending = std::max<std::size_t>(1, maximum_pending);
    result.eviction = EnforceMappingCandidateCapacity(
        maximum_pending - 1, protected_keys, pending_frames);
    result.have_slot = pending_frames->size() < maximum_pending;
    return result;
}

MappingCandidateBufferResult BufferMappingCandidateWithSoftCapacity(
    std::size_t maximum_pending,
    const std::unordered_set<std::int64_t>& protected_keys,
    map::MappingFrame frame,
    std::deque<map::MappingFrame>* pending_frames) {
    MappingCandidateBufferResult result;
    if (pending_frames == nullptr)
        return result;

    maximum_pending = std::max<std::size_t>(1, maximum_pending);
    result.eviction = EnforceMappingCandidateCapacity(
        maximum_pending - 1, protected_keys, pending_frames);
    result.soft_capacity_exceeded =
        pending_frames->size() >= maximum_pending;
    pending_frames->push_back(std::move(frame));
    return result;
}

MappingCandidateEvictionResult EnforceMappingCandidateCapacity(
    std::size_t maximum_pending,
    const std::unordered_set<std::int64_t>& protected_keys,
    std::deque<map::MappingFrame>* pending_frames) {
    if (pending_frames == nullptr)
        return {};
    MappingCandidateEvictionResult result;
    while (pending_frames->size() > maximum_pending) {
        auto candidate = std::find_if(
            pending_frames->begin(), pending_frames->end(),
            [&protected_keys](const map::MappingFrame& frame) {
                return protected_keys.count(TimestampKey(frame.timestamp)) ==
                       0;
            });
        if (candidate == pending_frames->end()) {
            result.saturated = true;
            break;
        }
        pending_frames->erase(candidate);
        ++result.unprotected_candidates;
    }
    return result;
}

std::vector<map::TimedPose> SelectChangedMappingPoses(
    const std::vector<TrajectoryPose>& trajectory,
    const std::unordered_map<std::int64_t, std::uint64_t>&
        submitted_frame_ids,
    const MappingPoseCache& last_sent_poses,
    double translation_threshold, double rotation_threshold_radians,
    bool force) {
    translation_threshold = std::max(0.0, translation_threshold);
    rotation_threshold_radians =
        std::max(0.0, rotation_threshold_radians);
    std::vector<map::TimedPose> updates;
    updates.reserve(std::min(trajectory.size(), submitted_frame_ids.size()));
    for (const TrajectoryPose& sample : trajectory) {
        const auto submitted =
            submitted_frame_ids.find(TimestampKey(sample.time));
        if (submitted == submitted_frame_ids.end())
            continue;
        const Eigen::Isometry3d pose = ToIsometry(sample.pose_world_camera);
        bool changed = force;
        const auto previous = last_sent_poses.find(submitted->second);
        if (previous == last_sent_poses.end()) {
            changed = true;
        } else if (!changed) {
            const double translation =
                (previous->second.translation() - pose.translation()).norm();
            Eigen::Quaterniond previous_rotation(previous->second.linear());
            Eigen::Quaterniond rotation(pose.linear());
            previous_rotation.normalize();
            rotation.normalize();
            const double dot = std::clamp(
                std::abs(previous_rotation.dot(rotation)), 0.0, 1.0);
            const double rotation_delta = 2.0 * std::acos(dot);
            const bool actually_changed =
                translation > std::numeric_limits<double>::epsilon() ||
                rotation_delta > std::numeric_limits<double>::epsilon();
            changed = actually_changed &&
                      (translation >= translation_threshold ||
                       rotation_delta >= rotation_threshold_radians);
        }
        if (!changed)
            continue;
        map::TimedPose update;
        update.frame_id = submitted->second;
        update.timestamp = sample.time;
        update.world_from_camera = pose;
        updates.push_back(std::move(update));
    }
    return updates;
}

void RecordSentMappingPoses(const std::vector<map::TimedPose>& poses,
                            MappingPoseCache* last_sent_poses) {
    if (last_sent_poses == nullptr)
        return;
    last_sent_poses->reserve(last_sent_poses->size() + poses.size());
    for (const map::TimedPose& pose : poses) {
        if (pose.frame_id != 0 && pose.world_from_camera.matrix().allFinite())
            last_sent_poses->insert_or_assign(pose.frame_id,
                                              pose.world_from_camera);
    }
}

bool ObserveMappingMap(bool map_id_valid, std::uint64_t map_id,
                       bool* observed_map_id_valid,
                       std::uint64_t* observed_map_id) {
    if (!map_id_valid || observed_map_id_valid == nullptr ||
        observed_map_id == nullptr) {
        return false;
    }
    if (!*observed_map_id_valid) {
        *observed_map_id_valid = true;
        *observed_map_id = map_id;
        return false;
    }
    const bool changed = *observed_map_id != map_id;
    *observed_map_id = map_id;
    return changed;
}

std::vector<CulledMappingFrame> SelectCulledMappingFrames(
    const std::vector<TrajectoryPose>& committed_trajectory,
    const std::vector<double>& pending_timestamps,
    const std::unordered_map<std::int64_t, std::uint64_t>&
        submitted_frame_ids,
    std::size_t missing_refresh_threshold,
    std::unordered_map<std::int64_t, std::size_t>*
        missing_refresh_counts) {
    if (missing_refresh_counts == nullptr)
        return {};
    missing_refresh_threshold =
        std::max<std::size_t>(1, missing_refresh_threshold);
    std::unordered_set<std::int64_t> active_keys;
    active_keys.reserve(
        committed_trajectory.size() + pending_timestamps.size());
    for (const TrajectoryPose& sample : committed_trajectory) {
        const std::int64_t key = TimestampKey(sample.time);
        if (key != kInvalidTimestampKey)
            active_keys.insert(key);
    }
    for (const double timestamp : pending_timestamps) {
        const std::int64_t key = TimestampKey(timestamp);
        if (key != kInvalidTimestampKey)
            active_keys.insert(key);
    }

    for (auto missing = missing_refresh_counts->begin();
         missing != missing_refresh_counts->end();) {
        if (submitted_frame_ids.count(missing->first) == 0U) {
            missing = missing_refresh_counts->erase(missing);
        } else {
            ++missing;
        }
    }

    std::vector<CulledMappingFrame> culled;
    for (const auto& [timestamp_key, frame_id] : submitted_frame_ids) {
        if (active_keys.count(timestamp_key) != 0U) {
            missing_refresh_counts->erase(timestamp_key);
            continue;
        }
        std::size_t& missing_count =
            (*missing_refresh_counts)[timestamp_key];
        if (missing_count < missing_refresh_threshold)
            ++missing_count;
        if (missing_count >= missing_refresh_threshold)
            culled.push_back({timestamp_key, frame_id});
    }
    return culled;
}

}  // namespace internal

class MapBuilder::Impl
{
public:
    explicit Impl(SystemConfig value) : config(std::move(value)) {}

    enum class PendingMappingDiscardReason {
        kStale,
        kCapacity,
        kProtectedKeyframeCapacity,
    };

    struct StoppedRuntime {
        std::unique_ptr<::autonomy::localization::atlas::System> system;
        std::unique_ptr<map::MappingPipeline> mapping;
        std::deque<map::MappingFrame> pending_mapping_frames;
        std::unordered_map<std::int64_t, std::uint64_t> submitted_frame_ids;
        std::unordered_set<std::int64_t> reported_unmatched_mapping_keys;
        map::MappingFrame monocular_reference;
        bool have_monocular_reference = false;
        std::uint64_t next_monocular_mapping_frame_id = 1;
        std::size_t submission_batch_limit = 1;
        std::uint64_t stale_candidates_discarded = 0;
        std::uint64_t capacity_candidates_discarded = 0;
        std::uint64_t protected_keyframes_capacity_dropped = 0;
        std::uint64_t unmatched_mapping_keyframes = 0;
    };

    StoppedRuntime TakeRuntime() {
        std::lock_guard<std::mutex> lock(mutex);
        StoppedRuntime runtime;
        runtime.system = std::move(system);
        runtime.mapping = std::move(mapping);
        runtime.pending_mapping_frames = std::move(pending_mapping_frames);
        runtime.submitted_frame_ids = std::move(submitted_frame_ids);
        runtime.reported_unmatched_mapping_keys =
            std::move(reported_unmatched_mapping_keys);
        runtime.monocular_reference = std::move(monocular_reference);
        runtime.have_monocular_reference = have_monocular_reference;
        runtime.next_monocular_mapping_frame_id =
            next_monocular_mapping_frame_id;
        runtime.submission_batch_limit = MappingSubmissionBatchLimit(config);
        runtime.stale_candidates_discarded =
            stale_candidates_discarded;
        runtime.capacity_candidates_discarded =
            capacity_candidates_discarded;
        runtime.protected_keyframes_capacity_dropped =
            protected_keyframes_capacity_dropped;
        runtime.unmatched_mapping_keyframes = unmatched_mapping_keyframes;
        imu.clear();
        pending_mapping_frames.clear();
        submitted_frame_ids.clear();
        reported_unmatched_mapping_keys.clear();
        protected_mapping_keyframe_keys.clear();
        last_sent_mapping_poses.clear();
        missing_submitted_keyframe_refreshes.clear();
        in_flight_mapping_frame_removals.clear();
        deferred_mapping_frame_removals.clear();
        mapping_map_id_valid = false;
        mapping_map_id = 0;
        monocular_reference = map::MappingFrame();
        have_monocular_reference = false;
        next_monocular_mapping_frame_id = 1;
        stale_candidates_discarded = 0;
        capacity_candidates_discarded = 0;
        protected_keyframes_capacity_dropped = 0;
        unmatched_mapping_keyframes = 0;
        last_mapping_refresh_time = -1.0;
        last_imu_time = -std::numeric_limits<double>::infinity();
        return runtime;
    }

    void RestoreStoppedMapping(
        std::unique_ptr<map::MappingPipeline> stopped_mapping) {
        if (!stopped_mapping)
            return;
        std::lock_guard<std::mutex> lock(mutex);
        if (!mapping)
            mapping = std::move(stopped_mapping);
    }

    static void ShutdownAndSynchronize(StoppedRuntime* runtime) {
        if (runtime == nullptr)
            return;
        std::vector<TrajectoryPose> trajectory;
        if (runtime->system) {
            runtime->system->Shutdown();
            trajectory = runtime->system->GetOptimizedKeyFrameTrajectory();
        }
        if (!runtime->mapping)
            return;
        if (!runtime->mapping->running()) {
            runtime->mapping->Stop();
            return;
        }

        if (!runtime->mapping->Flush())
            AWARN << "Atlas mapping worker did not drain before final keyframes";

        if (!trajectory.empty()) {
            UpdateMonocularReference(
                trajectory, true, &runtime->monocular_reference,
                &runtime->have_monocular_reference);
            while (!runtime->pending_mapping_frames.empty()) {
                const MappingSubmissionResult submitted =
                    SubmitMatchingMappingKeyframes(
                        trajectory, runtime->mapping.get(),
                        &runtime->pending_mapping_frames,
                        &runtime->submitted_frame_ids,
                        &runtime->monocular_reference,
                        &runtime->have_monocular_reference,
                        &runtime->next_monocular_mapping_frame_id,
                        &runtime->reported_unmatched_mapping_keys,
                        runtime->submission_batch_limit);
                runtime->unmatched_mapping_keyframes +=
                    submitted.new_unmatched_keyframes;
                if (submitted.rejected_frames > 0) {
                    AWARN << "Atlas mapping rejected "
                          << submitted.rejected_frames
                          << " final keyframe submissions";
                }
                if (submitted.matched_frames == 0)
                    break;
                if (submitted.submitted_frames > 0 &&
                    !runtime->mapping->Flush()) {
                    AWARN << "Atlas mapping worker did not flush a final "
                             "keyframe batch";
                    break;
                }
            }
            auto optimized_poses = BuildMappingPoseUpdates(
                trajectory, runtime->submitted_frame_ids);
            if (!optimized_poses.empty() &&
                !runtime->mapping->UpdatePoses(
                    std::move(optimized_poses))) {
                AWARN << "Atlas mapping rejected final optimized poses";
            }
        }
        if (!runtime->pending_mapping_frames.empty()) {
            AINFO << "Atlas shutdown discarded "
                  << runtime->pending_mapping_frames.size()
                  << " pending image candidates that were not promoted to "
                     "keyframes";
        }
        if (runtime->stale_candidates_discarded > 0 ||
            runtime->capacity_candidates_discarded > 0) {
            AINFO << "Atlas dense mapping pending candidate summary: "
                  << "stale_discarded="
                  << runtime->stale_candidates_discarded
                  << ", capacity_discarded="
                  << runtime->capacity_candidates_discarded;
        }
        if (!runtime->mapping->Flush())
            AWARN << "Atlas mapping worker did not flush during shutdown";
        const map::MappingPipelineStats stats = runtime->mapping->stats();
        const std::uint64_t nonduplicate_rejected =
            stats.rejected_frames >= stats.duplicate_frames
                ? stats.rejected_frames - stats.duplicate_frames
                : stats.rejected_frames;
        AINFO << "Atlas dense mapping pipeline summary: submitted="
              << stats.submitted_frames << ", processed="
              << stats.processed_frames << ", dropped="
              << stats.dropped_frames << ", gated=" << stats.gated_frames
              << ", rejected=" << stats.rejected_frames
              << ", duplicate=" << stats.duplicate_frames
              << ", removed=" << stats.removed_frames
              << ", removal_rebuild_failures="
              << stats.removal_rebuild_failures
              << ", retriangulated=" << stats.retriangulated_frames
              << ", skipped_incomplete_rebuilds="
              << stats.skipped_incomplete_rebuilds
              << ", history_archive_failures="
              << stats.history_archive_failures;
        if (runtime->unmatched_mapping_keyframes > 0 ||
            runtime->protected_keyframes_capacity_dropped > 0 ||
            stats.dropped_frames > 0 || nonduplicate_rejected > 0 ||
            stats.history_archive_failures > 0 ||
            stats.removal_rebuild_failures > 0) {
            AWARN << "Atlas dense mapping coverage loss: unmatched_keyframes="
                  << runtime->unmatched_mapping_keyframes
                  << ", protected_capacity_dropped="
                  << runtime->protected_keyframes_capacity_dropped
                  << ", queue_dropped=" << stats.dropped_frames
                  << ", nonduplicate_rejected=" << nonduplicate_rejected
                  << ", history_archive_failures="
                  << stats.history_archive_failures
                  << ", removal_rebuild_failures="
                  << stats.removal_rebuild_failures;
        }
        runtime->mapping->Stop();
    }

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
        latest.map_id_valid =
            system != nullptr && system->GetCurrentMapId(&latest.map_id);

        latest.tracking_state = system ? system->GetTrackingState() : 0;
        latest.valid = latest.tracking_state == 2 || latest.tracking_state == 5;
        SynchronizeMappingMap();
        if (latest.valid && camera_from_world.matrix().allFinite()) {
            latest.pose_world_camera = ToWorldCamera(camera_from_world);
        } else {
            latest.valid = false;
        }
        if (output != nullptr)
            *output = latest;
    }

    void SynchronizeMappingMap() {
        if (!mapping ||
            !internal::ObserveMappingMap(
                latest.map_id_valid, latest.map_id,
                &mapping_map_id_valid, &mapping_map_id)) {
            return;
        }
        const std::uint64_t new_map_id = mapping_map_id;
        AINFO << "Atlas dense mapping switched to map id " << new_map_id
              << "; resetting independent map coordinates";
        ResetMappingState();
        mapping_map_id_valid = true;
        mapping_map_id = new_map_id;
    }

    void QueueMappingFrame(map::MappingFrame frame) {
        if (!mapping || !latest.valid)
            return;
        if (system != nullptr) {
            internal::ProtectPendingMappingKeyframes(
                system->GetPendingMappingKeyFrameTimestamps(),
                submitted_frame_ids, &protected_mapping_keyframe_keys);
        }

        const std::size_t maximum_pending = std::max<std::size_t>(
            1, config.maximum_pending_mapping_frames);
        const std::int64_t incoming_key = TimestampKey(frame.timestamp);
        auto refresh_before_admission = [this, incoming_key,
                                         timestamp = frame.timestamp]() {
            // The incoming image is not buffered yet. Suppress a transient
            // unmatched report if LocalMapper committed it just before this
            // forced refresh; a genuine later loss remains reportable.
            const bool suppress_unmatched =
                incoming_key != kInvalidTimestampKey &&
                reported_unmatched_mapping_keys.insert(incoming_key).second;
            RefreshMappingKeyframes(timestamp, true);
            if (suppress_unmatched)
                reported_unmatched_mapping_keys.erase(incoming_key);
        };
        if (pending_mapping_frames.size() >= maximum_pending) {
            // The initial pending-keyframe snapshot can race with LocalMapper
            // committing the incoming keyframe. Refresh before any eviction,
            // then classify the buffered union before admitting this frame.
            refresh_before_admission();
        }
        internal::MappingCandidateBufferResult buffered =
            internal::BufferMappingCandidateWithSoftCapacity(
                maximum_pending, protected_mapping_keyframe_keys,
                std::move(frame), &pending_mapping_frames);
        for (std::size_t index = 0;
             index < buffered.eviction.unprotected_candidates; ++index) {
            RecordPendingMappingDiscard(
                PendingMappingDiscardReason::kCapacity);
        }
        // Do not synchronously Flush here. If every resident candidate is a
        // tentative/committed keyframe, keeping speculative overflow is
        // the only lossless non-blocking choice. A later refresh either
        // protects it or evicts it as an ordinary candidate.
    }

    void RecordPendingMappingDiscard(PendingMappingDiscardReason reason) {
        std::uint64_t* count = &stale_candidates_discarded;
        const char* label = "stale";
        std::uint64_t log_period = 1000;
        switch (reason) {
            case PendingMappingDiscardReason::kStale:
                break;
            case PendingMappingDiscardReason::kCapacity:
                count = &capacity_candidates_discarded;
                label = "capacity";
                log_period = 100;
                break;
            case PendingMappingDiscardReason::kProtectedKeyframeCapacity:
                count = &protected_keyframes_capacity_dropped;
                label = "capacity-keyframe";
                log_period = 1;
                break;
        }
        const std::uint64_t previous = (*count)++;
        if (previous != 0 && previous / log_period == *count / log_period)
            return;
        if (reason == PendingMappingDiscardReason::kStale) {
            AINFO << "Atlas dense mapping discarded stale unresolved "
                     "candidates: count="
                  << *count;
        } else {
            AWARN << "Atlas dense mapping discarded pending candidates: count="
                  << *count << ", reason=" << label;
        }
    }

    void RecordMappingSubmissionResult(
        const MappingSubmissionResult& result) {
        const std::uint64_t previous = unmatched_mapping_keyframes;
        unmatched_mapping_keyframes += result.new_unmatched_keyframes;
        if (result.new_unmatched_keyframes > 0 &&
            (previous == 0 || previous / 100 !=
                                  unmatched_mapping_keyframes / 100)) {
            AWARN << "Atlas dense mapping has optimized keyframes without "
                     "pending images: count="
                  << unmatched_mapping_keyframes;
        }
        if (result.rejected_frames > 0) {
            AWARN << "Atlas dense mapping rejected " << result.rejected_frames
                  << " keyframe submissions";
        }
    }

    void DrainMappingFrameRemovalResults() {
        const map::MappingFrameRemovalResults results =
            mapping->TakeFrameRemovalResults();
        if (results.removed_frame_ids.empty() &&
            results.retained_frame_ids.empty()) {
            return;
        }

        const std::unordered_set<std::uint64_t> removed_ids(
            results.removed_frame_ids.begin(),
            results.removed_frame_ids.end());
        for (auto submitted = submitted_frame_ids.begin();
             submitted != submitted_frame_ids.end();) {
            if (removed_ids.count(submitted->second) == 0U) {
                ++submitted;
                continue;
            }
            missing_submitted_keyframe_refreshes.erase(submitted->first);
            last_sent_mapping_poses.erase(submitted->second);
            submitted = submitted_frame_ids.erase(submitted);
        }
        for (const std::uint64_t frame_id : results.removed_frame_ids) {
            in_flight_mapping_frame_removals.erase(frame_id);
            deferred_mapping_frame_removals.erase(frame_id);
            last_sent_mapping_poses.erase(frame_id);
        }

        // A successful dependent removal may make an earlier monocular
        // reference removable. Retry old deferrals, but keep failures from
        // this completion deferred until the map changes again.
        if (!results.removed_frame_ids.empty())
            deferred_mapping_frame_removals.clear();
        for (const std::uint64_t frame_id : results.retained_frame_ids) {
            in_flight_mapping_frame_removals.erase(frame_id);
            deferred_mapping_frame_removals.insert(frame_id);
        }
    }

    void QueueCulledMappingFrames(
        const std::vector<internal::CulledMappingFrame>& culled) {
        std::vector<std::uint64_t> frame_ids;
        frame_ids.reserve(culled.size());
        for (const internal::CulledMappingFrame& frame : culled) {
            if (in_flight_mapping_frame_removals.count(frame.frame_id) != 0U ||
                deferred_mapping_frame_removals.count(frame.frame_id) != 0U) {
                continue;
            }
            frame_ids.push_back(frame.frame_id);
        }
        if (frame_ids.empty())
            return;
        const std::vector<std::uint64_t> accepted =
            mapping->RemoveFrames(std::move(frame_ids));
        in_flight_mapping_frame_removals.insert(accepted.begin(),
                                                accepted.end());
    }

    void RefreshMappingKeyframes(double timestamp, bool force = false) {
        if (!mapping || !system || !latest.valid)
            return;
        const bool map_changed = system->MapChanged();
        const double period =
            std::max(0.0, config.mapping_keyframe_poll_period_sec);
        const bool refresh_due =
            force || map_changed || last_mapping_refresh_time < 0.0 ||
            timestamp < last_mapping_refresh_time || period <= 0.0 ||
            timestamp - last_mapping_refresh_time >= period;
        if (!refresh_due)
            return;
        last_mapping_refresh_time = timestamp;
        DrainMappingFrameRemovalResults();
        if (map_changed)
            deferred_mapping_frame_removals.clear();

        // Snapshot pending first and committed second. LocalMapper moves a
        // keyframe in that direction, so this order cannot miss a keyframe
        // that changes state between the two calls.
        const std::vector<double> pending_keyframe_timestamps =
            system->GetPendingMappingKeyFrameTimestamps();
        // Only committed, non-bad keyframes may enter the dense map. Pending
        // LocalMapper keyframes are tentative and are used solely to protect
        // their image candidates from eviction.
        const std::vector<TrajectoryPose> trajectory =
            system->GetOptimizedKeyFrameTrajectory();
        for (const TrajectoryPose& sample : trajectory) {
            const auto submitted =
                submitted_frame_ids.find(TimestampKey(sample.time));
            if (submitted != submitted_frame_ids.end())
                deferred_mapping_frame_removals.erase(submitted->second);
        }
        for (const double pending_timestamp : pending_keyframe_timestamps) {
            const auto submitted = submitted_frame_ids.find(
                TimestampKey(pending_timestamp));
            if (submitted != submitted_frame_ids.end())
                deferred_mapping_frame_removals.erase(submitted->second);
        }
        const std::vector<internal::CulledMappingFrame> culled =
            internal::SelectCulledMappingFrames(
                trajectory, pending_keyframe_timestamps,
                submitted_frame_ids, 2U,
                &missing_submitted_keyframe_refreshes);
        QueueCulledMappingFrames(culled);
        if (trajectory.empty()) {
            protected_mapping_keyframe_keys.clear();
            internal::ProtectPendingMappingKeyframes(
                pending_keyframe_timestamps, submitted_frame_ids,
                &protected_mapping_keyframe_keys);
            return;
        }

        UpdateMonocularReference(trajectory, true,
                                 &monocular_reference,
                                 &have_monocular_reference);
        const MappingSubmissionResult submitted =
            SubmitMatchingMappingKeyframes(
                trajectory, mapping.get(), &pending_mapping_frames,
                &submitted_frame_ids, &monocular_reference,
                &have_monocular_reference,
                &next_monocular_mapping_frame_id,
                &reported_unmatched_mapping_keys,
                MappingSubmissionBatchLimit(config));
        RecordMappingSubmissionResult(submitted);

        const std::vector<map::TimedPose> pose_updates =
            internal::SelectChangedMappingPoses(
                trajectory, submitted_frame_ids, last_sent_mapping_poses,
                config.mapping_options.rebuild_translation_threshold,
                config.mapping_options.rebuild_rotation_threshold_radians,
                map_changed);
        if (!pose_updates.empty()) {
            if (mapping->UpdatePoses(pose_updates)) {
                internal::RecordSentMappingPoses(
                    pose_updates, &last_sent_mapping_poses);
            } else {
                AWARN << "Atlas mapping rejected optimized pose updates";
            }
        }
        if (map_changed && !mapping->RequestRebuild()) {
            AWARN << "Atlas mapping rejected loop-closure rebuild request";
        }

        const double oldest_useful_time = trajectory.back().time - 3.0;
        protected_mapping_keyframe_keys =
            internal::UnsubmittedMappingKeyframeKeys(
                trajectory, submitted_frame_ids);
        internal::ProtectPendingMappingKeyframes(
            pending_keyframe_timestamps, submitted_frame_ids,
            &protected_mapping_keyframe_keys);
        const std::size_t stale_discarded =
            internal::DiscardStaleMappingCandidates(
                oldest_useful_time, protected_mapping_keyframe_keys,
                &pending_mapping_frames);
        for (std::size_t index = 0; index < stale_discarded; ++index)
            RecordPendingMappingDiscard(PendingMappingDiscardReason::kStale);
        const internal::MappingCandidateEvictionResult capacity_evicted =
            internal::EnforceMappingCandidateCapacity(
                config.maximum_pending_mapping_frames,
                protected_mapping_keyframe_keys,
                &pending_mapping_frames);
        for (std::size_t index = 0;
             index < capacity_evicted.unprotected_candidates; ++index) {
            RecordPendingMappingDiscard(
                PendingMappingDiscardReason::kCapacity);
        }
        if (capacity_evicted.saturated && !submitted.submission_blocked) {
            // QueueMappingFrame normally prevents an oversized buffer. A
            // forced refresh gets one additional chance to free a protected
            // slot without ever deleting the protected candidate.
            const MappingSubmissionResult retried =
                SubmitMatchingMappingKeyframes(
                    trajectory, mapping.get(), &pending_mapping_frames,
                    &submitted_frame_ids, &monocular_reference,
                    &have_monocular_reference,
                    &next_monocular_mapping_frame_id,
                    &reported_unmatched_mapping_keys,
                    MappingSubmissionBatchLimit(config));
            RecordMappingSubmissionResult(retried);
        }
    }

    void ResetMappingState() {
        pending_mapping_frames.clear();
        submitted_frame_ids.clear();
        reported_unmatched_mapping_keys.clear();
        protected_mapping_keyframe_keys.clear();
        missing_submitted_keyframe_refreshes.clear();
        in_flight_mapping_frame_removals.clear();
        deferred_mapping_frame_removals.clear();
        monocular_reference = map::MappingFrame();
        have_monocular_reference = false;
        next_monocular_mapping_frame_id = 1;
        last_sent_mapping_poses.clear();
        mapping_map_id_valid = false;
        mapping_map_id = 0;
        stale_candidates_discarded = 0;
        capacity_candidates_discarded = 0;
        protected_keyframes_capacity_dropped = 0;
        unmatched_mapping_keyframes = 0;
        last_mapping_refresh_time = -1.0;
        if (mapping)
            mapping->Reset();
    }

    SystemConfig config;
    mutable std::mutex mutex;
    std::unique_ptr<::autonomy::localization::atlas::System> system;
    std::unique_ptr<map::MappingPipeline> mapping;
    map::CameraModel mapping_camera;
    map::CameraModel secondary_mapping_camera;
    Eigen::Isometry3d secondary_from_camera = Eigen::Isometry3d::Identity();
    std::deque<map::MappingFrame> pending_mapping_frames;
    std::unordered_map<std::int64_t, std::uint64_t> submitted_frame_ids;
    std::unordered_set<std::int64_t> reported_unmatched_mapping_keys;
    std::unordered_set<std::int64_t> protected_mapping_keyframe_keys;
    std::unordered_map<std::int64_t, std::size_t>
        missing_submitted_keyframe_refreshes;
    std::unordered_set<std::uint64_t> in_flight_mapping_frame_removals;
    std::unordered_set<std::uint64_t> deferred_mapping_frame_removals;
    map::MappingFrame monocular_reference;
    bool have_monocular_reference = false;
    std::uint64_t next_monocular_mapping_frame_id = 1;
    internal::MappingPoseCache last_sent_mapping_poses;
    bool mapping_map_id_valid = false;
    std::uint64_t mapping_map_id = 0;
    std::uint64_t stale_candidates_discarded = 0;
    std::uint64_t capacity_candidates_discarded = 0;
    std::uint64_t protected_keyframes_capacity_dropped = 0;
    std::uint64_t unmatched_mapping_keyframes = 0;
    std::deque<ImuSample> imu;
    TrackingResult latest;
    double last_imu_time = -std::numeric_limits<double>::infinity();
    double last_mapping_refresh_time = -1.0;
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

    if (impl_->config.enable_dense_mapping) {
        map::MappingPipelineOptions mapping_options =
            impl_->config.mapping_options;
        if (IsInertial(impl_->config.sensor)) {
            mapping_options.occupancy_grid.projection_plane =
                map::OccupancyGridOptions::ProjectionPlane::kXy;
            mapping_options.occupancy_grid.up_axis =
                map::OccupancyGridOptions::UpAxis::kPositiveZ;
        } else {
            mapping_options.occupancy_grid.projection_plane =
                map::OccupancyGridOptions::ProjectionPlane::kXz;
            mapping_options.occupancy_grid.up_axis =
                map::OccupancyGridOptions::UpAxis::kNegativeY;
            // The non-inertial world starts at the first camera, so a floor
            // commonly lies about 1-2 m below y=0 instead of around z=0.
            if (std::abs(mapping_options.occupancy_grid
                             .minimum_obstacle_height +
                         0.5) < 1e-12 &&
                std::abs(mapping_options.occupancy_grid
                             .maximum_obstacle_height -
                         2.0) < 1e-12) {
                mapping_options.occupancy_grid.minimum_obstacle_height = -2.5;
            }
        }
        mapping_options.queue_capacity =
            std::max<std::size_t>(4, mapping_options.queue_capacity);
        map::CameraModel camera;
        map::CameraModel secondary_camera;
        Eigen::Isometry3d secondary_from_camera =
            Eigen::Isometry3d::Identity();
        if (!LoadMappingSettings(impl_->config, &camera, &secondary_camera,
                                 &secondary_from_camera, &mapping_options)) {
            AWARN << "Atlas dense mapping disabled: invalid camera intrinsics "
                     "in "
                  << impl_->config.settings_path;
        } else if (IsStereo(impl_->config.sensor) &&
                   camera.stereo_baseline <= 0.0) {
            AWARN << "Atlas dense stereo mapping disabled: Stereo.b or "
                     "Camera.bf is missing";
        } else {
            try {
                impl_->config.mapping_options = mapping_options;
                impl_->mapping_camera = camera;
                impl_->secondary_mapping_camera = secondary_camera;
                impl_->secondary_from_camera = secondary_from_camera;
                impl_->mapping =
                    std::make_unique<map::MappingPipeline>(mapping_options);
                if (!impl_->mapping->Start()) {
                    AWARN << "Atlas dense mapping worker did not start";
                    impl_->mapping.reset();
                } else {
                    if (!mapping_options.occupancy_grid.use_sensor_height) {
                        AWARN << "Atlas.Map.SensorHeight is not configured; "
                                 "occupancy uses a conservative sensor-relative "
                                 "fallback and navigation height filtering is "
                                 "not ground-relative. Production navigation "
                                 "must configure SensorHeight";
                    }
                    if (impl_->config.sensor == SensorMode::kMonocular) {
                        AWARN << "Atlas pure-monocular map uses the visual-SLAM "
                                 "relative scale; occupancy resolution is not "
                                 "metric until an external scale is supplied";
                    }
                }
            } catch (const std::exception& error) {
                AWARN << "Atlas dense mapping disabled: " << error.what();
                impl_->mapping.reset();
            }
        }
    }
    return true;
}

void MapBuilder::Stop() {
    auto runtime = impl_->TakeRuntime();
    Impl::ShutdownAndSynchronize(&runtime);
}

bool MapBuilder::ShutdownAndSaveTumTrajectory(const std::string& path) {
    if (path.empty())
        return false;
    auto runtime = impl_->TakeRuntime();
    if (!runtime.system) {
        impl_->RestoreStoppedMapping(std::move(runtime.mapping));
        return false;
    }
    Impl::ShutdownAndSynchronize(&runtime);
    // Truncate first so a failed/empty save cannot be mistaken for a stale
    // trajectory left by a previous run.
    std::ofstream cleared(path, std::ios::out | std::ios::trunc);
    if (!cleared) {
        impl_->RestoreStoppedMapping(std::move(runtime.mapping));
        return false;
    }
    cleared.close();
    runtime.system->SaveTrajectoryTUM(path);
    std::ifstream saved(path);
    const bool result =
        saved.good() && saved.peek() != std::ifstream::traits_type::eof();
    impl_->RestoreStoppedMapping(std::move(runtime.mapping));
    return result;
}

bool MapBuilder::ShutdownAndSaveEurocTrajectory(const std::string& path) {
    if (path.empty())
        return false;
    auto runtime = impl_->TakeRuntime();
    if (!runtime.system) {
        impl_->RestoreStoppedMapping(std::move(runtime.mapping));
        return false;
    }
    Impl::ShutdownAndSynchronize(&runtime);
    runtime.system->SaveTrajectoryEuRoC(path);
    std::ifstream saved(path);
    const bool result =
        saved.good() && saved.peek() != std::ifstream::traits_type::eof();
    impl_->RestoreStoppedMapping(std::move(runtime.mapping));
    return result;
}

bool MapBuilder::ShutdownAndSaveKittiTrajectory(const std::string& path) {
    if (path.empty())
        return false;
    auto runtime = impl_->TakeRuntime();
    if (!runtime.system) {
        impl_->RestoreStoppedMapping(std::move(runtime.mapping));
        return false;
    }
    Impl::ShutdownAndSynchronize(&runtime);
    runtime.system->SaveTrajectoryKITTI(path);
    std::ifstream saved(path);
    const bool result =
        saved.good() && saved.peek() != std::ifstream::traits_type::eof();
    impl_->RestoreStoppedMapping(std::move(runtime.mapping));
    return result;
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
    const bool metric_scale_ready =
        impl_->config.sensor != SensorMode::kMonocularInertial ||
        impl_->system->isFinished();
    if (impl_->mapping && impl_->latest.valid && metric_scale_ready) {
        map::MappingFrame frame;
        frame.timestamp = image.time;
        frame.world_from_camera = ToIsometry(impl_->latest.pose_world_camera);
        frame.camera = impl_->mapping_camera;
        frame.depth_source = map::DepthSource::kMonocularTemporal;
        frame.color_encoding = ToColorEncoding(image.encoding);
        frame.color = image.pixels;
        impl_->QueueMappingFrame(std::move(frame));
        impl_->RefreshMappingKeyframes(image.time);
    }
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
    if (impl_->mapping && impl_->latest.valid) {
        map::MappingFrame frame;
        frame.timestamp = time;
        frame.world_from_camera = ToIsometry(impl_->latest.pose_world_camera);
        frame.camera = impl_->mapping_camera;
        frame.secondary_camera = impl_->secondary_mapping_camera;
        frame.secondary_from_camera = impl_->secondary_from_camera;
        frame.depth_source = map::DepthSource::kStereo;
        frame.color_encoding = ToColorEncoding(left.encoding);
        frame.color = left.pixels;
        frame.right = right.pixels;
        impl_->QueueMappingFrame(std::move(frame));
        impl_->RefreshMappingKeyframes(time);
    }
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
    if (impl_->mapping && impl_->latest.valid) {
        map::MappingFrame frame;
        frame.timestamp = time;
        frame.world_from_camera = ToIsometry(impl_->latest.pose_world_camera);
        frame.camera = impl_->mapping_camera;
        frame.depth_source = map::DepthSource::kRgbd;
        frame.color_encoding = ToColorEncoding(color.encoding);
        frame.color = color.pixels;
        frame.depth = depth.pixels;
        impl_->QueueMappingFrame(std::move(frame));
        impl_->RefreshMappingKeyframes(time);
    }
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

cv::Mat MapBuilder::TrackingImage() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->system)
        return cv::Mat();
    return impl_->system->DrawCurrentFrame();
}

cv::Mat MapBuilder::FeatureMatchesImage() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->system)
        return cv::Mat();
    return impl_->system->DrawFeatureMatches();
}

std::vector<TrajectoryPose> MapBuilder::OptimizedKeyFrameTrajectory() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->system)
        return {};
    return impl_->system->GetOptimizedKeyFrameTrajectory();
}

std::vector<LoopClosureEdge> MapBuilder::LoopClosureEdges() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->system)
        return {};
    return impl_->system->GetLoopClosureEdges();
}

std::shared_ptr<const map::MapSnapshot> MapBuilder::LatestMappingSnapshot()
    const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->mapping)
        return nullptr;
    return impl_->mapping->LatestSnapshot();
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
    impl_->ResetMappingState();
}

}  // namespace autonomy::localization::atlas
