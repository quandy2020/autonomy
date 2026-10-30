/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#include "atlas/map/mapping_pipeline.hpp"

#include "autolink/common/log.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <system_error>
#include <type_traits>
#include <unordered_set>
#include <utility>

#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace autonomy::localization::atlas::map {
namespace {
namespace fs = std::filesystem;

bool FiniteTransform(const Eigen::Isometry3d& transform) {
    return transform.matrix().allFinite();
}

bool ValidCameraProjection(CameraProjection projection) {
    switch (projection) {
        case CameraProjection::kRectified:
        case CameraProjection::kPinhole:
        case CameraProjection::kKannalaBrandt8:
            return true;
        case CameraProjection::kUnknown:
            return false;
    }
    return false;
}

double RotationAngle(const Eigen::Isometry3d& first,
                     const Eigen::Isometry3d& second) {
    const Eigen::Matrix3d relative =
        first.linear().transpose() * second.linear();
    return Eigen::AngleAxisd(relative).angle();
}

void RetainPointsWithinRadius(
    const Eigen::Vector3f& center, double radius,
    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>>* points) {
    if (points == nullptr || radius <= 0.0)
        return;
    const Eigen::Vector3d center_double = center.cast<double>();
    const double squared_radius = radius * radius;
    points->erase(
        std::remove_if(points->begin(), points->end(),
                       [&center_double, squared_radius](const ColorPoint& point) {
                           const Eigen::Vector3d delta =
                               point.position.cast<double>() - center_double;
                           return !delta.allFinite() ||
                                  delta.squaredNorm() > squared_radius;
                       }),
        points->end());
}

DenseMapOptions NavigationMapOptions(const MappingPipelineOptions& options) {
    DenseMapOptions navigation = options.dense_map;
    navigation.maximum_voxels = options.maximum_navigation_voxels;
    navigation.reserve_voxels =
        std::min(navigation.reserve_voxels, navigation.maximum_voxels);
    return navigation;
}

constexpr std::array<char, 8> kHistoryArchiveMagic = {
    'A', 'T', 'L', 'A', 'S', 'H', '1', '\0'};
constexpr std::uint32_t kHistoryArchiveVersion = 1;
constexpr std::uint64_t kHistoryArchiveHeaderBytes =
    kHistoryArchiveMagic.size() + sizeof(std::uint32_t) +
    3U * sizeof(std::uint64_t);
constexpr std::uint64_t kMaximumEncodedImageBytes =
    256ULL * 1024ULL * 1024ULL;
constexpr char kHistoryDirectoryPrefix[] = "atlas-map-history-";
std::atomic<std::uint64_t> g_history_directory_counter{0};

struct ArchivedPoint {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    std::uint8_t red = 0;
    std::uint8_t green = 0;
    std::uint8_t blue = 0;
    std::uint8_t reserved = 0;
};
static_assert(std::is_trivially_copyable_v<ArchivedPoint>);

template <typename T>
bool WriteValue(std::ostream* stream, const T& value) {
    static_assert(std::is_trivially_copyable_v<T>);
    if (stream == nullptr)
        return false;
    stream->write(reinterpret_cast<const char*>(&value), sizeof(value));
    return stream->good();
}

template <typename T>
bool ReadValue(std::istream* stream, T* value) {
    static_assert(std::is_trivially_copyable_v<T>);
    if (stream == nullptr || value == nullptr)
        return false;
    stream->read(reinterpret_cast<char*>(value), sizeof(*value));
    return stream->good();
}

bool EncodePng(const cv::Mat& image, std::vector<std::uint8_t>* encoded) {
    if (encoded == nullptr || image.empty())
        return false;
    try {
        return cv::imencode(".png", image, *encoded) && !encoded->empty();
    } catch (const cv::Exception&) {
        return false;
    }
}

bool DecodePng(const std::vector<std::uint8_t>& encoded, cv::Mat* image) {
    if (image == nullptr || encoded.empty())
        return false;
    try {
        *image = cv::imdecode(encoded, cv::IMREAD_UNCHANGED);
    } catch (const cv::Exception&) {
        return false;
    }
    return !image->empty();
}

bool ToGray8(const cv::Mat& image, ColorEncoding encoding, cv::Mat* gray) {
    if (gray == nullptr || image.empty())
        return false;
    if (image.type() == CV_8UC1) {
        *gray = image;
        return true;
    }
    if (image.type() != CV_8UC3 && image.type() != CV_8UC4)
        return false;
    const bool rgb = encoding == ColorEncoding::kRgb;
    const int code = image.channels() == 3
                         ? (rgb ? cv::COLOR_RGB2GRAY : cv::COLOR_BGR2GRAY)
                         : (rgb ? cv::COLOR_RGBA2GRAY : cv::COLOR_BGRA2GRAY);
    cv::cvtColor(image, *gray, code);
    return !gray->empty();
}

bool ComputeDisparity(const cv::Mat& left, const cv::Mat& right,
                      const ProjectionOptions& options,
                      int minimum_disparity, cv::Mat* disparity) {
    if (disparity == nullptr || left.empty() || right.empty() ||
        left.type() != CV_8UC1 || right.type() != CV_8UC1 ||
        left.size() != right.size()) {
        return false;
    }
    int disparities = options.stereo_number_of_disparities;
    disparities = std::max(16, (disparities + 15) / 16 * 16);
    int block_size = std::max(3, options.stereo_block_size);
    if (block_size % 2 == 0)
        ++block_size;
    try {
        cv::Ptr<cv::StereoSGBM> matcher = cv::StereoSGBM::create(
            minimum_disparity, disparities, block_size,
            8 * block_size * block_size, 32 * block_size * block_size, 1, 31,
            10, 100, 2, cv::StereoSGBM::MODE_SGBM_3WAY);
        matcher->compute(left, right, *disparity);
    } catch (const cv::Exception&) {
        return false;
    }
    return !disparity->empty() && disparity->type() == CV_16SC1;
}

cv::Mat CameraMatrix(const CameraModel& camera) {
    cv::Mat result = cv::Mat::eye(3, 3, CV_64F);
    result.at<double>(0, 0) = camera.fx;
    result.at<double>(0, 2) = camera.cx;
    result.at<double>(1, 1) = camera.fy;
    result.at<double>(1, 2) = camera.cy;
    return result;
}

cv::Mat Distortion(const CameraModel& camera) {
    const int count = camera.projection == CameraProjection::kKannalaBrandt8
                          ? 4
                          : 5;
    cv::Mat distortion(count, 1, CV_64F);
    for (int index = 0; index < count; ++index) {
        distortion.at<double>(index, 0) =
            camera.projection == CameraProjection::kRectified
                ? 0.0
                : camera.distortion[index];
    }
    return distortion;
}

cv::Mat RotationMatrix(const Eigen::Matrix3d& value) {
    cv::Mat result(3, 3, CV_64F);
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column)
            result.at<double>(row, column) = value(row, column);
    }
    return result;
}

cv::Mat TranslationVector(const Eigen::Vector3d& value) {
    cv::Mat result(3, 1, CV_64F);
    result.at<double>(0, 0) = value.x();
    result.at<double>(1, 0) = value.y();
    result.at<double>(2, 0) = value.z();
    return result;
}

struct StereoRectification {
    cv::Mat primary_gray;
    cv::Mat secondary_gray;
    cv::Mat primary_color;
    cv::Mat disparity_to_depth;
    cv::Mat primary_to_rectified;
    int minimum_disparity = 0;
};

bool RectifyStereoPair(
    const cv::Mat& primary_image, const cv::Mat& secondary_image,
    ColorEncoding encoding, const CameraModel& primary_camera,
    const CameraModel& secondary_camera,
    const Eigen::Isometry3d& secondary_from_primary,
    const ProjectionOptions& options, StereoRectification* rectified) {
    if (rectified == nullptr || primary_image.empty() ||
        secondary_image.empty() ||
        primary_image.size() != secondary_image.size() ||
        primary_camera.projection != secondary_camera.projection ||
        primary_camera.projection == CameraProjection::kUnknown ||
        !FiniteTransform(secondary_from_primary)) {
        return false;
    }
    cv::Mat primary_gray;
    cv::Mat secondary_gray;
    if (!ToGray8(primary_image, encoding, &primary_gray) ||
        !ToGray8(secondary_image, encoding, &secondary_gray)) {
        return false;
    }

    const cv::Mat primary_matrix = CameraMatrix(primary_camera);
    const cv::Mat secondary_matrix = CameraMatrix(secondary_camera);
    const cv::Mat primary_distortion = Distortion(primary_camera);
    const cv::Mat secondary_distortion = Distortion(secondary_camera);
    const cv::Mat rotation =
        RotationMatrix(secondary_from_primary.linear());
    const cv::Mat translation =
        TranslationVector(secondary_from_primary.translation());
    cv::Mat primary_rectification;
    cv::Mat secondary_rectification;
    cv::Mat primary_projection;
    cv::Mat secondary_projection;
    cv::Mat disparity_to_depth;
    try {
        if (primary_camera.projection ==
            CameraProjection::kKannalaBrandt8) {
            cv::fisheye::stereoRectify(
                primary_matrix, primary_distortion, secondary_matrix,
                secondary_distortion, primary_gray.size(), rotation,
                translation, primary_rectification, secondary_rectification,
                primary_projection, secondary_projection,
                disparity_to_depth, cv::CALIB_ZERO_DISPARITY,
                primary_gray.size(), 0.0, 1.0);
        } else {
            cv::stereoRectify(
                primary_matrix, primary_distortion, secondary_matrix,
                secondary_distortion, primary_gray.size(), rotation,
                translation, primary_rectification, secondary_rectification,
                primary_projection, secondary_projection,
                disparity_to_depth, cv::CALIB_ZERO_DISPARITY, 0.0,
                primary_gray.size());
        }
    } catch (const cv::Exception&) {
        return false;
    }
    if (primary_projection.empty() || secondary_projection.empty() ||
        disparity_to_depth.empty() ||
        std::abs(secondary_projection.at<double>(0, 3)) < 1e-9 ||
        std::abs(secondary_projection.at<double>(0, 0)) < 1e-9) {
        // StereoSGBM is horizontal, so vertical-only rectification is rejected.
        return false;
    }

    cv::Mat primary_map_x;
    cv::Mat primary_map_y;
    cv::Mat secondary_map_x;
    cv::Mat secondary_map_y;
    try {
        if (primary_camera.projection ==
            CameraProjection::kKannalaBrandt8) {
            cv::fisheye::initUndistortRectifyMap(
                primary_matrix, primary_distortion, primary_rectification,
                primary_projection, primary_gray.size(), CV_32FC1,
                primary_map_x, primary_map_y);
            cv::fisheye::initUndistortRectifyMap(
                secondary_matrix, secondary_distortion,
                secondary_rectification, secondary_projection,
                secondary_gray.size(), CV_32FC1, secondary_map_x,
                secondary_map_y);
        } else {
            cv::initUndistortRectifyMap(
                primary_matrix, primary_distortion, primary_rectification,
                primary_projection, primary_gray.size(), CV_32FC1,
                primary_map_x, primary_map_y);
            cv::initUndistortRectifyMap(
                secondary_matrix, secondary_distortion,
                secondary_rectification, secondary_projection,
                secondary_gray.size(), CV_32FC1, secondary_map_x,
                secondary_map_y);
        }
        cv::remap(primary_gray, rectified->primary_gray, primary_map_x,
                  primary_map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        cv::remap(secondary_gray, rectified->secondary_gray, secondary_map_x,
                  secondary_map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        cv::remap(primary_image, rectified->primary_color, primary_map_x,
                  primary_map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    } catch (const cv::Exception&) {
        return false;
    }
    rectified->disparity_to_depth = disparity_to_depth;
    rectified->primary_to_rectified = primary_rectification;
    const double rectified_translation =
        secondary_projection.at<double>(0, 3) /
        secondary_projection.at<double>(0, 0);
    int disparities = std::max(16, options.stereo_number_of_disparities);
    disparities = (disparities + 15) / 16 * 16;
    rectified->minimum_disparity = rectified_translation > 0.0
                                       ? -disparities
                                       : 0;
    return true;
}

bool UndistortRgbd(const MappingFrame& frame, cv::Mat* color,
                   cv::Mat* depth) {
    if (color == nullptr || depth == nullptr || frame.color.empty() ||
        frame.depth.empty() ||
        frame.camera.projection == CameraProjection::kUnknown) {
        return false;
    }
    if (frame.camera.projection == CameraProjection::kRectified) {
        *color = frame.color;
        *depth = frame.depth;
        return true;
    }
    const cv::Mat camera = CameraMatrix(frame.camera);
    const cv::Mat distortion = Distortion(frame.camera);
    const cv::Mat identity = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat map_x;
    cv::Mat map_y;
    try {
        if (frame.camera.projection ==
            CameraProjection::kKannalaBrandt8) {
            cv::fisheye::initUndistortRectifyMap(
                camera, distortion, identity, camera, frame.depth.size(),
                CV_32FC1, map_x, map_y);
        } else {
            cv::initUndistortRectifyMap(
                camera, distortion, identity, camera, frame.depth.size(),
                CV_32FC1, map_x, map_y);
        }
        cv::remap(frame.color, *color, map_x, map_y, cv::INTER_LINEAR,
                  cv::BORDER_CONSTANT);
        cv::remap(frame.depth, *depth, map_x, map_y, cv::INTER_NEAREST,
                  cv::BORDER_CONSTANT);
    } catch (const cv::Exception&) {
        return false;
    }
    return !color->empty() && !depth->empty();
}

bool ReadDepth(const cv::Mat& depth, int row, int column, double scale,
               double* value) {
    if (value == nullptr || !std::isfinite(scale) || scale <= 0.0)
        return false;
    double raw = 0.0;
    switch (depth.type()) {
        case CV_16UC1:
            raw = depth.at<std::uint16_t>(row, column);
            break;
        case CV_32FC1:
            raw = depth.at<float>(row, column);
            break;
        case CV_64FC1:
            raw = depth.at<double>(row, column);
            break;
        default:
            return false;
    }
    *value = raw / scale;
    return std::isfinite(*value);
}

void ReadColor(const MappingFrame& frame, int source_row, int source_column,
               int source_rows, int source_columns, ColorPoint* point) {
    if (point == nullptr || frame.color.empty())
        return;
    const int row = std::clamp(
        source_row * frame.color.rows / std::max(source_rows, 1), 0,
        frame.color.rows - 1);
    const int column = std::clamp(
        source_column * frame.color.cols / std::max(source_columns, 1), 0,
        frame.color.cols - 1);
    if (frame.color.type() == CV_8UC1) {
        const std::uint8_t value = frame.color.at<std::uint8_t>(row, column);
        point->red = value;
        point->green = value;
        point->blue = value;
        return;
    }
    if (frame.color.type() != CV_8UC3 && frame.color.type() != CV_8UC4)
        return;
    const std::uint8_t* pixel = frame.color.ptr<std::uint8_t>(row) +
                                column * frame.color.channels();
    if (frame.color_encoding == ColorEncoding::kRgb) {
        point->red = pixel[0];
        point->green = pixel[1];
        point->blue = pixel[2];
    } else {
        point->blue = pixel[0];
        point->green = pixel[1];
        point->red = pixel[2];
    }
}

}  // namespace

namespace internal {

MappingWorkerWork SelectMappingWorkerWork(
    bool idle_snapshot, bool reset_requested, bool rebuild_requested,
    bool have_queued_frames, bool have_frame_removals,
    bool have_pose_updates, bool flush_pending) {
    if (reset_requested)
        return MappingWorkerWork::kReset;
    if (rebuild_requested)
        return MappingWorkerWork::kForcedRebuild;
    if (have_queued_frames)
        return MappingWorkerWork::kFrame;
    if (have_frame_removals)
        return MappingWorkerWork::kFrameRemoval;
    if (have_pose_updates)
        return MappingWorkerWork::kPoseUpdate;
    if (flush_pending)
        return MappingWorkerWork::kFlush;
    if (idle_snapshot)
        return MappingWorkerWork::kIdleSnapshot;
    return MappingWorkerWork::kNone;
}

}  // namespace internal

bool CameraModel::IsValid() const {
    const bool distortion_is_finite =
        std::all_of(distortion.begin(), distortion.end(),
                    [](double value) { return std::isfinite(value); });
    return ValidCameraProjection(projection) &&
           distortion_is_finite && std::isfinite(fx) && std::isfinite(fy) &&
           std::isfinite(cx) && std::isfinite(cy) &&
           std::isfinite(depth_scale) && fx > 0.0 && fy > 0.0 &&
           depth_scale > 0.0;
}

MappingPipeline::MappingPipeline(MappingPipelineOptions options)
    : options_(std::move(options)),
      dense_map_(options_.dense_map),
      navigation_map_(NavigationMapOptions(options_)),
      ground_classifier_(options_.occupancy_grid),
      occupancy_grid_(options_.occupancy_grid) {
    if (options_.queue_capacity == 0 || options_.maximum_cached_frames == 0 ||
        options_.maximum_navigation_voxels == 0 ||
        options_.projection.pixel_stride <= 0 ||
        options_.projection.stereo_number_of_disparities <= 0 ||
        options_.projection.stereo_block_size < 3 ||
        options_.projection.maximum_points_per_frame == 0 ||
        !std::isfinite(options_.projection.minimum_temporal_baseline) ||
        !std::isfinite(options_.projection.maximum_temporal_baseline) ||
        options_.projection.minimum_temporal_baseline <= 0.0 ||
        options_.projection.minimum_temporal_baseline >=
            options_.projection.maximum_temporal_baseline ||
        !std::isfinite(options_.projection.minimum_depth) ||
        !std::isfinite(options_.projection.maximum_depth) ||
        options_.projection.minimum_depth <= 0.0 ||
        options_.projection.minimum_depth >=
            options_.projection.maximum_depth ||
        !std::isfinite(options_.minimum_translation) ||
        !std::isfinite(options_.minimum_rotation_radians) ||
        !std::isfinite(options_.maximum_frame_interval) ||
        !std::isfinite(options_.snapshot_period) ||
        !std::isfinite(options_.navigation_radius) ||
        !std::isfinite(options_.navigation_recenter_distance) ||
        options_.navigation_radius < 0.0 ||
        options_.navigation_recenter_distance < 0.0 ||
        (options_.navigation_radius > 0.0 &&
         options_.navigation_recenter_distance > options_.navigation_radius) ||
        !std::isfinite(options_.rebuild_translation_threshold) ||
        !std::isfinite(options_.rebuild_rotation_threshold_radians) ||
        options_.minimum_translation < 0.0 ||
        options_.minimum_rotation_radians < 0.0 ||
        options_.rebuild_translation_threshold < 0.0 ||
        options_.rebuild_rotation_threshold_radians < 0.0 ||
        options_.maximum_frame_interval < 0.0 ||
        options_.snapshot_period < 0.0) {
        throw std::invalid_argument("invalid mapping-pipeline options");
    }
}

MappingPipeline::~MappingPipeline() {
    Stop();
    CleanupHistoryCache();
}

bool MappingPipeline::Start() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_)
        return true;
    stop_requested_ = false;
    reset_requested_ = false;
    rebuild_requested_ = false;
    reset_completed_id_ = reset_request_id_;
    flush_completed_id_ = flush_request_id_;
    running_ = true;
    try {
        worker_ = std::thread(&MappingPipeline::WorkerLoop, this);
    } catch (...) {
        running_ = false;
        throw;
    }
    return true;
}

void MappingPipeline::Stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_ && !worker_.joinable())
            return;
        stop_requested_ = true;
    }
    condition_.notify_all();
    if (worker_.joinable())
        worker_.join();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        stop_requested_ = false;
    }
    condition_.notify_all();
}

bool MappingPipeline::running() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return running_;
}

bool MappingPipeline::ValidateFrame(const MappingFrame& frame) const {
    if (!std::isfinite(frame.timestamp) ||
        !FiniteTransform(frame.world_from_camera) ||
        !frame.camera.IsValid() || frame.color.empty()) {
        return false;
    }
    if (frame.color.type() != CV_8UC1 && frame.color.type() != CV_8UC3 &&
        frame.color.type() != CV_8UC4) {
        return false;
    }
    if (frame.depth_source == DepthSource::kRgbd) {
        return !frame.depth.empty() && frame.depth.rows > 0 &&
               frame.depth.cols > 0 &&
               frame.depth.size() == frame.color.size() &&
               (frame.depth.type() == CV_16UC1 ||
                frame.depth.type() == CV_32FC1 ||
                frame.depth.type() == CV_64FC1);
    }
    if (frame.depth_source == DepthSource::kStereo) {
        if (frame.right.empty() ||
            frame.right.size() != frame.color.size()) {
            return false;
        }
        if (frame.camera.projection == CameraProjection::kRectified) {
            const double baseline =
                frame.camera.stereo_baseline > 0.0
                    ? frame.camera.stereo_baseline
                    : frame.secondary_from_camera.translation().norm();
            return std::isfinite(baseline) && baseline > 0.0;
        }
        const double baseline =
            frame.secondary_from_camera.translation().norm();
        return frame.secondary_camera.IsValid() &&
               frame.secondary_camera.projection == frame.camera.projection &&
               FiniteTransform(frame.secondary_from_camera) &&
               std::isfinite(baseline) &&
               baseline >= options_.projection.minimum_temporal_baseline &&
               baseline <= options_.projection.maximum_temporal_baseline;
    }
    const double baseline = frame.reference_from_camera.translation().norm();
    return !frame.reference.empty() &&
           frame.reference.size() == frame.color.size() &&
           FiniteTransform(frame.reference_from_camera) &&
           std::isfinite(baseline) &&
           baseline >= options_.projection.minimum_temporal_baseline &&
           baseline <= options_.projection.maximum_temporal_baseline;
}

bool MappingPipeline::AssignFrameId(MappingFrame* frame) {
    if (frame == nullptr)
        return false;
    constexpr std::uint64_t kMaximumFrameId =
        std::numeric_limits<std::uint64_t>::max();
    std::uint64_t next = next_frame_id_.load(std::memory_order_relaxed);
    if (frame->frame_id != 0) {
        if (frame->frame_id == kMaximumFrameId) {
            while (next != 0 &&
                   !next_frame_id_.compare_exchange_weak(
                       next, 0, std::memory_order_relaxed)) {
            }
            return true;
        }
        const std::uint64_t following = frame->frame_id + 1;
        while (next != 0 && next < following &&
               !next_frame_id_.compare_exchange_weak(
                   next, following, std::memory_order_relaxed)) {
        }
        return true;
    }
    while (next != 0) {
        const std::uint64_t assigned = next;
        const std::uint64_t following =
            assigned == kMaximumFrameId ? 0 : assigned + 1;
        if (next_frame_id_.compare_exchange_weak(
                next, following, std::memory_order_relaxed)) {
            frame->frame_id = assigned;
            return true;
        }
    }
    return false;
}

bool MappingPipeline::Submit(MappingFrame frame,
                             std::uint64_t* assigned_frame_id) {
    return TrySubmit(&frame, assigned_frame_id) == SubmitResult::kAccepted;
}

MappingPipeline::SubmitResult MappingPipeline::TrySubmit(
    MappingFrame* frame, std::uint64_t* assigned_frame_id) {
    if (frame == nullptr || !ValidateFrame(*frame)) {
        ++rejected_frames_;
        return SubmitResult::kInvalid;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_ || stop_requested_)
            return SubmitResult::kStopped;
        // Exhaustion is permanent for an automatic-id frame and must take
        // precedence over transient queue pressure. This check does not
        // consume an id, so kQueueFull still leaves the frame retryable.
        if (frame->frame_id == 0 &&
            next_frame_id_.load(std::memory_order_relaxed) == 0) {
            ++rejected_frames_;
            AWARN_EVERY(100)
                << "Atlas mapping automatic frame ids exhausted";
            return SubmitResult::kInvalid;
        }
        if (queue_.size() >= options_.queue_capacity) {
            ++backpressured_frames_;
            return SubmitResult::kQueueFull;
        }
        // Assign only after capacity is reserved. A backpressured automatic
        // frame therefore keeps frame_id == 0 and consumes no stable id.
        if (!AssignFrameId(frame)) {
            ++rejected_frames_;
            AWARN_EVERY(100)
                << "Atlas mapping automatic frame ids exhausted";
            return SubmitResult::kInvalid;
        }
        if (assigned_frame_id != nullptr)
            *assigned_frame_id = frame->frame_id;
        queue_.push_back(std::move(*frame));
        ++submitted_frames_;
    }
    condition_.notify_one();
    return SubmitResult::kAccepted;
}

bool MappingPipeline::UpdatePoses(std::vector<TimedPose> poses) {
    poses.erase(
        std::remove_if(poses.begin(), poses.end(), [](const TimedPose& pose) {
            return pose.frame_id == 0 || !std::isfinite(pose.timestamp) ||
                   !FiniteTransform(pose.world_from_camera);
        }),
        poses.end());
    if (poses.empty())
        return false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_ || stop_requested_)
            return false;
        pending_poses_.reserve(pending_poses_.size() + poses.size());
        std::unordered_map<std::uint64_t, std::size_t> pending_indices;
        pending_indices.reserve(pending_poses_.size() + poses.size());
        for (std::size_t index = 0; index < pending_poses_.size(); ++index)
            pending_indices.emplace(pending_poses_[index].frame_id, index);
        for (TimedPose& pose : poses) {
            const auto pending = pending_indices.find(pose.frame_id);
            if (pending == pending_indices.end()) {
                pending_indices.emplace(pose.frame_id,
                                        pending_poses_.size());
                pending_poses_.push_back(std::move(pose));
            } else {
                pending_poses_[pending->second] = std::move(pose);
            }
        }
    }
    condition_.notify_one();
    return true;
}

std::vector<std::uint64_t> MappingPipeline::RemoveFrames(
    std::vector<std::uint64_t> frame_ids) {
    frame_ids.erase(
        std::remove(frame_ids.begin(), frame_ids.end(), 0U),
        frame_ids.end());
    std::sort(frame_ids.begin(), frame_ids.end());
    frame_ids.erase(std::unique(frame_ids.begin(), frame_ids.end()),
                    frame_ids.end());
    std::vector<std::uint64_t> accepted;
    if (frame_ids.empty())
        return accepted;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_ || stop_requested_)
            return accepted;
        const std::size_t capacity = std::max(
            options_.queue_capacity, options_.maximum_cached_frames);
        const std::size_t outstanding = pending_frame_removals_.size() +
            processing_frame_removals_.size() +
            completed_frame_removals_.size() +
            retained_frame_removals_.size();
        std::size_t available =
            outstanding < capacity ? capacity - outstanding : 0U;
        accepted.reserve(std::min(available, frame_ids.size()));
        for (const std::uint64_t frame_id : frame_ids) {
            if (pending_frame_removals_.count(frame_id) != 0U ||
                processing_frame_removals_.count(frame_id) != 0U) {
                accepted.push_back(frame_id);
                continue;
            }
            if (completed_frame_removals_.count(frame_id) != 0U ||
                retained_frame_removals_.count(frame_id) != 0U ||
                available == 0U) {
                continue;
            }
            pending_frame_removals_.insert(frame_id);
            accepted.push_back(frame_id);
            --available;
        }
    }
    if (!accepted.empty())
        condition_.notify_one();
    return accepted;
}

MappingFrameRemovalResults MappingPipeline::TakeFrameRemovalResults() {
    std::lock_guard<std::mutex> lock(mutex_);
    MappingFrameRemovalResults results;
    results.removed_frame_ids.assign(completed_frame_removals_.begin(),
                                     completed_frame_removals_.end());
    results.retained_frame_ids.assign(retained_frame_removals_.begin(),
                                      retained_frame_removals_.end());
    std::sort(results.removed_frame_ids.begin(),
              results.removed_frame_ids.end());
    std::sort(results.retained_frame_ids.begin(),
              results.retained_frame_ids.end());
    completed_frame_removals_.clear();
    retained_frame_removals_.clear();
    return results;
}

bool MappingPipeline::RequestRebuild() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_ || stop_requested_)
            return false;
        rebuild_requested_ = true;
    }
    condition_.notify_one();
    return true;
}

bool MappingPipeline::Flush() {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!running_ || stop_requested_ || !worker_.joinable())
        return false;
    const std::uint64_t request_id = ++flush_request_id_;
    condition_.notify_all();
    condition_.wait(lock, [this, request_id] {
        return flush_completed_id_ >= request_id || stop_requested_ ||
               !running_;
    });
    return flush_completed_id_ >= request_id;
}

void MappingPipeline::Reset() {
    std::unique_lock<std::mutex> lock(mutex_);
    reset_discarded_frames_.fetch_add(queue_.size(),
                                       std::memory_order_relaxed);
    queue_.clear();
    pending_poses_.clear();
    pending_frame_removals_.clear();
    completed_frame_removals_.clear();
    retained_frame_removals_.clear();
    rebuild_requested_ = false;

    // A concurrent Stop may already be joining a worker that can no longer
    // accept a new reset request. Wait for it to publish running_=false, then
    // clear synchronously while still excluding Start().
    if (stop_requested_ && running_) {
        condition_.wait(lock, [this] { return !running_; });
    }

    if (!running_ || !worker_.joinable()) {
        reset_requested_ = false;
        ResetWorkerState();
        reset_completed_id_ = reset_request_id_;
        lock.unlock();
        condition_.notify_all();
        return;
    }

    const std::uint64_t request_id = ++reset_request_id_;
    reset_requested_ = true;
    condition_.notify_all();
    condition_.wait(lock, [this, request_id] {
        return reset_completed_id_ >= request_id || !running_;
    });
    if (reset_completed_id_ >= request_id)
        return;

    // Defensive fallback for a worker that stopped between the checks above.
    reset_requested_ = false;
    ResetWorkerState();
    reset_completed_id_ = request_id;
    lock.unlock();
    condition_.notify_all();
}

void MappingPipeline::ResetWorkerState() {
    dense_map_.Reset();
    navigation_map_.Reset();
    occupancy_grid_.Reset();
    cache_.clear();
    cached_frames_by_id_.clear();
    resident_frame_order_.clear();
    optimized_poses_.clear();
    optimized_pose_order_.clear();
    monocular_frames_by_reference_pose_id_.clear();
    CleanupHistoryCache();
    cache_contains_complete_history_ = true;
    have_last_pose_ = false;
    have_navigation_center_ = false;
    navigation_center_.setZero();
    last_frame_time_ = 0.0;
    last_snapshot_time_ = -1.0;
    ++generation_;
    PublishSnapshot(0.0, true);
}

std::shared_ptr<const MapSnapshot> MappingPipeline::LatestSnapshot() const {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    return snapshot_;
}

MappingPipelineStats MappingPipeline::stats() const {
    MappingPipelineStats result;
    result.submitted_frames = submitted_frames_.load();
    result.dropped_frames = dropped_frames_.load();
    result.backpressured_frames = backpressured_frames_.load();
    result.reset_discarded_frames = reset_discarded_frames_.load();
    result.gated_frames = gated_frames_.load();
    result.processed_frames = processed_frames_.load();
    result.rejected_frames = rejected_frames_.load();
    result.removed_frames = removed_frames_.load();
    result.removal_rebuild_failures = removal_rebuild_failures_.load();
    result.duplicate_frames = duplicate_frames_.load();
    result.retriangulated_frames = retriangulated_frames_.load();
    result.skipped_incomplete_rebuilds =
        skipped_incomplete_rebuilds_.load();
    result.history_archive_failures =
        history_archive_failures_.load();
    return result;
}

bool MappingPipeline::ProjectFrame(
    const MappingFrame& frame,
    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>>* points)
    const {
    if (points == nullptr || !ValidateFrame(frame))
        return false;
    points->clear();

    cv::Mat disparity;
    cv::Mat working_depth = frame.depth;
    StereoRectification rectification;
    MappingFrame color_frame = frame;
    bool disparity_uses_q = false;
    double rectified_stereo_baseline = frame.camera.stereo_baseline;
    int rows = 0;
    int columns = 0;
    if (frame.depth_source == DepthSource::kRgbd) {
        if (!UndistortRgbd(frame, &color_frame.color, &working_depth))
            return false;
        rows = working_depth.rows;
        columns = working_depth.cols;
    } else if (frame.depth_source == DepthSource::kStereo &&
               frame.camera.projection == CameraProjection::kRectified) {
        cv::Mat left_gray;
        cv::Mat right_gray;
        if (!ToGray8(frame.color, frame.color_encoding, &left_gray) ||
            !ToGray8(frame.right, frame.color_encoding, &right_gray)) {
            return false;
        }
        if (!ComputeDisparity(left_gray, right_gray, options_.projection,
                              options_.projection.stereo_minimum_disparity,
                              &disparity)) {
            return false;
        }
        rows = disparity.rows;
        columns = disparity.cols;
        if (!(rectified_stereo_baseline > 0.0)) {
            rectified_stereo_baseline =
                frame.secondary_from_camera.translation().norm();
        }
    } else {
        const bool physical_stereo =
            frame.depth_source == DepthSource::kStereo;
        const cv::Mat& secondary_image =
            physical_stereo ? frame.right : frame.reference;
        const CameraModel& secondary_camera =
            physical_stereo ? frame.secondary_camera : frame.camera;
        const Eigen::Isometry3d& secondary_from_primary =
            physical_stereo ? frame.secondary_from_camera
                            : frame.reference_from_camera;
        if (!RectifyStereoPair(
                frame.color, secondary_image, frame.color_encoding,
                frame.camera, secondary_camera, secondary_from_primary,
                options_.projection, &rectification) ||
            !ComputeDisparity(
                rectification.primary_gray, rectification.secondary_gray,
                options_.projection, rectification.minimum_disparity,
                &disparity)) {
            return false;
        }
        color_frame.color = rectification.primary_color;
        disparity_uses_q = true;
        rows = disparity.rows;
        columns = disparity.cols;
    }

    cv::Matx44d disparity_to_depth = cv::Matx44d::eye();
    cv::Matx33d primary_to_rectified = cv::Matx33d::eye();
    if (disparity_uses_q) {
        for (int row = 0; row < 4; ++row) {
            for (int column = 0; column < 4; ++column) {
                disparity_to_depth(row, column) =
                    rectification.disparity_to_depth.at<double>(row, column);
            }
        }
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                primary_to_rectified(row, column) =
                    rectification.primary_to_rectified.at<double>(row,
                                                                   column);
            }
        }
    }

    const int stride = options_.projection.pixel_stride;
    const std::size_t estimate =
        static_cast<std::size_t>((rows + stride - 1) / stride) *
        static_cast<std::size_t>((columns + stride - 1) / stride);
    points->reserve(
        std::min(estimate, options_.projection.maximum_points_per_frame));
    for (int row = 0; row < rows; row += stride) {
        for (int column = 0; column < columns; column += stride) {
            double depth = 0.0;
            ColorPoint point;
            if (frame.depth_source == DepthSource::kRgbd) {
                if (!ReadDepth(working_depth, row, column,
                               frame.camera.depth_scale, &depth)) {
                    continue;
                }
                point.position = Eigen::Vector3f(
                    static_cast<float>((column - frame.camera.cx) * depth /
                                       frame.camera.fx),
                    static_cast<float>((row - frame.camera.cy) * depth /
                                       frame.camera.fy),
                    static_cast<float>(depth));
            } else if (!disparity_uses_q) {
                const double disparity_pixels = static_cast<double>(
                    disparity.at<std::int16_t>(row, column)) / 16.0;
                if (!std::isfinite(disparity_pixels) ||
                    disparity_pixels <= 0.0) {
                    continue;
                }
                depth = frame.camera.fx * rectified_stereo_baseline /
                        disparity_pixels;
                point.position = Eigen::Vector3f(
                    static_cast<float>((column - frame.camera.cx) * depth /
                                       frame.camera.fx),
                    static_cast<float>((row - frame.camera.cy) * depth /
                                       frame.camera.fy),
                    static_cast<float>(depth));
            } else {
                const double disparity_pixels = static_cast<double>(
                    disparity.at<std::int16_t>(row, column)) / 16.0;
                const double invalid_disparity =
                    static_cast<double>(rectification.minimum_disparity - 1);
                if (!std::isfinite(disparity_pixels) ||
                    disparity_pixels <= invalid_disparity + 1e-6) {
                    continue;
                }
                const cv::Vec4d homogeneous = disparity_to_depth *
                    cv::Vec4d(column, row, disparity_pixels, 1.0);
                if (!std::isfinite(homogeneous[3]) ||
                    std::abs(homogeneous[3]) < 1e-12) {
                    continue;
                }
                const cv::Vec3d rectified_point(
                    homogeneous[0] / homogeneous[3],
                    homogeneous[1] / homogeneous[3],
                    homogeneous[2] / homogeneous[3]);
                const cv::Vec3d current_point =
                    primary_to_rectified.t() * rectified_point;
                depth = current_point[2];
                if (!std::isfinite(current_point[0]) ||
                    !std::isfinite(current_point[1]) ||
                    !std::isfinite(current_point[2])) {
                    continue;
                }
                point.position = Eigen::Vector3f(
                    static_cast<float>(current_point[0]),
                    static_cast<float>(current_point[1]),
                    static_cast<float>(current_point[2]));
            }
            if (!std::isfinite(depth) ||
                depth < options_.projection.minimum_depth ||
                depth > options_.projection.maximum_depth) {
                continue;
            }
            ReadColor(color_frame, row, column, rows, columns, &point);
            points->push_back(point);
            if (points->size() >=
                options_.projection.maximum_points_per_frame) {
                return true;
            }
        }
    }
    return !points->empty();
}

bool MappingPipeline::PassesMotionGate(const MappingFrame& frame) const {
    if (!have_last_pose_ || frame.timestamp < last_frame_time_)
        return true;
    const double translation =
        (frame.world_from_camera.translation() - last_pose_.translation())
            .norm();
    const double rotation = RotationAngle(last_pose_, frame.world_from_camera);
    const double elapsed = frame.timestamp - last_frame_time_;
    return translation >= options_.minimum_translation ||
           rotation >= options_.minimum_rotation_radians ||
           elapsed >= options_.maximum_frame_interval;
}

std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>>
MappingPipeline::TransformPoints(
    const std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>>&
        camera_points,
    const Eigen::Isometry3d& world_from_camera) const {
    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>> world_points;
    world_points.reserve(camera_points.size());
    const Eigen::Isometry3f transform = world_from_camera.cast<float>();
    for (const ColorPoint& camera_point : camera_points) {
        ColorPoint world_point = camera_point;
        world_point.position = transform * camera_point.position;
        if (world_point.position.allFinite())
            world_points.push_back(world_point);
    }
    return world_points;
}

bool MappingPipeline::ProjectAndFilter(
    const MappingFrame& frame,
    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>>* points)
    const {
    if (!ProjectFrame(frame, points))
        return false;
    // RTAB-Map applies a local voxel filter before retaining a node cloud.
    // Keeping the reduced camera-frame cloud bounds both cache memory and the
    // cost of loop-closure rebuilds, while the global map performs a second
    // merge after transforming it to world coordinates.
    DenseMapOptions local_options = options_.dense_map;
    local_options.maximum_voxels = std::min(
        local_options.maximum_voxels,
        options_.projection.maximum_points_per_frame);
    local_options.reserve_voxels =
        std::min(points->size(), local_options.maximum_voxels);
    DenseMap local_map(local_options);
    local_map.Integrate(*points);
    *points = std::move(local_map.Snapshot(frame.timestamp, 0).points);
    return !points->empty();
}

bool MappingPipeline::EnsureHistoryCacheDirectory() {
    if (history_cache_directory_created_)
        return true;
    if (options_.maximum_history_cache_bytes == 0)
        return false;

    std::error_code error;
    fs::path parent;
    if (options_.history_cache_directory.empty()) {
        parent = fs::temp_directory_path(error);
    } else {
        parent = fs::path(options_.history_cache_directory);
    }
    if (error || parent.empty() || !fs::is_directory(parent, error) || error)
        return false;

    const std::uint64_t seed =
        static_cast<std::uint64_t>(
            std::chrono::steady_clock::now().time_since_epoch().count()) ^
        g_history_directory_counter.fetch_add(1);
    for (std::uint64_t attempt = 0; attempt < 64; ++attempt) {
        const fs::path candidate =
            parent / (std::string(kHistoryDirectoryPrefix) +
                      std::to_string(seed) + "-" + std::to_string(attempt));
        error.clear();
        if (fs::create_directory(candidate, error)) {
            history_cache_directory_ = candidate.string();
            history_cache_directory_created_ = true;
            return true;
        }
    }
    return false;
}

bool MappingPipeline::WriteArchivedPayload(
    CachedFrame* cached, const PointVector& points,
    const MappingFrame* source_frame) {
    if (cached == nullptr || points.empty() ||
        !EnsureHistoryCacheDirectory()) {
        return false;
    }

    std::vector<std::uint8_t> color_png;
    std::vector<std::uint8_t> reference_png;
    if (source_frame != nullptr) {
        if (source_frame->depth_source != DepthSource::kMonocularTemporal ||
            !EncodePng(source_frame->color, &color_png) ||
            !EncodePng(source_frame->reference, &reference_png)) {
            return false;
        }
    }
    if (color_png.size() > kMaximumEncodedImageBytes ||
        reference_png.size() > kMaximumEncodedImageBytes) {
        return false;
    }
    if (points.size() >
        std::numeric_limits<std::uint64_t>::max() / sizeof(ArchivedPoint)) {
        return false;
    }
    const std::uint64_t point_bytes =
        static_cast<std::uint64_t>(points.size()) * sizeof(ArchivedPoint);
    if (color_png.size() > std::numeric_limits<std::uint64_t>::max() -
                               kHistoryArchiveHeaderBytes - point_bytes ||
        reference_png.size() >
            std::numeric_limits<std::uint64_t>::max() -
                kHistoryArchiveHeaderBytes - point_bytes - color_png.size()) {
        return false;
    }
    const std::uint64_t archive_bytes =
        kHistoryArchiveHeaderBytes + point_bytes + color_png.size() +
        reference_png.size();
    const std::uint64_t previous_bytes =
        cached->archived ? cached->archive_bytes : 0;
    if (previous_bytes > history_cache_bytes_)
        return false;
    const std::uint64_t base_bytes = history_cache_bytes_ - previous_bytes;
    if (archive_bytes > options_.maximum_history_cache_bytes -
                            std::min(base_bytes,
                                     options_.maximum_history_cache_bytes)) {
        return false;
    }

    const std::uint64_t sequence = next_archive_sequence_++;
    const fs::path final_path =
        fs::path(history_cache_directory_) /
        ("frame-" + std::to_string(cached->frame_id) + "-" +
         std::to_string(sequence) + ".bin");
    const fs::path partial_path = final_path.string() + ".partial";
    std::ofstream output(partial_path, std::ios::binary | std::ios::trunc);
    const std::uint64_t point_count = points.size();
    const std::uint64_t color_bytes = color_png.size();
    const std::uint64_t reference_bytes = reference_png.size();
    output.write(kHistoryArchiveMagic.data(), kHistoryArchiveMagic.size());
    bool success = output.good() &&
                   WriteValue(&output, kHistoryArchiveVersion) &&
                   WriteValue(&output, point_count) &&
                   WriteValue(&output, color_bytes) &&
                   WriteValue(&output, reference_bytes);
    constexpr std::size_t kChunkSize = 4096;
    std::array<ArchivedPoint, kChunkSize> block;
    for (std::size_t begin = 0; success && begin < points.size();
         begin += kChunkSize) {
        const std::size_t count =
            std::min(kChunkSize, points.size() - begin);
        for (std::size_t index = 0; index < count; ++index) {
            const ColorPoint& point = points[begin + index];
            block[index] = {point.position.x(), point.position.y(),
                            point.position.z(), point.red, point.green,
                            point.blue, 0};
        }
        output.write(reinterpret_cast<const char*>(block.data()),
                     static_cast<std::streamsize>(count * sizeof(ArchivedPoint)));
        success = output.good();
    }
    if (success && !color_png.empty()) {
        output.write(reinterpret_cast<const char*>(color_png.data()),
                     static_cast<std::streamsize>(color_png.size()));
        success = output.good();
    }
    if (success && !reference_png.empty()) {
        output.write(reinterpret_cast<const char*>(reference_png.data()),
                     static_cast<std::streamsize>(reference_png.size()));
        success = output.good();
    }
    output.close();
    success = success && !output.fail();

    std::error_code error;
    if (!success) {
        fs::remove(partial_path, error);
        return false;
    }
    fs::rename(partial_path, final_path, error);
    if (error) {
        error.clear();
        fs::remove(partial_path, error);
        return false;
    }

    const std::string old_path = cached->archive_path;
    if (!old_path.empty()) {
        error.clear();
        if (!fs::remove(fs::path(old_path), error) || error) {
            error.clear();
            fs::remove(final_path, error);
            return false;
        }
    }
    cached->archive_path = final_path.string();
    cached->archive_bytes = archive_bytes;
    cached->archived = true;
    history_cache_bytes_ = base_bytes + archive_bytes;
    if (source_frame != nullptr) {
        if (!cached->source_frame)
            cached->source_frame = std::make_shared<MappingFrame>();
        *cached->source_frame = *source_frame;
    }
    DropResidentPayload(cached);
    return true;
}

bool MappingPipeline::ReadArchivedPayload(const CachedFrame& cached,
                                          PointVector* points,
                                          MappingFrame* source_frame) const {
    if (!cached.archived || cached.archive_path.empty())
        return false;
    std::error_code file_error;
    const fs::path archive_path(cached.archive_path);
    if (!fs::is_regular_file(archive_path, file_error) || file_error)
        return false;
    file_error.clear();
    const std::uintmax_t file_bytes = fs::file_size(archive_path, file_error);
    if (file_error || file_bytes != cached.archive_bytes ||
        file_bytes > options_.maximum_history_cache_bytes) {
        return false;
    }
    std::ifstream input(cached.archive_path, std::ios::binary);
    std::array<char, kHistoryArchiveMagic.size()> magic{};
    input.read(magic.data(), magic.size());
    std::uint32_t version = 0;
    std::uint64_t point_count = 0;
    std::uint64_t color_bytes = 0;
    std::uint64_t reference_bytes = 0;
    if (!input.good() || magic != kHistoryArchiveMagic ||
        !ReadValue(&input, &version) || version != kHistoryArchiveVersion ||
        !ReadValue(&input, &point_count) ||
        !ReadValue(&input, &color_bytes) ||
        !ReadValue(&input, &reference_bytes) ||
        color_bytes > kMaximumEncodedImageBytes ||
        reference_bytes > kMaximumEncodedImageBytes ||
        point_count > options_.projection.maximum_points_per_frame ||
        point_count > std::numeric_limits<std::uint64_t>::max() /
                          sizeof(ArchivedPoint)) {
        return false;
    }
    const std::uint64_t point_bytes = point_count * sizeof(ArchivedPoint);
    if (point_bytes > std::numeric_limits<std::uint64_t>::max() -
                          kHistoryArchiveHeaderBytes) {
        return false;
    }
    std::uint64_t expected_bytes =
        kHistoryArchiveHeaderBytes + point_bytes;
    if (color_bytes >
        std::numeric_limits<std::uint64_t>::max() - expected_bytes) {
        return false;
    }
    expected_bytes += color_bytes;
    if (reference_bytes >
        std::numeric_limits<std::uint64_t>::max() - expected_bytes) {
        return false;
    }
    expected_bytes += reference_bytes;
    if (expected_bytes != cached.archive_bytes ||
        expected_bytes > options_.maximum_history_cache_bytes ||
        point_bytes >
        static_cast<std::uint64_t>(
            std::numeric_limits<std::streamoff>::max())) {
        return false;
    }
    if (file_bytes != expected_bytes)
        return false;

    constexpr std::size_t kChunkSize = 4096;
    std::array<ArchivedPoint, kChunkSize> block;
    if (points != nullptr) {
        points->clear();
        points->reserve(static_cast<std::size_t>(point_count));
        std::uint64_t remaining = point_count;
        while (remaining > 0) {
            const std::size_t count = static_cast<std::size_t>(
                std::min<std::uint64_t>(remaining, kChunkSize));
            input.read(reinterpret_cast<char*>(block.data()),
                       static_cast<std::streamsize>(count *
                                                    sizeof(ArchivedPoint)));
            if (!input.good())
                return false;
            for (std::size_t index = 0; index < count; ++index) {
                ColorPoint point;
                point.position = Eigen::Vector3f(
                    block[index].x, block[index].y, block[index].z);
                point.red = block[index].red;
                point.green = block[index].green;
                point.blue = block[index].blue;
                points->push_back(point);
            }
            remaining -= count;
        }
    } else {
        input.seekg(static_cast<std::streamoff>(point_bytes), std::ios::cur);
        if (!input.good())
            return false;
    }

    if (source_frame == nullptr)
        return true;
    if (!cached.source_frame || color_bytes == 0 || reference_bytes == 0 ||
        color_bytes > static_cast<std::uint64_t>(
                          std::numeric_limits<std::size_t>::max()) ||
        reference_bytes > static_cast<std::uint64_t>(
                              std::numeric_limits<std::size_t>::max())) {
        return false;
    }
    std::vector<std::uint8_t> color_png(static_cast<std::size_t>(color_bytes));
    std::vector<std::uint8_t> reference_png(
        static_cast<std::size_t>(reference_bytes));
    input.read(reinterpret_cast<char*>(color_png.data()),
               static_cast<std::streamsize>(color_png.size()));
    if (!input.good())
        return false;
    input.read(reinterpret_cast<char*>(reference_png.data()),
               static_cast<std::streamsize>(reference_png.size()));
    if (!input.good())
        return false;
    *source_frame = *cached.source_frame;
    return DecodePng(color_png, &source_frame->color) &&
           DecodePng(reference_png, &source_frame->reference);
}

void MappingPipeline::DropResidentPayload(CachedFrame* cached) {
    if (cached == nullptr)
        return;
    cached->camera_points.clear();
    cached->camera_points.shrink_to_fit();
    if (cached->source_frame) {
        cached->source_frame->color.release();
        cached->source_frame->depth.release();
        cached->source_frame->right.release();
        cached->source_frame->reference.release();
    }
    cached->payload_resident = false;
}

void MappingPipeline::RemoveArchivedPayload(CachedFrame* cached) {
    if (cached == nullptr || !cached->archived)
        return;
    std::error_code error;
    const bool removed = fs::remove(fs::path(cached->archive_path), error);
    if (removed && !error && cached->archive_bytes <= history_cache_bytes_)
        history_cache_bytes_ -= cached->archive_bytes;
    cached->archived = false;
    cached->archive_path.clear();
    cached->archive_bytes = 0;
}

void MappingPipeline::MarkHistoryIncomplete() {
    cache_contains_complete_history_ = false;
    ++history_archive_failures_;
}

void MappingPipeline::EnforceResidentCacheLimit() {
    while (resident_frame_order_.size() > options_.maximum_cached_frames) {
        const std::uint64_t frame_id = resident_frame_order_.front();
        resident_frame_order_.pop_front();
        const auto cached = cached_frames_by_id_.find(frame_id);
        if (cached == cached_frames_by_id_.end() ||
            !cached->second->payload_resident) {
            continue;
        }
        CachedFrame* frame = &*cached->second;
        if (cache_contains_complete_history_ &&
            WriteArchivedPayload(frame, frame->camera_points,
                                 frame->source_frame.get())) {
            continue;
        }
        if (cache_contains_complete_history_)
            MarkHistoryIncomplete();
        DropResidentPayload(frame);
    }
}

void MappingPipeline::CleanupHistoryCache() {
    if (history_cache_directory_created_ &&
        !history_cache_directory_.empty()) {
        const fs::path directory(history_cache_directory_);
        const std::string name = directory.filename().string();
        if (name.rfind(kHistoryDirectoryPrefix, 0) != 0) {
            ++history_archive_failures_;
            return;
        }
        std::error_code error;
        fs::remove_all(directory, error);
        if (error) {
            ++history_archive_failures_;
            return;
        }
    }
    history_cache_directory_.clear();
    history_cache_bytes_ = 0;
    next_archive_sequence_ = 0;
    history_cache_directory_created_ = false;
}

void MappingPipeline::ProcessFrame(MappingFrame frame) {
    if (cached_frames_by_id_.find(frame.frame_id) !=
        cached_frames_by_id_.end()) {
        ++duplicate_frames_;
        ++rejected_frames_;
        AWARN_EVERY(100) << "Atlas mapping rejected duplicate frame id "
                         << frame.frame_id;
        return;
    }
    const bool has_monocular_reference =
        frame.depth_source == DepthSource::kMonocularTemporal &&
        frame.reference_frame_id != 0;
    Eigen::Isometry3d world_from_reference = Eigen::Isometry3d::Identity();
    if (has_monocular_reference) {
        world_from_reference =
            frame.world_from_camera * frame.reference_from_camera.inverse();
    }
    const auto optimized_current = optimized_poses_.find(frame.frame_id);
    if (optimized_current != optimized_poses_.end())
        frame.world_from_camera = optimized_current->second;
    if (has_monocular_reference) {
        const auto optimized_reference =
            optimized_poses_.find(frame.reference_frame_id);
        if (optimized_reference != optimized_poses_.end())
            world_from_reference = optimized_reference->second;
        frame.reference_from_camera =
            world_from_reference.inverse() * frame.world_from_camera;
    }
    if (!PassesMotionGate(frame)) {
        ++gated_frames_;
        return;
    }
    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>> camera_points;
    if (!ProjectAndFilter(frame, &camera_points)) {
        ++rejected_frames_;
        return;
    }
    auto world_points = TransformPoints(camera_points, frame.world_from_camera);
    if (world_points.empty()) {
        ++rejected_frames_;
        return;
    }

    CachedFrame cached;
    cached.frame_id = frame.frame_id;
    cached.timestamp = frame.timestamp;
    cached.world_from_camera = frame.world_from_camera;
    cached.camera_points = std::move(camera_points);
    if (frame.depth_source == DepthSource::kMonocularTemporal)
        cached.source_frame = std::make_shared<MappingFrame>(frame);
    cache_.push_back(std::move(cached));
    const auto cached_iterator = std::prev(cache_.end());
    cached_frames_by_id_.insert_or_assign(frame.frame_id, cached_iterator);
    resident_frame_order_.push_back(frame.frame_id);
    if (has_monocular_reference) {
        monocular_frames_by_reference_pose_id_.emplace(
            frame.reference_frame_id, frame.frame_id);
    }
    EnforceResidentCacheLimit();

    StoreOptimizedPose(frame.frame_id, frame.world_from_camera);
    if (frame.depth_source == DepthSource::kMonocularTemporal &&
        frame.reference_frame_id != 0) {
        StoreOptimizedPose(
            frame.reference_frame_id,
            frame.world_from_camera * frame.reference_from_camera.inverse());
    }
    const Eigen::Vector3f sensor_origin =
        frame.world_from_camera.translation().cast<float>();
    SemanticPointCloud semantic_points;
    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>>
        navigation_points;
    if (!ground_classifier_.Classify(sensor_origin, world_points,
                                     &semantic_points, nullptr,
                                     &navigation_points)) {
        ++rejected_frames_;
        return;
    }
    if (options_.navigation_radius > 0.0) {
        if (!have_navigation_center_) {
            navigation_center_ = sensor_origin;
            have_navigation_center_ = true;
        } else {
            const Eigen::Vector3d displacement =
                sensor_origin.cast<double>() -
                navigation_center_.cast<double>();
            const double threshold = options_.navigation_recenter_distance;
            if (displacement.squaredNorm() > threshold * threshold) {
                navigation_center_ = sensor_origin;
                navigation_map_.RemoveOutsideRadius(
                    navigation_center_, options_.navigation_radius);
            }
        }
        RetainPointsWithinRadius(navigation_center_,
                                 options_.navigation_radius,
                                 &navigation_points);
    }
    dense_map_.Integrate(world_points);
    navigation_map_.Integrate(navigation_points);
    occupancy_grid_.Integrate(sensor_origin, semantic_points);
    have_last_pose_ = true;
    last_pose_ = frame.world_from_camera;
    last_frame_time_ = frame.timestamp;
    ++generation_;
    ++processed_frames_;
    PublishSnapshot(frame.timestamp, false);
}

void MappingPipeline::StoreOptimizedPose(
    std::uint64_t frame_id,
    const Eigen::Isometry3d& world_from_camera) {
    const auto existing = optimized_poses_.find(frame_id);
    if (existing == optimized_poses_.end()) {
        optimized_poses_.emplace(frame_id, world_from_camera);
        optimized_pose_order_.push_back(frame_id);
    } else {
        existing->second = world_from_camera;
    }
}

void MappingPipeline::PruneOptimizedPoses() {
    std::unordered_set<std::uint64_t> queued_pose_ids;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queued_pose_ids.reserve(queue_.size() * 2U);
        for (const MappingFrame& queued : queue_) {
            if (queued.frame_id != 0)
                queued_pose_ids.insert(queued.frame_id);
            if (queued.reference_frame_id != 0)
                queued_pose_ids.insert(queued.reference_frame_id);
        }
    }
    const std::size_t retained_pose_limit =
        cache_.size() * 2U + queued_pose_ids.size();
    const std::size_t soft_limit =
        options_.maximum_cached_frames * 3U + options_.queue_capacity;
    const std::size_t limit = std::max(retained_pose_limit, soft_limit);
    std::size_t attempts = optimized_pose_order_.size();
    while (optimized_poses_.size() > limit && attempts-- > 0U &&
           !optimized_pose_order_.empty()) {
        const std::uint64_t frame_id = optimized_pose_order_.front();
        optimized_pose_order_.pop_front();
        if (cached_frames_by_id_.find(frame_id) !=
                cached_frames_by_id_.end() ||
            monocular_frames_by_reference_pose_id_.find(frame_id) !=
                monocular_frames_by_reference_pose_id_.end() ||
            queued_pose_ids.count(frame_id) != 0U) {
            optimized_pose_order_.push_back(frame_id);
        } else {
            optimized_poses_.erase(frame_id);
        }
    }
}

void MappingPipeline::ApplyPoseUpdates(std::vector<TimedPose> poses,
                                       bool force_rebuild) {
    bool changed = force_rebuild;
    std::unordered_set<std::uint64_t> changed_pose_ids;
    std::vector<std::pair<CachedFrameIterator, TimedPose>>
        cached_pose_updates;
    cached_pose_updates.reserve(poses.size());
    const bool can_rebuild_complete_map = cache_contains_complete_history_;
    double timestamp = last_frame_time_;
    for (const TimedPose& pose : poses) {
        const auto previous_pose = optimized_poses_.find(pose.frame_id);
        if (previous_pose != optimized_poses_.end()) {
            const double translation =
                (previous_pose->second.translation() -
                 pose.world_from_camera.translation())
                    .norm();
            const double rotation =
                RotationAngle(previous_pose->second,
                              pose.world_from_camera);
            if (translation > std::numeric_limits<double>::epsilon() ||
                rotation > std::numeric_limits<double>::epsilon()) {
                changed_pose_ids.insert(pose.frame_id);
            }
            changed = changed ||
                      translation >=
                          options_.rebuild_translation_threshold ||
                      rotation >=
                          options_.rebuild_rotation_threshold_radians;
        } else {
            changed_pose_ids.insert(pose.frame_id);
        }
        StoreOptimizedPose(pose.frame_id, pose.world_from_camera);
        const auto cached = cached_frames_by_id_.find(pose.frame_id);
        if (cached == cached_frames_by_id_.end())
            continue;
        CachedFrameIterator iterator = cached->second;
        const double translation =
            (iterator->world_from_camera.translation() -
             pose.world_from_camera.translation())
                .norm();
        const double rotation =
            RotationAngle(iterator->world_from_camera, pose.world_from_camera);
        changed = changed ||
                  translation >= options_.rebuild_translation_threshold ||
                  rotation >= options_.rebuild_rotation_threshold_radians;
        if (can_rebuild_complete_map)
            cached_pose_updates.emplace_back(iterator, pose);
    }
    PruneOptimizedPoses();
    if (!changed)
        return;

    if (!can_rebuild_complete_map) {
        ++skipped_incomplete_rebuilds_;
        return;
    }

    for (const auto& [cached, pose] : cached_pose_updates) {
        cached->world_from_camera = pose.world_from_camera;
        cached->timestamp = pose.timestamp;
        timestamp = std::max(timestamp, pose.timestamp);
    }

    std::unordered_set<std::uint64_t> monocular_frames_to_update;
    for (const std::uint64_t pose_id : changed_pose_ids) {
        const auto current_frame = cached_frames_by_id_.find(pose_id);
        if (current_frame != cached_frames_by_id_.end() &&
            current_frame->second->source_frame &&
            current_frame->second->source_frame->depth_source ==
                DepthSource::kMonocularTemporal) {
            monocular_frames_to_update.insert(pose_id);
        }
        const auto reference_range =
            monocular_frames_by_reference_pose_id_.equal_range(pose_id);
        for (auto dependent = reference_range.first;
             dependent != reference_range.second; ++dependent) {
            monocular_frames_to_update.insert(dependent->second);
        }
    }
    for (const std::uint64_t frame_id : monocular_frames_to_update) {
        const auto cached_frame = cached_frames_by_id_.find(frame_id);
        if (cached_frame == cached_frames_by_id_.end()) {
            MarkHistoryIncomplete();
            ++skipped_incomplete_rebuilds_;
            return;
        }
        CachedFrame& cached = *cached_frame->second;
        const auto current_pose = optimized_poses_.find(cached.frame_id);
        const auto reference_pose = optimized_poses_.find(
            cached.source_frame->reference_frame_id);
        if (cached.source_frame->reference_frame_id == 0 ||
            current_pose == optimized_poses_.end() ||
            reference_pose == optimized_poses_.end()) {
            MarkHistoryIncomplete();
            ++skipped_incomplete_rebuilds_;
            return;
        }
        MappingFrame updated = *cached.source_frame;
        if (!cached.payload_resident &&
            (!cached.archived ||
             !ReadArchivedPayload(cached, nullptr, &updated))) {
            MarkHistoryIncomplete();
            ++skipped_incomplete_rebuilds_;
            return;
        }
        updated.world_from_camera = current_pose->second;
        const Eigen::Isometry3d reference_from_camera =
            reference_pose->second.inverse() * current_pose->second;
        const double relative_translation =
            (updated.reference_from_camera.translation() -
             reference_from_camera.translation())
                .norm();
        const double relative_rotation = RotationAngle(
            updated.reference_from_camera, reference_from_camera);
        if (relative_translation <= std::numeric_limits<double>::epsilon() &&
            relative_rotation <= std::numeric_limits<double>::epsilon()) {
            continue;
        }
        updated.reference_from_camera = reference_from_camera;
        PointVector points;
        if (!ProjectAndFilter(updated, &points)) {
            MarkHistoryIncomplete();
            ++skipped_incomplete_rebuilds_;
            return;
        }
        if (cached.payload_resident) {
            cached.camera_points = std::move(points);
            cached.source_frame =
                std::make_shared<MappingFrame>(std::move(updated));
        } else if (!WriteArchivedPayload(&cached, points, &updated)) {
            MarkHistoryIncomplete();
            ++skipped_incomplete_rebuilds_;
            return;
        }
        cached.world_from_camera = current_pose->second;
        ++retriangulated_frames_;
    }
    Rebuild(timestamp);
}

MappingFrameRemovalResults MappingPipeline::ApplyFrameRemovals(
    std::vector<std::uint64_t> frame_ids,
    std::vector<std::uint64_t> removed_queued_frame_ids) {
    std::sort(frame_ids.begin(), frame_ids.end());
    frame_ids.erase(std::unique(frame_ids.begin(), frame_ids.end()),
                    frame_ids.end());

    std::unordered_set<std::uint64_t> removable(frame_ids.begin(),
                                                 frame_ids.end());
    if (!cache_contains_complete_history_) {
        for (const std::uint64_t frame_id : frame_ids) {
            if (cached_frames_by_id_.count(frame_id) != 0U)
                removable.erase(frame_id);
        }
    }

    // A monocular reference pose can disappear only with every cached frame
    // that still depends on it. Iterate because a dependent refused above can
    // in turn make its own reference non-removable.
    const auto retain_required_references = [this, &removable]() {
        bool changed = false;
        do {
            changed = false;
            for (auto candidate = removable.begin();
                 candidate != removable.end();) {
                bool has_retained_dependent = false;
                const auto dependents =
                    monocular_frames_by_reference_pose_id_.equal_range(
                        *candidate);
                for (auto dependent = dependents.first;
                     dependent != dependents.second; ++dependent) {
                    if (removable.count(dependent->second) == 0U) {
                        has_retained_dependent = true;
                        break;
                    }
                }
                if (has_retained_dependent) {
                    candidate = removable.erase(candidate);
                    changed = true;
                } else {
                    ++candidate;
                }
            }
        } while (changed);
    };
    retain_required_references();
    // Queue records have not contributed to the map. A duplicate id can also
    // name an older cached frame, however, so only cache-free ids are complete
    // without a successful rebuild.
    for (const std::uint64_t frame_id : removed_queued_frame_ids) {
        if (cached_frames_by_id_.count(frame_id) == 0U)
            removable.insert(frame_id);
    }

    std::unordered_set<std::uint64_t> cached_exclusions;
    for (const std::uint64_t frame_id : removable) {
        if (cached_frames_by_id_.count(frame_id) != 0U)
            cached_exclusions.insert(frame_id);
    }

    const CachedFrame* latest_survivor = nullptr;
    for (const CachedFrame& cached : cache_) {
        if (cached_exclusions.count(cached.frame_id) != 0U)
            continue;
        if (latest_survivor == nullptr ||
            cached.timestamp > latest_survivor->timestamp ||
            (cached.timestamp == latest_survivor->timestamp &&
             cached.frame_id > latest_survivor->frame_id)) {
            latest_survivor = &cached;
        }
    }
    const double rebuilt_timestamp =
        latest_survivor == nullptr ? 0.0 : latest_survivor->timestamp;

    if (!cached_exclusions.empty() &&
        !Rebuild(rebuilt_timestamp, cached_exclusions)) {
        for (const std::uint64_t frame_id : cached_exclusions)
            removable.erase(frame_id);
        cached_exclusions.clear();
        retain_required_references();
        for (const std::uint64_t frame_id : removed_queued_frame_ids) {
            if (cached_frames_by_id_.count(frame_id) == 0U)
                removable.insert(frame_id);
        }
    }

    std::size_t removed_cached_frames = 0;
    for (const std::uint64_t frame_id : removable) {
        const auto cached = cached_frames_by_id_.find(frame_id);
        if (cached == cached_frames_by_id_.end())
            continue;
        CachedFrameIterator iterator = cached->second;
        RemoveArchivedPayload(&*iterator);
        resident_frame_order_.erase(
            std::remove(resident_frame_order_.begin(),
                        resident_frame_order_.end(), frame_id),
            resident_frame_order_.end());
        cache_.erase(iterator);
        cached_frames_by_id_.erase(cached);
        ++removed_cached_frames;
    }

    for (auto dependent = monocular_frames_by_reference_pose_id_.begin();
         dependent != monocular_frames_by_reference_pose_id_.end();) {
        if (removable.count(dependent->second) != 0U) {
            dependent =
                monocular_frames_by_reference_pose_id_.erase(dependent);
        } else {
            ++dependent;
        }
    }
    for (const std::uint64_t frame_id : removable) {
        if (monocular_frames_by_reference_pose_id_.count(frame_id) == 0U) {
            optimized_poses_.erase(frame_id);
            optimized_pose_order_.erase(
                std::remove(optimized_pose_order_.begin(),
                            optimized_pose_order_.end(), frame_id),
                optimized_pose_order_.end());
        }
    }

    removed_frames_.fetch_add(
        removed_queued_frame_ids.size() + removed_cached_frames,
        std::memory_order_relaxed);

    if (removed_cached_frames > 0U) {
        have_last_pose_ = latest_survivor != nullptr;
        last_pose_ = latest_survivor == nullptr
                         ? Eigen::Isometry3d::Identity()
                         : latest_survivor->world_from_camera;
        last_frame_time_ = rebuilt_timestamp;
    }
    PruneOptimizedPoses();

    MappingFrameRemovalResults results;
    results.removed_frame_ids.assign(removable.begin(), removable.end());
    for (const std::uint64_t frame_id : frame_ids) {
        if (removable.count(frame_id) == 0U)
            results.retained_frame_ids.push_back(frame_id);
    }
    std::sort(results.removed_frame_ids.begin(),
              results.removed_frame_ids.end());
    if (!results.retained_frame_ids.empty()) {
        removal_rebuild_failures_.fetch_add(
            results.retained_frame_ids.size(), std::memory_order_relaxed);
        AWARN << "Atlas mapping retained "
              << results.retained_frame_ids.size()
              << " culled frames because a safe rebuild was unavailable";
    }
    return results;
}

bool MappingPipeline::Rebuild(
    double timestamp,
    const std::unordered_set<std::uint64_t>& excluded_frame_ids) {
    if (!cache_contains_complete_history_) {
        ++skipped_incomplete_rebuilds_;
        return false;
    }

    DenseMap rebuilt_dense_map(options_.dense_map);
    DenseMap rebuilt_navigation_map(NavigationMapOptions(options_));
    bool have_rebuilt_navigation_center = false;
    Eigen::Vector3f rebuilt_navigation_center = Eigen::Vector3f::Zero();
    double latest_navigation_timestamp =
        -std::numeric_limits<double>::infinity();
    std::uint64_t latest_navigation_frame_id = 0;
    if (options_.navigation_radius > 0.0) {
        for (const CachedFrame& cached : cache_) {
            if (excluded_frame_ids.count(cached.frame_id) != 0U)
                continue;
            if (!have_rebuilt_navigation_center ||
                cached.timestamp > latest_navigation_timestamp ||
                (cached.timestamp == latest_navigation_timestamp &&
                 cached.frame_id > latest_navigation_frame_id)) {
                rebuilt_navigation_center =
                    cached.world_from_camera.translation().cast<float>();
                latest_navigation_timestamp = cached.timestamp;
                latest_navigation_frame_id = cached.frame_id;
                have_rebuilt_navigation_center = true;
            }
        }
    }
    OccupancyGridMap rebuilt_occupancy_grid(options_.occupancy_grid);
    for (const CachedFrame& cached : cache_) {
        if (excluded_frame_ids.count(cached.frame_id) != 0U)
            continue;
        PointVector archived_points;
        const PointVector* camera_points = &cached.camera_points;
        if (!cached.payload_resident) {
            if (!cached.archived ||
                !ReadArchivedPayload(cached, &archived_points, nullptr)) {
                MarkHistoryIncomplete();
                ++skipped_incomplete_rebuilds_;
                return false;
            }
            camera_points = &archived_points;
        }
        auto world_points =
            TransformPoints(*camera_points, cached.world_from_camera);
        const Eigen::Vector3f sensor_origin =
            cached.world_from_camera.translation().cast<float>();
        SemanticPointCloud semantic_points;
        PointVector navigation_points;
        if (!ground_classifier_.Classify(sensor_origin, world_points,
                                         &semantic_points, nullptr,
                                         &navigation_points)) {
            continue;
        }
        if (have_rebuilt_navigation_center) {
            RetainPointsWithinRadius(rebuilt_navigation_center,
                                     options_.navigation_radius,
                                     &navigation_points);
        }
        rebuilt_dense_map.Integrate(world_points);
        rebuilt_navigation_map.Integrate(navigation_points);
        rebuilt_occupancy_grid.Integrate(sensor_origin, semantic_points);
    }
    dense_map_ = std::move(rebuilt_dense_map);
    navigation_map_ = std::move(rebuilt_navigation_map);
    have_navigation_center_ = have_rebuilt_navigation_center;
    navigation_center_ = have_rebuilt_navigation_center
                             ? rebuilt_navigation_center
                             : Eigen::Vector3f::Zero();
    occupancy_grid_ = std::move(rebuilt_occupancy_grid);
    ++generation_;
    PublishSnapshot(timestamp, true);
    return true;
}

void MappingPipeline::PublishSnapshot(double timestamp, bool force) {
    if (!force && last_snapshot_time_ >= 0.0 &&
        timestamp >= last_snapshot_time_ &&
        timestamp - last_snapshot_time_ < options_.snapshot_period) {
        return;
    }
    auto snapshot = std::make_shared<MapSnapshot>();
    snapshot->dense_map = dense_map_.Snapshot(timestamp, generation_);
    snapshot->navigation_map =
        navigation_map_.Snapshot(timestamp, generation_);
    snapshot->occupancy_grid =
        occupancy_grid_.Snapshot(timestamp, generation_);
    {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        snapshot_ = std::move(snapshot);
    }
    last_snapshot_time_ = timestamp;
    last_published_generation_ = generation_;
}

void MappingPipeline::WorkerLoop() {
    for (;;) {
        MappingFrame frame;
        std::vector<TimedPose> poses;
        std::vector<std::uint64_t> frame_removals;
        std::vector<std::uint64_t> removed_queued_frame_ids;
        bool have_frame = false;
        bool reset = false;
        bool rebuild = false;
        bool idle_snapshot = false;
        std::uint64_t reset_request_id = 0;
        std::uint64_t flush_request_id = 0;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            const auto work_ready = [this] {
                return stop_requested_ || reset_requested_ ||
                       rebuild_requested_ || !pending_poses_.empty() ||
                       !pending_frame_removals_.empty() ||
                       !queue_.empty() ||
                       flush_completed_id_ < flush_request_id_;
            };
            if (options_.snapshot_period > 0.0 &&
                generation_ != last_published_generation_) {
                idle_snapshot = !condition_.wait_for(
                    lock,
                    std::chrono::duration<double>(options_.snapshot_period),
                    work_ready);
            } else {
                condition_.wait(lock, work_ready);
            }
            if (stop_requested_ && !reset_requested_ &&
                !rebuild_requested_ && pending_poses_.empty() &&
                pending_frame_removals_.empty() && queue_.empty()) {
                const bool needs_final_snapshot =
                    generation_ != last_published_generation_;
                lock.unlock();
                if (needs_final_snapshot)
                    PublishSnapshot(last_frame_time_, true);
                return;
            }
            const internal::MappingWorkerWork work =
                internal::SelectMappingWorkerWork(
                    idle_snapshot, reset_requested_, rebuild_requested_,
                    !queue_.empty(), !pending_frame_removals_.empty(),
                    !pending_poses_.empty(),
                    flush_completed_id_ < flush_request_id_);
            if (work == internal::MappingWorkerWork::kIdleSnapshot) {
                // Map work is worker-owned, so snapshotting after releasing
                // the queue mutex cannot race fusion or a rebuild.
            } else if (work == internal::MappingWorkerWork::kReset) {
                reset_requested_ = false;
                reset = true;
                reset_request_id = reset_request_id_;
            } else if (work ==
                       internal::MappingWorkerWork::kForcedRebuild) {
                rebuild = true;
                rebuild_requested_ = false;
                poses = std::move(pending_poses_);
                pending_poses_.clear();
            } else if (work == internal::MappingWorkerWork::kFrame) {
                frame = std::move(queue_.front());
                queue_.pop_front();
                have_frame = true;
            } else if (work ==
                       internal::MappingWorkerWork::kFrameRemoval) {
                frame_removals.assign(pending_frame_removals_.begin(),
                                      pending_frame_removals_.end());
                pending_frame_removals_.clear();
                processing_frame_removals_.insert(frame_removals.begin(),
                                                   frame_removals.end());
                const std::unordered_set<std::uint64_t> removal_set(
                    frame_removals.begin(), frame_removals.end());
                for (auto queued = queue_.begin(); queued != queue_.end();) {
                    if (removal_set.count(queued->frame_id) == 0U) {
                        ++queued;
                        continue;
                    }
                    removed_queued_frame_ids.push_back(queued->frame_id);
                    queued = queue_.erase(queued);
                }
            } else if (work == internal::MappingWorkerWork::kPoseUpdate) {
                poses = std::move(pending_poses_);
                pending_poses_.clear();
            } else if (work == internal::MappingWorkerWork::kFlush) {
                flush_request_id = flush_request_id_;
            }
        }
        if (idle_snapshot) {
            PublishSnapshot(last_frame_time_, true);
        } else if (reset) {
            ResetWorkerState();
            {
                std::lock_guard<std::mutex> lock(mutex_);
                processing_frame_removals_.clear();
                completed_frame_removals_.clear();
                retained_frame_removals_.clear();
                reset_completed_id_ =
                    std::max(reset_completed_id_, reset_request_id);
            }
            condition_.notify_all();
        } else if (!frame_removals.empty()) {
            MappingFrameRemovalResults results = ApplyFrameRemovals(
                frame_removals,
                std::move(removed_queued_frame_ids));
            const std::unordered_set<std::uint64_t> removed_ids(
                results.removed_frame_ids.begin(),
                results.removed_frame_ids.end());
            {
                std::lock_guard<std::mutex> lock(mutex_);
                pending_poses_.erase(
                    std::remove_if(
                        pending_poses_.begin(), pending_poses_.end(),
                        [&removed_ids](const TimedPose& pose) {
                            return removed_ids.count(pose.frame_id) != 0U;
                        }),
                    pending_poses_.end());
                for (const std::uint64_t frame_id : frame_removals)
                    processing_frame_removals_.erase(frame_id);
                completed_frame_removals_.insert(
                    results.removed_frame_ids.begin(),
                    results.removed_frame_ids.end());
                retained_frame_removals_.insert(
                    results.retained_frame_ids.begin(),
                    results.retained_frame_ids.end());
            }
            condition_.notify_all();
        } else if (rebuild || !poses.empty()) {
            ApplyPoseUpdates(std::move(poses), rebuild);
        } else if (have_frame) {
            ProcessFrame(std::move(frame));
        } else if (flush_request_id != 0) {
            if (generation_ != last_published_generation_)
                PublishSnapshot(last_frame_time_, true);
            {
                std::lock_guard<std::mutex> lock(mutex_);
                flush_completed_id_ =
                    std::max(flush_completed_id_, flush_request_id);
            }
            condition_.notify_all();
        }
    }
}

}  // namespace autonomy::localization::atlas::map
