/* Copyright 2026 The Openbot Authors. */
/** @file sensor_message_decoder.hpp
 * @brief Bounded decoding of native depth images and timed sensor-frame clouds.
 */
#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "autonomy/perception/observation/lidar_observation_adapter.hpp"
#include "autonomy/perception/observation/rgbd_observation_adapter.hpp"

namespace autonomy::perception {

/** @brief Scalar encodings matching the native PointField message values. */
enum class FieldScalar : std::uint8_t {
    kInt8 = 1, kUint8 = 2, kInt16 = 3, kUint16 = 4,
    kInt32 = 5, kUint32 = 6, kFloat32 = 7, kFloat64 = 8
};

/** @brief One nonoverlapping scalar field in a binary point record. */
struct CloudField {
    std::string name;  ///< Exact native field name, without guessed aliases.
    std::uint32_t offset = 0;  ///< Byte offset within one point record.
    FieldScalar scalar = FieldScalar::kFloat32;  ///< Explicit scalar encoding.
    std::uint32_t count = 1;  ///< Required fields must contain exactly one value.
};

/** @brief Borrowed native image bytes; valid for the synchronous decode call. */
struct DepthImageBytes {
    std::uint32_t width = 0, height = 0;  ///< Raster dimensions in pixels.
    std::uint32_t row_step = 0;  ///< Encoded bytes per row, including padding.
    bool big_endian = false;  ///< Byte order of each encoded scalar.
    std::string encoding;  ///< Supported encodings: 16UC1 or 32FC1.
    std::string frame_id;  ///< Actual depth optical frame, not the RGB frame.
    double stamp = 0;  ///< Acquisition time in the owner clock domain, seconds.
    std::string_view data;  ///< Borrowed encoded storage, including row padding.
};

/** @brief Borrowed sensor-frame cloud bytes with an explicit point layout. */
struct PointCloudBytes {
    std::uint32_t width = 0, height = 0;  ///< Encoded point grid dimensions.
    std::uint32_t point_step = 0, row_step = 0;  ///< Point/row byte strides.
    bool big_endian = false;  ///< Encoded scalar byte order.
    std::string frame_id;  ///< Sensor frame of x/y/z; registered map clouds are not raw scans.
    double stamp = 0;  ///< Scan start time; per-point times are relative to it.
    std::vector<CloudField> fields;  ///< Native layout, including irrelevant fields.
    std::string_view data;  ///< Borrowed encoded storage.
};

/** @brief Fixed calibration for a rectified depth stream. */
struct DepthCalibration {
    CameraModel camera;  ///< Intrinsics of the actual encoded raster.
    std::string frame_id;  ///< Must exactly match the depth optical frame.
    std::uint64_t revision = 0;  ///< Nonzero immutable calibration identity.
    bool rectified = false;  ///< Producer explicitly asserts pinhole rectification.
};

/** @brief Explicit decoding/timing policy; no automatic unit or timing inference. */
struct SensorDecodingOptions {
    double uint16_depth_scale = .001;  ///< Meters per 16UC1 native unit; 32FC1 is already meters.
    double minimum_range = .1, maximum_range = 30;  ///< LiDAR ray limits, meters.
    std::string time_offset_field = "time";  ///< Relative per-point time field.
    double time_offset_scale = 1;  ///< Seconds per encoded time unit.
    double maximum_scan_duration = .5;  ///< Maximum permitted relative time, seconds.
    bool instantaneous_cloud = false;  ///< Only true for an explicitly simultaneous acquisition.
    std::size_t expected_scan_points = 0;  ///< Known encoded count; zero means delivery completeness unknown.
    double valid_for = .5;  ///< Evidence lifetime after acquisition, seconds.
    std::size_t maximum_image_pixels = 1920 * 1080;  ///< Allocation/work bound for depth input.
    std::size_t maximum_cloud_points = 200000;  ///< Allocation/work bound for cloud input.
    std::size_t maximum_payload_bytes = 64 * 1024 * 1024;  ///< Encoded message byte bound.
};

/** @brief Validates all finite ranges and positive work limits.
 * @throws std::invalid_argument If a policy is malformed.
 */
void ValidateSensorDecodingOptions(const SensorDecodingOptions& options);

/** @brief Decode owned native depth samples, preserving invalid pixels.
 * @param image Borrowed raster; padding and endian order are honored.
 * @param calibration Rectified intrinsics of this exact stream.
 * @param options Explicit unit/lifetime/work limits.
 * @return Owned unlocalized frame, or nullopt for an invalid layout/policy.
 * Localization must be supplied before adapting rays. Empty/invalid depth
 * remains unknown and is never converted into a no-hit free-space ray.
 */
std::optional<RgbdFrame> DecodeDepthImage(const DepthImageBytes& image,
    const DepthCalibration& calibration, const SensorDecodingOptions& options);

/** @brief Decode owned XYZ beams and their actual relative acquisition times.
 * @param cloud Raw sensor-frame XYZ records, not registered map points.
 * @param options Declared time field/units or explicit simultaneous policy.
 * @return Owned unlocalized scan; invalid individual measurements remain invalid.
 * Missing timing rejects a rolling scan. No-hit rays cannot be reconstructed
 * from a hit-only PointCloud2, and missing scan packets cannot be inferred here.
 */
std::optional<LidarScan> DecodePointCloud(const PointCloudBytes& cloud,
    const SensorDecodingOptions& options);

}  // namespace autonomy::perception
