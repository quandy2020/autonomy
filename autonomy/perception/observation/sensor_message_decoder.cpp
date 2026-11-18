/* Copyright 2026 The Openbot Authors. */
/** @file sensor_message_decoder.cpp
 * @brief Checked raster/point strides and endian-neutral measurement decoding.
 */
#include "autonomy/perception/observation/sensor_message_decoder.hpp"

#include <cstring>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace autonomy::perception {
namespace {
/** @brief Return a declared scalar width; zero denotes an unsupported type. */
std::size_t Width(FieldScalar scalar) {
    switch (scalar) {
        case FieldScalar::kInt8: case FieldScalar::kUint8: return 1;
        case FieldScalar::kInt16: case FieldScalar::kUint16: return 2;
        case FieldScalar::kInt32: case FieldScalar::kUint32:
        case FieldScalar::kFloat32: return 4;
        case FieldScalar::kFloat64: return 8;
    }
    return 0;
}
/** @brief Load bits without alignment assumptions or host-endian dependence. */
std::uint64_t Bits(const char* bytes, std::size_t size, bool big_endian) {
    std::uint64_t bits = 0;
    for (std::size_t i = 0; i < size; ++i) {
        const auto shift = 8 * (big_endian ? size - 1 - i : i);
        bits |= static_cast<std::uint64_t>(static_cast<unsigned char>(bytes[i])) << shift;
    }
    return bits;
}
/** @brief Decode a declared scalar; memcpy avoids strict-aliasing violations. */
double Scalar(const char* bytes, FieldScalar type, bool big_endian) {
    const auto bits = Bits(bytes, Width(type), big_endian);
    switch (type) {
        case FieldScalar::kFloat32: {
            const auto word = static_cast<std::uint32_t>(bits);
            float value; std::memcpy(&value, &word, sizeof(value)); return value;
        }
        case FieldScalar::kFloat64: {
            double value; std::memcpy(&value, &bits, sizeof(value)); return value;
        }
        case FieldScalar::kInt8: return static_cast<std::int8_t>(bits);
        case FieldScalar::kInt16: return static_cast<std::int16_t>(bits);
        case FieldScalar::kInt32: return static_cast<std::int32_t>(bits);
        default: return static_cast<double>(bits);
    }
}
/** @brief Check a complete bounded raster before allocating or reading bytes. */
bool Layout(std::uint32_t width, std::uint32_t height, std::uint32_t item_step,
    std::uint32_t row_step, std::size_t storage, std::size_t maximum_items,
    std::size_t maximum_bytes) {
    if (!width || !height || !item_step || !row_step || storage > maximum_bytes) return false;
    const std::uint64_t count = static_cast<std::uint64_t>(width) * height;
    const std::uint64_t row_bytes = static_cast<std::uint64_t>(width) * item_step;
    const std::uint64_t bytes = static_cast<std::uint64_t>(row_step) * height;
    return count <= maximum_items && row_bytes <= row_step && bytes == storage;
}
/** @brief Return a uniquely named field, rejecting missing/ambiguous metadata. */
const CloudField* Find(const PointCloudBytes& cloud, const std::string& name) {
    const CloudField* found = nullptr;
    for (const auto& field : cloud.fields) {
        if (field.name == name) {
            if (found) return nullptr;
            found = &field;
        }
    }
    return found;
}
/** @brief Required XYZ fields contain one finite-width floating-point scalar. */
bool Coordinate(const CloudField* field) {
    return field && field->count == 1 &&
        (field->scalar == FieldScalar::kFloat32 || field->scalar == FieldScalar::kFloat64);
}
}  // namespace

void ValidateSensorDecodingOptions(const SensorDecodingOptions& o) {
    if (!std::isfinite(o.uint16_depth_scale) || o.uint16_depth_scale <= 0 ||
        !std::isfinite(o.minimum_range) || o.minimum_range <= 0 ||
        !std::isfinite(o.maximum_range) || o.maximum_range <= o.minimum_range ||
        !std::isfinite(o.time_offset_scale) || o.time_offset_scale <= 0 ||
        !std::isfinite(o.maximum_scan_duration) || o.maximum_scan_duration < 0 ||
        (!o.instantaneous_cloud && (o.maximum_scan_duration == 0 || o.time_offset_field.empty())) ||
        !std::isfinite(o.valid_for) || o.valid_for <= 0 ||
        !o.maximum_image_pixels || !o.maximum_cloud_points || !o.maximum_payload_bytes ||
        o.expected_scan_points > o.maximum_cloud_points)
        throw std::invalid_argument("invalid sensor decoding policy");
}

std::optional<RgbdFrame> DecodeDepthImage(const DepthImageBytes& image,
    const DepthCalibration& calibration, const SensorDecodingOptions& options) {
    ValidateSensorDecodingOptions(options);
    const auto step = image.encoding == "16UC1" ? 2U : image.encoding == "32FC1" ? 4U : 0U;
    if (!step || !calibration.revision || !calibration.rectified ||
        !calibration.camera.Valid() || image.frame_id.empty() ||
        image.frame_id != calibration.frame_id || !std::isfinite(image.stamp) ||
        image.width != static_cast<std::uint32_t>(calibration.camera.width) ||
        image.height != static_cast<std::uint32_t>(calibration.camera.height) ||
        !Layout(image.width, image.height, step, image.row_step, image.data.size(),
            options.maximum_image_pixels, options.maximum_payload_bytes)) return {};
    RgbdFrame frame;
    frame.camera = calibration.camera; frame.stamp = image.stamp;
    frame.frame_id = image.frame_id; frame.valid_for = options.valid_for;
    frame.depth_scale = step == 2 ? options.uint16_depth_scale : 1;
    frame.depth.reserve(static_cast<std::size_t>(image.width) * image.height);
    for (std::uint32_t y = 0; y < image.height; ++y) {
        for (std::uint32_t x = 0; x < image.width; ++x) {
            const auto offset = static_cast<std::size_t>(y) * image.row_step + x * step;
            const auto type = step == 2 ? FieldScalar::kUint16 : FieldScalar::kFloat32;
            frame.depth.push_back(static_cast<float>(Scalar(image.data.data() + offset, type, image.big_endian)));
        }
    }
    return frame;
}

std::optional<LidarScan> DecodePointCloud(const PointCloudBytes& cloud,
    const SensorDecodingOptions& options) {
    ValidateSensorDecodingOptions(options);
    if (cloud.frame_id.empty() || !std::isfinite(cloud.stamp) ||
        !Layout(cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data.size(),
            options.maximum_cloud_points, options.maximum_payload_bytes)) return {};
    std::unordered_set<std::string> names;
    for (const auto& field : cloud.fields) {
        const auto size = Width(field.scalar);
        if (field.name.empty() || !names.insert(field.name).second || !size || !field.count ||
            static_cast<std::uint64_t>(field.offset) + size * static_cast<std::uint64_t>(field.count) > cloud.point_step)
            return {};
    }
    const auto* x = Find(cloud, "x"); const auto* y = Find(cloud, "y"); const auto* z = Find(cloud, "z");
    const auto* time = options.instantaneous_cloud ? nullptr : Find(cloud, options.time_offset_field);
    if (!Coordinate(x) || !Coordinate(y) || !Coordinate(z) ||
        (!options.instantaneous_cloud && (!time || time->count != 1))) return {};
    std::vector<const CloudField*> required{x, y, z};
    if (time) required.push_back(time);
    for (std::size_t i = 0; i < required.size(); ++i) {
        for (std::size_t j = i + 1; j < required.size(); ++j) {
            const auto* a = required[i]; const auto* b = required[j];
            if (a->offset < b->offset + Width(b->scalar) && b->offset < a->offset + Width(a->scalar)) return {};
        }
    }
    LidarScan scan;
    scan.stamp = cloud.stamp; scan.frame_id = cloud.frame_id;
    scan.min_range = options.minimum_range; scan.max_range = options.maximum_range;
    scan.valid_for = options.valid_for;
    const auto count = static_cast<std::size_t>(cloud.width) * cloud.height;
    if (options.expected_scan_points && count > options.expected_scan_points) return {};
    scan.completeness = options.expected_scan_points ?
        static_cast<double>(count) / options.expected_scan_points : 1;
    scan.beams.reserve(count);
    for (std::uint32_t row = 0; row < cloud.height; ++row) {
        for (std::uint32_t column = 0; column < cloud.width; ++column) {
            const char* record = cloud.data.data() + static_cast<std::size_t>(row) * cloud.row_step +
                static_cast<std::size_t>(column) * cloud.point_step;
            const Vec3 point{Scalar(record + x->offset, x->scalar, cloud.big_endian),
                Scalar(record + y->offset, y->scalar, cloud.big_endian),
                Scalar(record + z->offset, z->scalar, cloud.big_endian)};
            LidarBeam beam;
            beam.range = point.Norm();
            beam.time_offset = time ? Scalar(record + time->offset, time->scalar, cloud.big_endian) *
                options.time_offset_scale : 0;
            beam.valid = point.Finite() && std::isfinite(beam.range) &&
                beam.range >= scan.min_range && beam.range <= scan.max_range &&
                std::isfinite(beam.time_offset) && beam.time_offset >= 0 &&
                beam.time_offset <= options.maximum_scan_duration;
            if (std::isfinite(beam.time_offset) && beam.time_offset >= 0 &&
                beam.time_offset <= options.maximum_scan_duration)
                scan.duration = std::max(scan.duration, beam.time_offset);
            if (beam.valid) beam.direction = point * (1 / beam.range);
            scan.beams.push_back(beam);
        }
    }
    return scan;
}
}  // namespace autonomy::perception
