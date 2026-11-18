/* Copyright 2026 The Openbot Authors. */
/** @file native_sensor_messages.cpp
 * @brief Native metadata decoding without invented sequences, units or transforms.
 */
#include "autonomy/perception/observation/native_sensor_messages.hpp"

#include <cmath>

namespace autonomy::perception {
namespace {
/** @brief Validate seconds/nanoseconds before conversion to the shared clock. */
std::optional<double> Stamp(const ::automsgs::msgs::std_msgs::Header& header) {
    if (!header.has_stamp() || header.frame_id().empty() || header.stamp().nanosec() >= 1000000000)
        return {};
    return static_cast<double>(header.stamp().sec()) + header.stamp().nanosec() * 1e-9;
}
}  // namespace

std::optional<DepthImageBytes> NativeDepthImage(const ::automsgs::msgs::sensor_msgs::Image& image) {
    if (!image.has_header()) return {};
    const auto stamp = Stamp(image.header());
    if (!stamp) return {};
    return DepthImageBytes{image.width(), image.height(), image.step(), image.is_bigendian(),
        image.encoding(), image.header().frame_id(), *stamp, image.data()};
}

std::optional<PointCloudBytes> NativePointCloud(const ::automsgs::msgs::sensor_msgs::PointCloud2& cloud) {
    if (!cloud.has_header() || cloud.fields_size() > 256) return {};
    const auto stamp = Stamp(cloud.header());
    if (!stamp) return {};
    PointCloudBytes bytes;
    bytes.width = cloud.width(); bytes.height = cloud.height();
    bytes.point_step = cloud.point_step(); bytes.row_step = cloud.row_step();
    bytes.big_endian = cloud.is_bigendian(); bytes.frame_id = cloud.header().frame_id();
    bytes.stamp = *stamp; bytes.data = cloud.data();
    for (const auto& field : cloud.fields()) {
        if (field.datatype() < 1 || field.datatype() > 8) return {};
        bytes.fields.push_back({field.name(), field.offset(),
            static_cast<FieldScalar>(field.datatype()), field.count()});
    }
    return bytes;
}

std::optional<DepthCalibration> NativeDepthCalibration(
    const ::automsgs::msgs::sensor_msgs::CameraInfo& info, std::uint64_t revision,
    bool rectified, double minimum_depth, double maximum_depth) {
    if (!info.has_header() || info.header().frame_id().empty() || !revision || !rectified ||
        info.width() > 16384 || info.height() > 16384 ||
        info.binning_x() > 1 || info.binning_y() > 1) return {};
    if (info.has_roi() && (info.roi().x_offset() || info.roi().y_offset() ||
        (info.roi().width() && info.roi().width() != info.width()) ||
        (info.roi().height() && info.roi().height() != info.height()))) return {};
    CameraModel camera;
    camera.width = static_cast<int>(info.width()); camera.height = static_cast<int>(info.height());
    camera.min_depth = minimum_depth; camera.max_depth = maximum_depth;
    if (info.p_size() == 12) {
        for (double value : info.p()) if (!std::isfinite(value)) return {};
        if (info.p(1) || info.p(3) || info.p(4) || info.p(7) || info.p(8) || info.p(9) ||
            info.p(10) != 1 || info.p(11)) return {};
        camera.fx = info.p(0); camera.fy = info.p(5); camera.cx = info.p(2); camera.cy = info.p(6);
    } else {
        if (info.p_size() || info.k_size() != 9) return {};
        for (double value : info.d()) if (!std::isfinite(value) || value != 0) return {};
        for (double value : info.k()) if (!std::isfinite(value)) return {};
        if (info.k(1) || info.k(3) || info.k(6) || info.k(7) || info.k(8) != 1) return {};
        camera.fx = info.k(0); camera.fy = info.k(4); camera.cx = info.k(2); camera.cy = info.k(5);
    }
    if (!camera.Valid()) return {};
    return DepthCalibration{camera, info.header().frame_id(), revision, rectified};
}
}  // namespace autonomy::perception
