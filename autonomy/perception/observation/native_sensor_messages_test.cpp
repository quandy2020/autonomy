/* Copyright 2026 The Openbot Authors. */
/** @file native_sensor_messages_test.cpp
 * @brief Actual generated automsgs wire messages exercise production adapters.
 * @details These tests serialize and parse the repository's real Protobuf
 * definitions. They do not substitute message classes or certify Autolink reader,
 * real localization, mapper fusion, model inference or physical robot operation.
 */
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include "autonomy/perception/observation/native_sensor_messages.hpp"
#include "autonomy/perception/observation/observation_ingress.hpp"

namespace p = autonomy::perception;
namespace m = automsgs::msgs::sensor_msgs;
namespace {

/** @brief Report a failed production contract without requiring a mock framework. */
void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

/** @brief Round-trip the real generated class through its native wire format. */
template <typename Message>
Message WireRoundTrip(const Message& original) {
    std::string serialized;
    Require(original.SerializeToString(&serialized), "actual Protobuf serialization");
    Message decoded;
    Require(decoded.ParseFromString(serialized), "actual Protobuf parsing");
    return decoded;
}

/** @brief Encode scalar payload bits in a specified wire byte order. */
void Word(std::string& payload, std::size_t offset, std::uint64_t value,
          unsigned bytes, bool big_endian = false) {
    for (unsigned i = 0; i < bytes; ++i)
        payload.at(offset + i) = static_cast<char>(
            (value >> (8 * (big_endian ? bytes - 1 - i : i))) & 255);
}

/** @brief Encode an IEEE FLOAT32 point field independently of the decoder. */
void Float(std::string& payload, std::size_t offset, float value) {
    std::uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    Word(payload, offset, bits, 4);
}

/** @brief Build explicit rectified calibration in a native CameraInfo message. */
m::CameraInfo CameraInfo() {
    m::CameraInfo info;
    info.mutable_header()->set_frame_id("depth_optical");
    info.set_width(2);
    info.set_height(2);
    for (double value : {100., 0., .5, 0., 110., .5, 0., 0., 1.}) info.add_k(value);
    for (double value : {100., 0., .5, 0., 0., 110., .5, 0., 0., 0., 1., 0.}) info.add_p(value);
    return info;
}

/** @brief Build stamped native depth with real big-endian padded image storage. */
m::Image DepthImage() {
    m::Image image;
    image.mutable_header()->set_frame_id("depth_optical");
    image.mutable_header()->mutable_stamp()->set_sec(3);
    image.mutable_header()->mutable_stamp()->set_nanosec(250000000);
    image.set_width(2);
    image.set_height(2);
    image.set_step(6);
    image.set_encoding("16UC1");
    image.set_is_bigendian(true);
    std::string payload(12, '\0');
    Word(payload, 0, 1000, 2, true);
    Word(payload, 6, 2000, 2, true);
    image.set_data(payload);
    return image;
}

/** @brief Build native hit-only point records with explicit actual relative time. */
m::PointCloud2 PointCloud() {
    m::PointCloud2 cloud;
    cloud.mutable_header()->set_frame_id("lidar");
    cloud.mutable_header()->mutable_stamp()->set_sec(3);
    cloud.mutable_header()->mutable_stamp()->set_nanosec(250000000);
    cloud.set_width(2);
    cloud.set_height(1);
    cloud.set_point_step(16);
    cloud.set_row_step(36);
    for (const auto& entry : {std::pair<const char*, unsigned>{"x", 0},
         {"y", 4}, {"z", 8}, {"time", 12}}) {
        auto* field = cloud.add_fields();
        field->set_name(entry.first);
        field->set_offset(entry.second);
        field->set_count(1);
        field->set_datatype(m::PointField::FLOAT32);
    }
    std::string payload(36, '\0');
    Float(payload, 0, 1);
    Float(payload, 16, 1);
    Float(payload, 28, .1F);
    cloud.set_data(payload);
    return cloud;
}

/** @brief Image serialization retains timestamp, byte order, padding and units. */
void DepthWireLayout() {
    auto image = WireRoundTrip(DepthImage());
    auto info = WireRoundTrip(CameraInfo());
    const auto bytes = p::NativeDepthImage(image);
    const auto calibration = p::NativeDepthCalibration(info, 7, true, .1, 8);
    Require(bytes && calibration && bytes->stamp == 3.25 && bytes->row_step == 6 &&
        bytes->big_endian && bytes->data.data() == image.data().data(),
        "native descriptor borrows real padded wire payload and acquisition time");
    const auto frame = p::DecodeDepthImage(*bytes, *calibration, {});
    Require(frame && frame->depth.size() == 4 && frame->depth[0] == 1000 &&
        frame->depth[2] == 2000 && frame->depth_scale == .001,
        "native units and explicit scale survive byte decoding");
    std::optional<p::ObservationPacket> received;
    p::ObservationIngress ingress(p::SensorMode::kRgbdOnly, {},
        [](auto sensor, const auto& sensor_frame, double stamp) {
            p::AcquisitionLocalization proof;
            proof.world_frame = "map";
            proof.sensor_frame = sensor_frame;
            proof.map_epoch = 4;
            proof.calibration_revision = 7;
            proof.support = p::Mask(sensor);
            proof.healthy = true;
            proof.valid_from = stamp - 1;
            proof.valid_until = stamp + 1;
            return proof;
        }, [&](auto packet, double) { received = std::move(packet); return true; },
        [] { return 3.4; });
    Require(ingress.PushDepth(*bytes, *calibration, 1) == p::ObservationIngressResult::kAccepted &&
        received && received->depth_view && received->depth_view->axial_depth[0] == 1 &&
        received->depth_view->axial_depth[2] == 2 && !received->depth_view->valid[1] &&
        !received->depth_view->valid[3] && received->observation.valid_fraction == .5,
        "wire-native depth is scaled once with its invalid-pixel denominator");
}

/** @brief Metadata rejection is distinct from later layout and byte validation. */
void HeaderAndPayloadRejection() {
    auto image = DepthImage();
    image.clear_header();
    Require(!p::NativeDepthImage(image), "missing native header is rejected");
    image = DepthImage();
    image.mutable_header()->clear_stamp();
    Require(!p::NativeDepthImage(image), "absent acquisition time is not arrival time");
    image = DepthImage();
    image.mutable_header()->mutable_stamp()->set_nanosec(1000000000);
    Require(!p::NativeDepthImage(image), "invalid nanoseconds cannot alter clock domain");
    image = DepthImage();
    image.mutable_header()->set_frame_id("");
    Require(!p::NativeDepthImage(image), "empty sensor frame is rejected");
    image = DepthImage();
    image.set_data(std::string(1, '\0'));
    const auto bytes = p::NativeDepthImage(image);
    const auto calibration = p::NativeDepthCalibration(CameraInfo(), 7, true, .1, 8);
    Require(bytes && calibration && !p::DecodeDepthImage(*bytes, *calibration, {}),
        "truncated native payload is rejected before reading pixels");
}

/** @brief Explicit rectified P or undistorted K must match the actual depth raster. */
void CalibrationContract() {
    auto info = CameraInfo();
    info.add_d(.2);
    auto calibration = p::NativeDepthCalibration(WireRoundTrip(info), 7, true, .1, 8);
    Require(calibration && calibration->camera.fx == 100 && calibration->camera.fy == 110,
        "explicit rectified projection is used rather than original raw distortion");
    Require(!p::NativeDepthCalibration(info, 0, true, .1, 8) &&
        !p::NativeDepthCalibration(info, 7, false, .1, 8),
        "calibration revision and stream rectification must be explicit");
    info.mutable_p()->Set(3, .1);
    Require(!p::NativeDepthCalibration(info, 7, true, .1, 8),
        "shifted projective origin is not silently treated as axial depth");
    info = CameraInfo();
    info.clear_p();
    Require(p::NativeDepthCalibration(info, 7, true, .1, 8).has_value(),
        "undistorted K fallback is supported");
    info.add_d(.1);
    Require(!p::NativeDepthCalibration(info, 7, true, .1, 8),
        "distorted K cannot represent rectified depth pixels");
}

/** @brief Binning, crop, dimensions and range require adjusted calibration. */
void RasterAndRangeRejection() {
    auto info = CameraInfo();
    info.set_binning_x(2);
    Require(!p::NativeDepthCalibration(info, 7, true, .1, 8), "unadjusted binning rejected");
    info = CameraInfo();
    info.mutable_roi()->set_x_offset(1);
    Require(!p::NativeDepthCalibration(info, 7, true, .1, 8), "unadjusted cropped ROI rejected");
    info = CameraInfo();
    info.set_width(0);
    Require(!p::NativeDepthCalibration(info, 7, true, .1, 8), "zero raster width rejected");
    info = CameraInfo();
    Require(!p::NativeDepthCalibration(info, 7, true,
        std::numeric_limits<double>::quiet_NaN(), 8), "nonfinite metric range rejected");
}

/** @brief Actual typed point fields and times survive Protobuf transport. */
void CloudWireLayout() {
    auto cloud = WireRoundTrip(PointCloud());
    const auto bytes = p::NativePointCloud(cloud);
    Require(bytes && bytes->stamp == 3.25 && bytes->fields.size() == 4 &&
        bytes->row_step == 36 && bytes->data.data() == cloud.data().data(),
        "native cloud preserves field metadata, row padding and actual time");
    const auto scan = p::DecodePointCloud(*bytes, {});
    Require(scan && scan->beams.size() == 2 && std::abs(scan->duration - .1) < 1e-6,
        "actual point time is decoded rather than inferred from receipt");
    cloud.mutable_fields()->RemoveLast();
    const auto untimed = p::NativePointCloud(cloud);
    Require(untimed && !p::DecodePointCloud(*untimed, {}),
        "rolling native cloud without times is not fabricated as instantaneous");
}

/** @brief Native unknown enums and ambiguous coordinate fields cannot be decoded. */
void CloudFieldRejection() {
    auto cloud = PointCloud();
    cloud.mutable_fields(0)->set_datatype(m::PointField::DATA_TYPE_UNKNOWN);
    Require(!p::NativePointCloud(WireRoundTrip(cloud)), "unknown point field enum rejected");
    cloud = PointCloud();
    *cloud.add_fields() = cloud.fields(0);
    const auto bytes = p::NativePointCloud(cloud);
    Require(bytes && !p::DecodePointCloud(*bytes, {}), "duplicate native coordinates rejected");
    cloud = PointCloud();
    for (unsigned i = 0; i < 253; ++i) cloud.add_fields();
    Require(!p::NativePointCloud(cloud), "native field metadata allocation is bounded");
}

/** @brief Wire-native input reaches real ingress with explicit external pose proofs. */
void WireToAcquisitionIngress() {
    auto cloud = WireRoundTrip(PointCloud());
    const auto bytes = p::NativePointCloud(cloud);
    Require(bytes.has_value(), "native timed descriptor available");
    std::optional<p::ObservationPacket> received;
    p::ObservationIngress ingress(p::SensorMode::kLidarOnly, {},
        [](auto sensor, const auto& frame, double stamp) {
            p::AcquisitionLocalization proof;
            proof.world_frame = "map";
            proof.sensor_frame = frame;
            proof.map_epoch = 4;
            proof.calibration_revision = 7;
            proof.support = p::Mask(sensor);
            proof.healthy = true;
            proof.valid_from = 3;
            proof.valid_until = 4;
            proof.world_from_base.translation.x = (stamp - 3.25) * 10;
            proof.world_from_sensor = proof.world_from_base;
            return proof;
        },
        [&](auto packet, double) { received = std::move(packet); return true; },
        [] { return 3.4; });
    Require(ingress.PushCloud(*bytes, 1) == p::ObservationIngressResult::kAccepted &&
        received && received->observation.rays.size() == 2 && received->acquisition_poses.size() == 2 &&
        std::abs(received->observation.rays[1].Endpoint().x - 2) < 1e-5,
        "wire-native points use individual acquisition-time transformations");
    for (const auto& ray : received->observation.rays)
        Require(ray.hit, "hit-only PointCloud2 cannot invent no-return free rays");
}

}  // namespace

int main() {
    const std::pair<const char*, void (*)()> scenarios[] = {
        {"native depth wire layout", DepthWireLayout},
        {"native header payload rejection", HeaderAndPayloadRejection},
        {"native calibration contract", CalibrationContract},
        {"native raster range rejection", RasterAndRangeRejection},
        {"native cloud wire layout", CloudWireLayout},
        {"native cloud field rejection", CloudFieldRejection},
        {"native wire acquisition ingress", WireToAcquisitionIngress}};
    for (const auto& scenario : scenarios) {
        try {
            scenario.second();
            std::cout << "PASS " << scenario.first << '\n';
        } catch (const std::exception& error) {
            std::cerr << "FAIL " << scenario.first << ": " << error.what() << '\n';
            return 1;
        }
    }
}
