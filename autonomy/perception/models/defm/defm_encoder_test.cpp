/* Copyright 2026 The Openbot Authors (duyongquan). */
/** @file defm_encoder_test.cpp
 * @brief Real pretrained DeFM/ORT parity, native image decoding and shared cache tests.
 */
#include "autonomy/perception/models/defm/defm_encoder.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>

#include "nlohmann/json.hpp"
#include "autonomy/control/controller/policy_controller/observation/buffer.hpp"
#include "autonomy/control/controller/policy_controller/observation/defm_features.hpp"
#include "autonomy/control/controller/policy_controller/observation/encoder.hpp"
#include "autonomy/control/controller/policy_controller/observation/tensor_frame_encoder.hpp"

namespace d = autonomy::perception::defm;
namespace p = autonomy::control::controller::policy_controller;
namespace proto = autonomy::control::proto;
using Image = automsgs::msgs::sensor_msgs::Image;
namespace {
/** @brief Fail a scenario with an explicit contract or real inference error. */
void Require(bool condition, const std::string& reason) {
    if (!condition) throw std::runtime_error(reason);
}

/** @brief Build a padded native raster, explicitly encoding scalar byte order. */
std::shared_ptr<const Image> DepthImage(const std::vector<float>& values,
                                      int height, int width, bool big_endian = false,
                                      bool uint16 = false) {
    auto image = std::make_shared<Image>();
    image->mutable_header()->set_frame_id("depth_optical");
    image->mutable_header()->mutable_stamp()->set_sec(1000);
    image->set_height(height); image->set_width(width);
    image->set_encoding(uint16 ? "16UC1" : "32FC1"); image->set_is_bigendian(big_endian);
    const std::size_t element = uint16 ? 2 : 4;
    const std::size_t stride = width * element + 8;
    image->set_step(stride);
    std::string storage(height * stride, '\x55');
    Require(values.size() == static_cast<std::size_t>(height) * width, "fixture raster size");
    for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x) {
        const float value = values[static_cast<std::size_t>(y) * width + x];
        std::uint32_t bits = 0;
        if (uint16) bits = static_cast<std::uint16_t>(std::clamp(std::round(value * 1000), 0.f, 65535.f));
        else std::memcpy(&bits, &value, sizeof(value));
        for (std::size_t b = 0; b < element; ++b) {
            const auto shift = big_endian ? element - 1 - b : b;
            storage[static_cast<std::size_t>(y) * stride + x * element + b] = static_cast<char>(bits >> (8 * shift));
        }
    }
    image->set_data(std::move(storage));
    return image;
}

/** @brief Compare actual native model values with independent official Torch fixtures. */
void Near(const autonomy::common::network::Tensor& tensor, const std::vector<float>& expected) {
    const float* values = nullptr; std::size_t count = 0;
    Require(tensor.TryViewFloat32(&values, &count) && count == expected.size(), "actual output shape/type");
    for (std::size_t i = 0; i < count; ++i)
        Require(std::isfinite(values[i]) && std::abs(values[i] - expected[i]) <= 3e-4 + 3e-4 * std::abs(expected[i]),
                "actual DeFM output disagrees with official Torch fixture at " + std::to_string(i));
}

/** @brief Reuse one real calibrated exported model, never a synthetic inference seam. */
d::DefmEncoderOptions Options(const std::string& model, const nlohmann::json& fixture) {
    d::DefmEncoderOptions o;
    o.inference.model_path = model; o.inference.backend_id = "onnx";
    o.calibration.camera.height = fixture.at("camera_height");
    o.calibration.camera.width = fixture.at("camera_width");
    o.calibration.camera.fx = 500; o.calibration.camera.fy = 500;
    o.calibration.camera.cx = o.calibration.camera.width / 2.;
    o.calibration.camera.cy = o.calibration.camera.height / 2.;
    o.calibration.camera.min_depth = .1; o.calibration.camera.max_depth = 8;
    o.calibration.frame_id = "depth_optical"; o.calibration.revision = 7; o.calibration.rectified = true;
    o.target_height = fixture.at("target_height"); o.target_width = fixture.at("target_width");
    // CPU parity tests authorize a long acquisition lifetime. Production defaults
    // remain .2 seconds and reject results that expire while the engine runs.
    o.maximum_age = 10; o.decoding.valid_for = 10; o.cache_capacity = 2;
    return o;
}

/** @brief Actual feature injection into the native buffer, encoder and finite lease checks. */
void PolicyInput(const d::DefmDepthFeatures& features) {
    proto::ObservationInput input;
    input.set_name("patch_tokens"); input.set_modality(proto::OBSERVATION_RAW); input.set_required(true);
    for (const auto axis : {1, features.patch_count, features.feature_dimensions}) input.add_shape(axis);
    p::TensorFrame frame; std::string error;
    Require(p::MakeDefmTensorFrame(features, input, "patch_tokens", 1000.02, &frame, &error), error);
    Require(frame.stamp_sec == 1000 && frame.valid_until_sec == features.valid_until &&
            frame.shape == std::vector<int64_t>({1, features.patch_count, 384}), "injection renewed or reshaped acquisition");
    p::ObservationBuffer buffer; buffer.SetRawTensor(frame);
    auto snapshot = buffer.Snapshot();
    std::vector<float> values;
    Require(p::EncodeRawTensor(snapshot, input, &values, &error), error);
    Require(values == features.tensors.at("patch_tokens").ToFloat32(), "native RAW altered shared features");
    auto dynamic = input; dynamic.set_shape(1, -1);
    Require(p::EncodeRawTensor(snapshot, dynamic, &values, &error) && values == frame.data,
            "generic RAW lost its explicit single dynamic-axis contract");
    auto changed = input; changed.set_shape(1, features.patch_count / 2); changed.set_shape(2, 768);
    Require(!p::EncodeRawTensor(snapshot, changed, &values, &error) && values.empty(), "same-size different axes accepted");
    auto damaged = frame; damaged.data.pop_back(); buffer.SetRawTensor(damaged);
    Require(!p::EncodeRawTensor(buffer.Snapshot(), input, &values, &error), "RAW padded truncated token payload");
    damaged = frame; damaged.data.push_back(0); buffer.SetRawTensor(damaged);
    Require(!p::EncodeRawTensor(buffer.Snapshot(), input, &values, &error), "RAW truncated extra token payload");
    damaged = frame; damaged.data[0] = std::numeric_limits<float>::quiet_NaN(); buffer.SetRawTensor(damaged);
    Require(!p::EncodeRawTensor(buffer.Snapshot(), input, &values, &error), "RAW accepted nonfinite token");
    damaged = frame; damaged.shape = {std::numeric_limits<int64_t>::max(), std::numeric_limits<int64_t>::max(), 384};
    buffer.SetRawTensor(damaged);
    Require(!p::EncodeRawTensor(buffer.Snapshot(), input, &values, &error), "overflowing RAW shape accepted");
    buffer.SetRawTensor(frame); snapshot = buffer.Snapshot();
    proto::PolicyConfig declaration;
    declaration.add_inputs()->CopyFrom(input);
    declaration.mutable_action()->set_type(proto::ACTION_SPACE_TWIST);
    declaration.mutable_action()->set_name("action");
    p::PolicyConfig config;
    Require(p::PolicyConfig::LoadFromProto(declaration, &config, &error), error);
    Require(p::ValidateRawTensorLifetimes(snapshot, config, 1000.03, &error), error);
    Require(!p::ValidateRawTensorLifetimes(snapshot, config, features.valid_until, &error), "lease accepted at expiry");
    Require(!p::ValidateRawTensorLifetimes(snapshot, config, 999, &error), "future raw acquisition accepted");
    p::ObservationEncodeContext context; context.now_sec = features.valid_until;
    context.missing_modality_policy = proto::MISSING_MODALITY_ZERO_FILL;
    context.missing_modality_grace_sec = 10;
    snapshot.modality_last_wall_sec[input.name()] = context.now_sec - .01;
    autonomy::common::network::FloatTensorMap tensors; bool zero_filled = false;
    Require(!p::EncodeObservations(snapshot, config, context, &tensors, &error, &zero_filled) && !zero_filled,
            "expired real features were zero-filled under grace");
    changed = input; changed.set_shape(1, -1);
    Require(!p::MakeDefmTensorFrame(features, changed, "patch_tokens", 1000.02, &frame, &error) && frame.data.empty(),
            "unspecified DeFM axes accepted");
    Require(!p::MakeDefmTensorFrame(features, input, "patch_tokens", features.valid_until, &frame, &error), "expired features injected");
    Require(!p::MakeDefmTensorFrame(features, input, "patch_tokens", 999, &frame, &error), "future features injected");
    changed = input; changed.mutable_norm()->add_mean(.5);
    Require(!p::MakeDefmTensorFrame(features, changed, "patch_tokens", 1000.02, &frame, &error), "second normalization silently applied");
}

/** @brief Actual inference, original metadata, shared concurrent consumers and invalid frames. */
void Check(const std::string& model, const nlohmann::json& fixture) {
    auto options = Options(model, fixture);
    std::string error;
    auto encoder = d::DefmEncoder::Create(options, &error);
    Require(encoder != nullptr, error);
    const auto depth = fixture.at("depth_metric").get<std::vector<float>>();
    auto image = DepthImage(depth, options.calibration.camera.height, options.calibration.camera.width);
    const auto result = encoder->Encode(image, 1000.01, &error);
    Require(result != nullptr, error);
    Require(result->image == image && result->calibration_revision == 7 && result->stamp == 1000 &&
            result->valid_until == 1010 && result->patch_count == fixture.at("patch_count") &&
            result->feature_dimensions == fixture.at("feature_dimensions"), "original observation/token identity");
    Require(result->valid_fraction > .9 && !result->valid_pixels[0] && !result->valid_pixels[1] &&
            !result->valid_pixels[2] && !result->valid_pixels[3], "invalid depths became free-space evidence");
    for (const auto* name : {"normalized_depth", "patch_tokens", "global_feature"})
        Near(result->tensors.at(name), fixture.at(name).get<std::vector<float>>());
    PolicyInput(*result);
    std::vector<std::shared_ptr<const d::DefmDepthFeatures>> reads(4);
    std::vector<std::thread> consumers;
    for (std::size_t i = 0; i < reads.size(); ++i)
        consumers.emplace_back([&, i] { reads[i] = encoder->Encode(image, 1000.02); });
    for (auto& thread : consumers) thread.join();
    for (const auto& read : reads) Require(read == result, "same Image was independently re-encoded");
    Require(encoder->inference_count() == 1, "cached consumers performed another inference");
    Require(!encoder->Encode(image, 1011, &error) && !encoder->Encode(image, 999, &error), "stale/future observation accepted");
    Require(!encoder->Encode(nullptr, 1000.01, &error), "missing depth replaced with tokens");
    auto empty = DepthImage(std::vector<float>(depth.size(), 0), options.calibration.camera.height, options.calibration.camera.width);
    Require(!encoder->Encode(empty, 1000.01, &error), "all-invalid camera manufactured features");
    Require(encoder->inference_count() == 1, "invalid frame invoked real model");
    auto malformed = std::make_shared<Image>(*image); malformed->set_step(1);
    Require(!encoder->Encode(malformed, 1000.01, &error), "malformed row stride accepted");
    malformed = std::make_shared<Image>(*image); malformed->mutable_header()->set_frame_id("rgb_optical");
    Require(!encoder->Encode(malformed, 1000.01, &error), "wrong optical frame accepted");
    auto endian = DepthImage(depth, options.calibration.camera.height, options.calibration.camera.width, true);
    const auto second = encoder->Encode(endian, 1000.01, &error);
    Require(second != nullptr, error);
    Near(second->tensors.at("patch_tokens"), result->tensors.at("patch_tokens").ToFloat32());
    std::vector<float> quantized(depth.size());
    for (std::size_t i = 0; i < depth.size(); ++i)
        quantized[i] = static_cast<float>(std::clamp(std::round(depth[i] * 1000), 0.f, 65535.f) * .001);
    auto short_image = DepthImage(quantized, options.calibration.camera.height, options.calibration.camera.width, true, true);
    auto float_image = DepthImage(quantized, options.calibration.camera.height, options.calibration.camera.width);
    const auto from_short = encoder->Encode(short_image, 1000.01, &error);
    Require(from_short != nullptr, error);
    const auto from_float = encoder->Encode(float_image, 1000.01, &error);
    Require(from_float != nullptr, error);
    Near(from_short->tensors.at("patch_tokens"), from_float->tensors.at("patch_tokens").ToFloat32());
    encoder->ClearCache();
    Require(encoder->Encode(image, 1000.01, &error) != result, "clear cache returned the original object");
    auto bad = options; bad.calibration.camera.width += 14;
    Require(!d::DefmEncoder::Create(bad, &error), "wrong static model camera dimensions accepted");
    bad = options; bad.target_width += 14;
    Require(!d::DefmEncoder::Create(bad, &error), "wrong normalized/token dimensions accepted");
    bad = options; bad.calibration.rectified = false;
    Require(!d::DefmEncoder::Create(bad, &error), "unrectified camera silently accepted");
    bad = options; bad.inference.model_path.clear();
    Require(!d::DefmEncoder::Create(bad, &error), "missing weights replaced by a model");
}
/** @brief Actual model features are stacked without resampling, duplicate padding or lease renewal. */
void SharedHistory(const std::string& model, const nlohmann::json& fixture) {
    auto options = Options(model, fixture); options.cache_capacity = 4;
    std::string error;
    auto encoder = d::DefmEncoder::Create(options, &error); Require(encoder != nullptr, error);
    const auto depth = fixture.at("depth_metric").get<std::vector<float>>();
    std::vector<std::shared_ptr<const d::DefmDepthFeatures>> history;
    for (unsigned i = 0; i < 4; ++i) {
        auto image = std::make_shared<Image>(*DepthImage(depth, options.calibration.camera.height, options.calibration.camera.width));
        image->mutable_header()->mutable_stamp()->set_nanosec(i*100000000);
        auto actual = encoder->Encode(image, 1000.31, &error);
        Require(actual != nullptr, error); history.push_back(std::move(actual));
    }
    proto::ObservationInput input;
    input.set_name("observation_depth_tokens"); input.set_modality(proto::OBSERVATION_RAW); input.set_required(true);
    for (const auto axis : {1, 4, history.front()->patch_count, 384}) input.add_shape(axis);
    p::TensorFrame frame;
    Require(p::MakeDefmHistoryTensorFrame(history, input, 1000.31, &frame, &error), error);
    Require(frame.shape == std::vector<int64_t>({1,4,history.front()->patch_count,384}) &&
        frame.stamp_sec == history.back()->stamp && frame.valid_until_sec == history.front()->valid_until,
        "history preserves exact axes, newest acquisition and earliest original expiration");
    std::vector<float> expected;
    for (const auto& actual : history) {
        const auto values = actual->tensors.at("patch_tokens").ToFloat32();
        expected.insert(expected.end(),values.begin(),values.end());
    }
    Require(frame.data == expected && encoder->inference_count() == 4,
        "history assembly changed real tokens or performed another backbone inference");
    p::ObservationBuffer buffer; buffer.SetRawTensor(frame);
    std::vector<float> encoded;
    Require(p::EncodeRawTensor(buffer.Snapshot(),input,&encoded,&error) && encoded == expected,
        "existing native RAW encoder preserves the actual four-frame token order");
    auto invalid = history; std::swap(invalid[0],invalid[1]);
    Require(!p::MakeDefmHistoryTensorFrame(invalid,input,1000.31,&frame,&error) && frame.data.empty(),
        "reordered acquisitions cannot enter the history-conditioned head");
    invalid = history; invalid.back() = invalid.front();
    Require(!p::MakeDefmHistoryTensorFrame(invalid,input,1000.31,&frame,&error),"duplicating a real frame is not valid history padding");
    invalid = history; invalid[2].reset();
    Require(!p::MakeDefmHistoryTensorFrame(invalid,input,1000.31,&frame,&error),"missing history cannot be filled with zero tokens");
    auto changed = std::make_shared<d::DefmDepthFeatures>(*history[1]); ++changed->calibration_revision;
    invalid = history; invalid[1] = changed;
    Require(!p::MakeDefmHistoryTensorFrame(invalid,input,1000.31,&frame,&error),"different camera calibration revokes history input");
    auto wrong = input; wrong.set_shape(2,384); wrong.set_shape(3,history.front()->patch_count);
    Require(!p::MakeDefmHistoryTensorFrame(history,wrong,1000.31,&frame,&error),"equal element counts cannot hide incorrect token axes");
    Require(!p::MakeDefmHistoryTensorFrame(history,input,1010.01,&frame,&error) && frame.data.empty(),
        "newer frames cannot renew expired oldest context");
}
}  // namespace

/** @brief Test requires an actual exported official model and its independent fixture. */
int main(int argc, char** argv) {
    try {
        Require(argc == 3 && *argv[1] && *argv[2], "configure AUTONOMY_DEFM_TEST_MODEL and AUTONOMY_DEFM_TEST_FIXTURE");
        std::ifstream file(argv[2]); Require(file.good(), "official fixture is missing");
        nlohmann::json fixture; file >> fixture;
        Check(argv[1], fixture);
        SharedHistory(argv[1], fixture);
        std::cout << "Real DeFM native inference and official preprocessing parity passed\n";
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << exception.what() << '\n'; return 1;
    }
}
