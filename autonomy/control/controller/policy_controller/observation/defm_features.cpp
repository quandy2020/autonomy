/* Copyright 2026 The Openbot Authors (duyongquan). */
/** @file defm_features.cpp
 * @brief Exact token shapes and original acquisition leases for native policies.
 */
#include "autonomy/control/controller/policy_controller/observation/defm_features.hpp"
#include <algorithm>
#include <cmath>

namespace autonomy::control::controller::policy_controller {
namespace {
/** @brief Uniform rejection; never renew a feature acquisition lease. */
bool Reject(std::string* error, const std::string& reason) {
    if (error) *error = "DeFM policy input: " + reason;
    return false;
}
}  // namespace

bool MakeDefmTensorFrame(const perception::defm::DefmDepthFeatures& features,
                        const proto::ObservationInput& input,
                        const std::string& tensor_name, double now_sec,
                        TensorFrame* output, std::string* error) {
    if (error) error->clear();
    if (!output) return Reject(error, "null TensorFrame output");
    *output = {};
    if (!features.image || !features.calibration_revision || features.patch_count <= 0 ||
        features.patch_count > 1024 || features.feature_dimensions != 384 ||
        !std::isfinite(features.valid_fraction) || features.valid_fraction <= 0.0 ||
        features.valid_fraction > 1.0 || !std::isfinite(features.stamp) || features.stamp <= 0.0 ||
        HeaderStampSeconds(features.image->header()) != features.stamp ||
        features.image->header().frame_id().empty() ||
        features.image->header().stamp().nanosec() >= 1000000000 ||
        !std::isfinite(now_sec) || now_sec < features.stamp ||
        !std::isfinite(features.valid_until) || features.valid_until <= features.stamp ||
        features.valid_until - features.stamp > 10.0 || now_sec >= features.valid_until) {
        return Reject(error, "invalid, expired or unmatched real acquisition");
    }
    std::vector<int64_t> shape;
    if (tensor_name == "patch_tokens") shape = {1, features.patch_count, features.feature_dimensions};
    else if (tensor_name == "global_feature") shape = {1, features.feature_dimensions};
    else return Reject(error, "unsupported official feature output");
    if (input.modality() != proto::OBSERVATION_RAW || input.name().empty() ||
        !input.required() || input.norm().ByteSizeLong() != 0 ||
        input.shape_size() != static_cast<int>(shape.size())) {
        return Reject(error, "requires a mandatory static RAW input with no second normalization");
    }
    for (int i = 0; i < input.shape_size(); ++i)
        if (input.shape(i) != shape[static_cast<std::size_t>(i)])
            return Reject(error, "active model axes do not match official tokens");
    const auto it = features.tensors.find(tensor_name);
    const float* values = nullptr;
    std::size_t count = 0;
    const std::size_t expected = tensor_name == "patch_tokens" ?
        static_cast<std::size_t>(features.patch_count) * features.feature_dimensions :
        static_cast<std::size_t>(features.feature_dimensions);
    if (it == features.tensors.end() || !it->second.TryViewFloat32(&values, &count) ||
        !values || count != expected ||
        !std::all_of(values, values + count, [](float value) { return std::isfinite(value); })) {
        return Reject(error, "missing, malformed or nonfinite actual features");
    }
    output->name = input.name();
    output->shape = std::move(shape);
    output->data.assign(values, values + count);
    output->stamp_sec = features.stamp;
    output->valid_until_sec = features.valid_until;
    return true;
}

bool ValidateRawTensorLifetimes(const ObservationSnapshot& snapshot,
                               const PolicyConfig& config, double now_sec,
                               std::string* error) {
    for (const auto& input : config.proto().inputs()) {
        if (input.modality() != proto::OBSERVATION_RAW) continue;
        const auto it = snapshot.raw_tensors.find(input.name());
        if (it == snapshot.raw_tensors.end() || !it->second ||
            it->second->valid_until_sec == 0.0) continue;
        const auto& frame = *it->second;
        if (!std::isfinite(now_sec) || !std::isfinite(frame.stamp_sec) || frame.stamp_sec <= 0.0 ||
            !std::isfinite(frame.valid_until_sec) || frame.valid_until_sec <= frame.stamp_sec ||
            now_sec < frame.stamp_sec || now_sec >= frame.valid_until_sec)
            return Reject(error, "RAW acquisition expired or is in the future: " + input.name());
    }
    return true;
}
}  // namespace autonomy::control::controller::policy_controller

