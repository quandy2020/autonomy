/* Copyright 2026 The Openbot Authors (duyongquan). */
/** @file defm_features.cpp
 * @brief Exact token shapes and original acquisition leases for native policies.
 */
#include "autonomy/control/controller/policy_controller/observation/defm_features.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

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

bool MakeDefmHistoryTensorFrame(
    const std::vector<std::shared_ptr<const perception::defm::DefmDepthFeatures>>& history,
    const proto::ObservationInput& input, double now_sec,
    TensorFrame* output, std::string* error) {
    if (error) error->clear();
    if (!output) return Reject(error, "null history TensorFrame output");
    *output = {};
    if (history.empty() || history.size() > 8 || !history.front() ||
        input.modality() != proto::OBSERVATION_RAW || input.name().empty() || !input.required() ||
        input.norm().ByteSizeLong() || input.shape_size() != 4 || input.shape(0) != 1 ||
        input.shape(1) != static_cast<std::int64_t>(history.size()) ||
        input.shape(2) != history.front()->patch_count || input.shape(3) != history.front()->feature_dimensions)
        return Reject(error, "history requires exact mandatory [1,T,N,D] RAW axes");
    const auto& first = *history.front();
    if (!first.image || first.patch_count <= 0 || first.patch_count > 1024 || first.feature_dimensions != 384)
        return Reject(error, "history has no valid original camera/token contract");
    proto::ObservationInput one;
    one.set_name(input.name()); one.set_modality(proto::OBSERVATION_RAW); one.set_required(true);
    one.add_shape(1); one.add_shape(first.patch_count); one.add_shape(first.feature_dimensions);
    TensorFrame staged;  ///< Complete output becomes visible only after all original acquisitions pass.
    staged.name = input.name(); staged.shape = {1, static_cast<std::int64_t>(history.size()), first.patch_count, first.feature_dimensions};
    staged.valid_until_sec = std::numeric_limits<double>::infinity();
    staged.data.reserve(history.size()*static_cast<std::size_t>(first.patch_count)*first.feature_dimensions);
    double previous_stamp = 0;  ///< Strict chronological identity; no temporal reorder or duplicate padding.
    for (const auto& features : history) {
        if (!features || !features->image || features->calibration_revision != first.calibration_revision ||
            features->image->header().frame_id() != first.image->header().frame_id() ||
            features->image->width() != first.image->width() || features->image->height() != first.image->height() ||
            features->patch_count != first.patch_count || features->feature_dimensions != first.feature_dimensions ||
            features->stamp <= previous_stamp)
            return Reject(error, "history is missing, reordered or has incompatible real acquisitions");
        TensorFrame frame;
        if (!MakeDefmTensorFrame(*features, one, "patch_tokens", now_sec, &frame, error)) return false;
        staged.data.insert(staged.data.end(), frame.data.begin(), frame.data.end());
        staged.valid_until_sec = std::min(staged.valid_until_sec, frame.valid_until_sec);
        staged.stamp_sec = frame.stamp_sec; previous_stamp = frame.stamp_sec;
    }
    *output = std::move(staged);
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

