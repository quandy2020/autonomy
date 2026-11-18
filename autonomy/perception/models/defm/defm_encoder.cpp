/* Copyright 2026 The Openbot Authors (duyongquan). */
/** @file defm_encoder.cpp
 * @brief Native metric decoding and shared frozen DeFM backend execution.
 */
#include "autonomy/perception/models/defm/defm_encoder.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace autonomy::perception::defm {
namespace {
/** @brief Set a rejection reason without returning an old successful result. */
void Reject(std::string* error, const std::string& reason) {
    if (error) *error = "DeFM: " + reason;
}

/** @brief Exact static float32 names/shapes; dimension products alone are insufficient. */
bool Matches(const common::network::ModelTensorInfo& info, const std::string& name,
             const std::vector<std::int64_t>& shape) {
    return info.name == name && info.element_type == common::network::ElementType::kFloat32 &&
           info.shape.Dims() == shape;
}

/** @brief Check every exported output independent of metadata ordering. */
bool ValidOutputs(const std::vector<common::network::ModelTensorInfo>& outputs,
                  const DefmEncoderOptions& options) {
    if (outputs.size() != 3) return false;
    const int patches = (options.target_height / 14) * (options.target_width / 14);
    bool normalized = false, tokens = false, global = false;
    for (const auto& info : outputs) {
        normalized |= Matches(info, "normalized_depth", {1, 3, options.target_height, options.target_width});
        tokens |= Matches(info, "patch_tokens", {1, patches, options.feature_dimensions});
        global |= Matches(info, "global_feature", {1, options.feature_dimensions});
    }
    return normalized && tokens && global;
}

/** @brief Real outputs must have their expected element count and finite values. */
bool ValidTensor(const common::network::TensorMap& tensors, const char* name, std::size_t expected) {
    const auto it = tensors.find(name);
    if (it == tensors.end()) return false;
    const float* values = nullptr;
    std::size_t count = 0;
    return it->second.TryViewFloat32(&values, &count) && values && count == expected &&
           std::all_of(values, values + count, [](float value) { return std::isfinite(value); });
}
}  // namespace

DefmEncoder::DefmEncoder(DefmEncoderOptions options, std::unique_ptr<common::network::Engine> engine)
    : options_(std::move(options)), engine_(std::move(engine)) {}

std::shared_ptr<DefmEncoder> DefmEncoder::Create(DefmEncoderOptions options, std::string* error) {
    if (error) error->clear();
    try {
        ValidateSensorDecodingOptions(options.decoding);
        const auto& camera = options.calibration.camera;
        if (options.inference.model_path.empty() || !camera.Valid() ||
            options.calibration.frame_id.empty() || !options.calibration.revision ||
            !options.calibration.rectified || camera.height > 1080 || camera.width > 1920 ||
            options.target_height < 14 || options.target_width < 14 ||
            options.target_height > 448 || options.target_width > 448 ||
            options.target_height % 14 || options.target_width % 14 || options.feature_dimensions != 384 ||
            !std::isfinite(options.maximum_age) || options.maximum_age <= 0 || options.maximum_age > 10 ||
            !std::isfinite(options.minimum_valid_fraction) || options.minimum_valid_fraction <= 0 ||
            options.minimum_valid_fraction > 1 || !options.cache_capacity || options.cache_capacity > 8) {
            Reject(error, "invalid calibrated camera or frozen ViT-S/14 contract");
            return nullptr;
        }
        auto engine = common::network::Engine::CreateEngine(options.inference, error);
        if (!engine) return nullptr;
        const auto inputs = engine->GetInputInfos();
        if (inputs.size() != 1 || !Matches(inputs.front(), "depth_metric", {1, 1, camera.height, camera.width}) ||
            !ValidOutputs(engine->GetOutputInfos(), options)) {
            Reject(error, "model must contain official preprocessing and exact static depth/token I/O");
            return nullptr;
        }
        return std::shared_ptr<DefmEncoder>(new DefmEncoder(std::move(options), std::move(engine)));
    } catch (const std::exception& exception) {
        Reject(error, exception.what());
        return nullptr;
    }
}

std::shared_ptr<const DefmDepthFeatures> DefmEncoder::Encode(
    std::shared_ptr<const ::automsgs::msgs::sensor_msgs::Image> image,
    double now_sec, std::string* error) {
    if (error) error->clear();
    if (!image || !std::isfinite(now_sec) || now_sec <= 0) {
        Reject(error, "missing real depth observation or invalid caller clock");
        return nullptr;
    }
    const auto view = NativeDepthImage(*image);
    if (!view || view->width != static_cast<unsigned>(options_.calibration.camera.width) ||
        view->height != static_cast<unsigned>(options_.calibration.camera.height) ||
        view->frame_id != options_.calibration.frame_id || view->stamp > now_sec ||
        now_sec - view->stamp >= std::min(options_.maximum_age, options_.decoding.valid_for)) {
        Reject(error, "unmatched camera, future/stale depth, or malformed acquisition header");
        return nullptr;
    }
    const auto started = std::chrono::steady_clock::now();
    const auto current_time = [&] {
        return now_sec + std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    };
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_time() >= view->stamp + std::min(options_.maximum_age, options_.decoding.valid_for)) {
        Reject(error, "depth expired while waiting for shared inference"); return nullptr;
    }
    for (const auto& cached : cache_) {
        if (cached->image == image && current_time() < cached->valid_until) return cached;
    }
    try {
        const auto decoded = DecodeDepthImage(*view, options_.calibration, options_.decoding);
        if (!decoded) { Reject(error, "invalid calibrated depth byte layout"); return nullptr; }
        auto result = std::make_shared<DefmDepthFeatures>();
        result->image = image;
        result->calibration_revision = options_.calibration.revision;
        result->stamp = view->stamp;
        result->valid_until = view->stamp + std::min(options_.maximum_age, options_.decoding.valid_for);
        result->patch_count = (options_.target_height / 14) * (options_.target_width / 14);
        result->feature_dimensions = options_.feature_dimensions;
        result->valid_pixels.resize(decoded->depth.size());
        std::vector<float> metric(decoded->depth.size());
        std::size_t valid = 0;
        for (std::size_t i = 0; i < metric.size(); ++i) {
            // This unit conversion retains the original values. Sanitization,
            // clipping, metric channels and resize all belong to the official graph.
            metric[i] = static_cast<float>(decoded->depth[i] * decoded->depth_scale);
            const double depth = metric[i];
            result->valid_pixels[i] = std::isfinite(depth) && depth >= decoded->camera.min_depth &&
                                       depth <= decoded->camera.max_depth;
            valid += result->valid_pixels[i];
        }
        result->valid_fraction = static_cast<double>(valid) / metric.size();
        if (result->valid_fraction < options_.minimum_valid_fraction) {
            Reject(error, "insufficient measured valid depth; tokens cannot manufacture geometry");
            return nullptr;
        }
        common::network::TensorMap inputs;
        inputs.emplace("depth_metric", common::network::Tensor::FromFloat32(std::move(metric)));
        if (!engine_->Run(inputs, &result->tensors)) {
            Reject(error, engine_->GetLastError()); return nullptr;
        }
        if (result->tensors.size() != 3 ||
            !ValidTensor(result->tensors, "normalized_depth", 3U * options_.target_height * options_.target_width) ||
            !ValidTensor(result->tensors, "patch_tokens", static_cast<std::size_t>(result->patch_count) * result->feature_dimensions) ||
            !ValidTensor(result->tensors, "global_feature", result->feature_dimensions)) {
            Reject(error, "nonfinite or malformed actual model output"); return nullptr;
        }
        if (current_time() >= result->valid_until) {
            Reject(error, "depth expired during actual model inference"); return nullptr;
        }
        ++inference_count_;
        cache_.push_back(result);
        while (cache_.size() > options_.cache_capacity) cache_.pop_front();
        return result;
    } catch (const std::exception& exception) {
        Reject(error, exception.what()); return nullptr;
    }
}

void DefmEncoder::ClearCache() {
    std::lock_guard<std::mutex> lock(mutex_);
    cache_.clear();
}

std::size_t DefmEncoder::inference_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return inference_count_;
}
}  // namespace autonomy::perception::defm
