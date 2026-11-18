/* Copyright 2026 The Openbot Authors (duyongquan). */
/**
 * @file defm_encoder.hpp
 * @brief Frozen DeFM inference over actual stamped depth messages and shared tokens.
 */
#pragma once

#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "autonomy/common/network/backend/engine.hpp"
#include "autonomy/perception/observation/native_sensor_messages.hpp"

namespace autonomy::perception::defm {

/** @brief Static camera and exported frozen-model contract, with bounded work. */
struct DefmEncoderOptions {
    common::network::InferenceOptions inference;  ///< Existing ONNX/TensorRT backend and real model path.
    DepthCalibration calibration;                ///< Actual rectified depth camera and immutable revision.
    SensorDecodingOptions decoding;              ///< Existing byte-layout, metric-unit and allocation policy.
    int target_height = 224;                     ///< Exported normalized raster height; divisible by 14.
    int target_width = 224;                      ///< Exported normalized raster width; divisible by 14.
    int feature_dimensions = 384;                ///< Official ViT-S/14 normalized token dimension.
    double maximum_age = .2;                     ///< Acquisition validity in caller-clock seconds; never renewed.
    double minimum_valid_fraction = .1;          ///< Required measured in-range depth fraction, not inferred free space.
    std::size_t cache_capacity = 4;              ///< Shared immutable frames retained for decision/control consumers.
};

/**
 * @brief Immutable, process-local output using existing messages and tensor types.
 *
 * This is not a new wire message. The original Image owns frame/time identity;
 * tokens do not constitute localization, occupancy, clearance or coverage evidence.
 */
struct DefmDepthFeatures {
    std::shared_ptr<const ::automsgs::msgs::sensor_msgs::Image> image;  ///< Original immutable native depth observation.
    std::uint64_t calibration_revision = 0;  ///< Calibration used to decode this exact raster.
    double stamp = 0, valid_until = 0;        ///< Original acquisition time and absolute expiration, seconds.
    double valid_fraction = 0;              ///< Fraction of original pixels with measured in-range metric depth.
    int patch_count = 0;                    ///< Number of spatial tokens in row-major patch-grid order.
    int feature_dimensions = 0;             ///< Channels of each patch/global token.
    std::vector<std::uint8_t> valid_pixels;   ///< Original-resolution validity; missing pixels remain unknown.
    common::network::TensorMap tensors;      ///< normalized_depth, patch_tokens and global_feature; float32.
};

/**
 * @brief One serialized frozen backend shared by decision and local control.
 *
 * The exported graph calls official preprocess_depth_batch before the backbone.
 * C++ decodes units only; it does not invent another DeFM normalization/resize.
 * Repeated consumers of the same immutable Image share the cached result. They
 * must still validate time, pose and alignment in their own decision loops.
 */
class DefmEncoder {
public:
    /** @brief Load an actual exported graph and check its exact static I/O.
     * @param options Real calibrated camera, backend and exported raster contract.
     * @param error Explicit configuration/backend/shape rejection, if requested.
     * @return Null on failure; a backend without weights is never substituted.
     */
    static std::shared_ptr<DefmEncoder> Create(DefmEncoderOptions options, std::string* error = nullptr);

    /** @brief Decode and encode one real depth frame, or reuse its immutable tokens.
     * @param image Actual native 16UC1/32FC1 observation; must remain immutable.
     * @param now_sec Current acquisition-compatible clock time, seconds.
     * @param error Explicit layout, validity or inference failure.
     * @return Shared const features, or null for invalid/stale/unknown-only depth.
     */
    std::shared_ptr<const DefmDepthFeatures> Encode(
        std::shared_ptr<const ::automsgs::msgs::sensor_msgs::Image> image,
        double now_sec, std::string* error = nullptr);

    /** @brief Drop cached frames without changing model weights or calibration. */
    void ClearCache();
    /** @brief Successful actual backend invocations; cached reads do not increase it. */
    std::size_t inference_count() const;

private:
    /** @brief Private ownership of one already validated inference backend. */
    DefmEncoder(DefmEncoderOptions options, std::unique_ptr<common::network::Engine> engine);
    DefmEncoderOptions options_;                     ///< Immutable camera/model configuration after Create.
    std::unique_ptr<common::network::Engine> engine_;  ///< Existing inference engine; no training path.
    mutable std::mutex mutex_;                       ///< Serializes backend access and bounded frame cache.
    std::deque<std::shared_ptr<const DefmDepthFeatures>> cache_;  ///< Oldest-first immutable feature cache.
    std::size_t inference_count_ = 0;                 ///< Successfully validated real inference outputs.
};

}  // namespace autonomy::perception::defm
