/* Copyright 2026 The Openbot Authors (duyongquan). */
/**
 * @file defm_features.hpp
 * @brief Strict injection of shared frozen depth features into existing RAW inputs.
 */
#pragma once
#include <string>
#include "autonomy/control/controller/policy_controller/config.hpp"
#include "autonomy/control/controller/policy_controller/types.hpp"
#include "autonomy/perception/models/defm/defm_encoder.hpp"

namespace autonomy::control::controller::policy_controller {
/**
 * @brief Convert one real shared feature result to the existing TensorFrame type.
 * @param features Exact immutable depth encoder result with original acquisition.
 * @param input Active model's explicitly declared static RAW input.
 * @param tensor_name Official output to forward: patch_tokens or global_feature.
 * @param now_sec Current time in the original acquisition clock, seconds.
 * @param output Existing process-local tensor; cleared on rejection.
 * @param error Optional shape, time or model-contract rejection.
 * @return True only for finite features with the exact declared model axes.
 *
 * There is no padding, learned projection, invented depth image or new wire
 * message. A model trained for ResNet image features must be adapted separately.
 */
bool MakeDefmTensorFrame(const perception::defm::DefmDepthFeatures& features,
                        const proto::ObservationInput& input,
                        const std::string& tensor_name, double now_sec,
                        TensorFrame* output, std::string* error = nullptr);

/**
 * @brief Recheck finite RAW acquisition leases after inference and tracking.
 * @param snapshot Exact observation snapshot consumed by this control tick.
 * @param config Active model configuration, identifying the RAW consumers.
 * @param now_sec Current acquisition-compatible time, including time spent solving.
 * @param error Optional explicit expired/future-frame reason.
 * @return False when a consumed finite lease has expired or is malformed.
 */
bool ValidateRawTensorLifetimes(const ObservationSnapshot& snapshot,
                               const PolicyConfig& config, double now_sec,
                               std::string* error = nullptr);
}  // namespace autonomy::control::controller::policy_controller

