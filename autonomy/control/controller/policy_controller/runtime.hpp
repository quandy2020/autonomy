/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

/**
 * @file runtime.hpp
 * @brief Encode → infer → decode without ControllerServer wiring.
 */

#pragma once

#include <future>
#include <memory>
#include <string>
#include <vector>

#include <automsgs/msgs/geometry_msgs/twist.pb.h>

#include "autonomy/common/network/common/tensor.hpp"
#include "autonomy/control/controller/policy_controller/model.hpp"
#include "autonomy/control/controller/policy_controller/types.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {

/**
 * @brief Reusable hot-path buffers for @ref PredictTwist.
 */
struct PolicyRuntimeBuffers {
    common::network::FloatTensorMap inputs;
    common::network::FloatTensorMap outputs;
    std::vector<float> action_sequence;
};

/**
 * @brief Tracks at most one abandoned async forward after infer_timeout.
 *
 * Keeps the model + heap buffers alive until ORT finishes so hot-path buffers
 * are never raced. Next ticks that see @c Busy() force zero until reaped.
 */
struct InferWatchdog {
    std::future<bool> pending;
    PolicyModel::SharedPointer model;
    std::shared_ptr<PolicyRuntimeBuffers> buffers;
    std::shared_ptr<automsgs::msgs::geometry_msgs::Twist> action;
    std::shared_ptr<std::string> error;

    void ReapIfReady();
    bool Busy() const;
    void Reset();
};

/**
 * @brief Run one policy inference tick: encode → infer → decode Twist.
 *
 * Pass a non-null @p watchdog with @c budget.Enabled() to enforce
 * single-shot forward via @c wait_for (abandoned late results stay in the
 * watchdog). Diffusion still uses step-level soft budget checks.
 *
 * @return true on success.
 */
bool PredictTwist(PolicyModel::SharedPointer model,
                  const ObservationSnapshot& snapshot,
                  const ObservationEncodeContext& encode_context,
                  PolicyRuntimeBuffers* buffers,
                  automsgs::msgs::geometry_msgs::Twist* action,
                  std::string* error = nullptr, bool* zero_filled = nullptr,
                  InferBudget* budget = nullptr,
                  InferWatchdog* watchdog = nullptr);

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
