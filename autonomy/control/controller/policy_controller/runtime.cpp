/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/control/controller/policy_controller/runtime.hpp"

#include <chrono>
#include <future>
#include <memory>
#include <utility>

#include "autonomy/control/controller/policy_controller/action/space.hpp"
#include "autonomy/control/controller/policy_controller/diffusion.hpp"
#include "autonomy/control/controller/policy_controller/observation/encoder.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {
namespace {

bool RunSingleShotForward(const PolicyModel& model,
                          PolicyRuntimeBuffers* buffers,
                          const ActionSchema& schema,
                          automsgs::msgs::geometry_msgs::Twist* action,
                          std::string* error) {
    buffers->outputs.clear();
    if (!model.RunInference(buffers->inputs, &buffers->outputs, error)) {
        return false;
    }
    const auto it = buffers->outputs.find(schema.tensor_name);
    if (it == buffers->outputs.end()) {
        if (error != nullptr) {
            *error = "Missing action output tensor: " + schema.tensor_name;
        }
        return false;
    }
    return TwistActionSpace::Decode(it->second, schema, action, error);
}

}  // namespace

void InferWatchdog::ReapIfReady() {
    if (!pending.valid()) {
        return;
    }
    if (pending.wait_for(std::chrono::seconds(0)) !=
        std::future_status::ready) {
        return;
    }
    pending.get();
    model.reset();
    buffers.reset();
    action.reset();
    error.reset();
}

bool InferWatchdog::Busy() const {
    return pending.valid() &&
           pending.wait_for(std::chrono::seconds(0)) !=
               std::future_status::ready;
}

void InferWatchdog::Reset() {
    if (pending.valid()) {
        pending.wait();
        pending.get();
    }
    model.reset();
    buffers.reset();
    action.reset();
    error.reset();
}

bool PredictTwist(PolicyModel::ConstSharedPointer model,
                  const ObservationSnapshot& snapshot,
                  const ObservationEncodeContext& encode_context,
                  PolicyRuntimeBuffers* buffers,
                  automsgs::msgs::geometry_msgs::Twist* action,
                  std::string* error, bool* zero_filled, InferBudget* budget,
                  InferWatchdog* watchdog) {
    if (!model || buffers == nullptr || action == nullptr) {
        if (error != nullptr && !model) {
            *error = "Null PolicyModel";
        }
        return false;
    }
    if (watchdog != nullptr) {
        watchdog->ReapIfReady();
        if (watchdog->Busy()) {
            if (error != nullptr) {
                *error = "Inference timeout (previous forward still running)";
            }
            return false;
        }
    }
    if (budget != nullptr && budget->Exceeded()) {
        if (error != nullptr) {
            *error = "Inference timeout before encode";
        }
        return false;
    }

    const auto encode_start = std::chrono::steady_clock::now();
    buffers->inputs.clear();
    if (!EncodeObservations(snapshot, model->config(), encode_context,
                            &buffers->inputs, error, zero_filled)) {
        if (budget != nullptr) {
            budget->encode_elapsed_ms =
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - encode_start)
                    .count();
        }
        return false;
    }
    if (budget != nullptr) {
        budget->encode_elapsed_ms = std::chrono::duration<double, std::milli>(
                                        std::chrono::steady_clock::now() -
                                        encode_start)
                                        .count();
    }
    if (budget != nullptr && budget->Exceeded()) {
        if (error != nullptr) {
            *error = "Inference timeout after encode";
        }
        return false;
    }

    const ActionSchema& schema = model->config().action_schema();
    action->Clear();
    const auto infer_start = std::chrono::steady_clock::now();

    if (model->config().is_diffusion()) {
        buffers->action_sequence.clear();
        if (!RunDiffusionInference(*model, buffers->inputs,
                                   &buffers->action_sequence, error, budget)) {
            if (budget != nullptr) {
                budget->infer_elapsed_ms =
                    std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - infer_start)
                        .count();
            }
            return false;
        }
        if (budget != nullptr) {
            budget->infer_elapsed_ms =
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - infer_start)
                    .count();
        }
        return DiffusionTwistActionSpace::Decode(buffers->action_sequence,
                                                 schema, action, error);
    }

    if (budget != nullptr && budget->Enabled() && watchdog != nullptr) {
        const double remaining = budget->RemainingSec();
        if (remaining <= 0.0) {
            if (error != nullptr) {
                *error = "Inference timeout before forward";
            }
            budget->infer_elapsed_ms = 0.0;
            return false;
        }

        watchdog->model = model;
        watchdog->buffers = std::make_shared<PolicyRuntimeBuffers>();
        watchdog->buffers->inputs = buffers->inputs;
        watchdog->action =
            std::make_shared<automsgs::msgs::geometry_msgs::Twist>();
        watchdog->error = std::make_shared<std::string>();
        auto job_model = watchdog->model;
        auto job_buffers = watchdog->buffers;
        auto job_action = watchdog->action;
        auto job_error = watchdog->error;
        // Capture shared ownership (not stack references) so an abandoned
        // forward survives hot-swap / tick return.
        watchdog->pending = std::async(
            std::launch::async,
            [job_model, job_buffers, job_action, job_error]() {
                const ActionSchema& job_schema =
                    job_model->config().action_schema();
                return RunSingleShotForward(*job_model, job_buffers.get(),
                                            job_schema, job_action.get(),
                                            job_error.get());
            });

        if (watchdog->pending.wait_for(
                std::chrono::duration<double>(remaining)) !=
            std::future_status::ready) {
            if (error != nullptr) {
                *error = "Inference timeout during forward";
            }
            budget->infer_elapsed_ms =
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - infer_start)
                    .count();
            return false;
        }

        const bool ok = watchdog->pending.get();
        budget->infer_elapsed_ms = std::chrono::duration<double, std::milli>(
                                       std::chrono::steady_clock::now() -
                                       infer_start)
                                       .count();
        if (!ok) {
            if (error != nullptr) {
                *error = *job_error;
            }
            watchdog->Reset();
            return false;
        }
        buffers->outputs = std::move(job_buffers->outputs);
        *action = *job_action;
        watchdog->model.reset();
        watchdog->buffers.reset();
        watchdog->action.reset();
        watchdog->error.reset();
        return true;
    }

    const bool ok =
        RunSingleShotForward(*model, buffers, schema, action, error);
    if (budget != nullptr) {
        budget->infer_elapsed_ms = std::chrono::duration<double, std::milli>(
                                       std::chrono::steady_clock::now() -
                                       infer_start)
                                       .count();
        if (ok && budget->Exceeded()) {
            if (error != nullptr) {
                *error = "Inference timeout after forward";
            }
            return false;
        }
    }
    return ok;
}

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
