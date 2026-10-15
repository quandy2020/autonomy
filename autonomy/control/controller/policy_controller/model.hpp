/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

/**
 * @file model.hpp
 * @brief Loaded inference engine bound to a PolicyConfig.
 */

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "autonomy/common/network/backend/engine.hpp"
#include "autonomy/common/network/common/tensor.hpp"
#include "autonomy/control/controller/policy_controller/config.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {

/**
 * @class PolicyModel
 * @brief One exported policy graph (ONNX / TensorRT) plus its config.
 *
 * @c RunInference is guarded by @c inference_mutex_ and is not safe for
 * concurrent calls. @ref PolicyManager keeps a @c shared_ptr alive for the
 * duration of a control-thread tick so hot-swap cannot free the engine mid-run.
 */
class PolicyModel
{
public:
    using SharedPointer = std::shared_ptr<PolicyModel>;
    using ConstSharedPointer = std::shared_ptr<const PolicyModel>;

    /**
     * @brief Create engine, warmup, and validate I/O against @p config.
     * @return nullptr on failure (@p error filled when provided).
     */
    static SharedPointer Create(PolicyConfig config, std::string* error);

    /**
     * @brief Bound configuration (includes cached diffusion options).
     */
    const PolicyConfig& config() const { return config_; }
    const std::string& name() const { return config_.proto().name(); }
    const std::string& version() const {
        return config_.proto().version();
    }

    /**
     * @brief Run one forward pass.
     * @param[in] inputs Named float tensors (observation / diffusion slots).
     * @param[out] outputs Named float tensors from the engine.
     * @param[out] error Optional failure reason.
     * @return true on success.
     */
    bool RunInference(const ::autonomy::common::network::FloatTensorMap& inputs,
                      ::autonomy::common::network::FloatTensorMap* outputs,
                      std::string* error) const;

private:
    PolicyModel(PolicyConfig config,
                std::unique_ptr<::autonomy::common::network::Engine> engine);

    PolicyConfig config_;
    std::unique_ptr<::autonomy::common::network::Engine> engine_;
    mutable std::mutex inference_mutex_;
};

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
