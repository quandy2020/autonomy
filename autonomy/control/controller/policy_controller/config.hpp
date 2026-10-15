/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

/**
 * @file config.hpp
 * @brief Parsed PolicyConfig sidecar (observation / action / engine).
 */

#pragma once

#include <string>
#include <vector>

#include "autonomy/common/network/common/options.hpp"
#include "autonomy/common/network/common/tensor.hpp"
#include "autonomy/control/controller/policy_controller/action/space.hpp"
#include "autonomy/control/proto/policy_controller.pb.h"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {

/**
 * @class PolicyConfig
 * @brief Validated policy configuration loaded from textproto or proto.
 *
 * On load:
 * - Expands @c observation { kinematic: true ... } into internal @c inputs[]
 * - Builds @ref ActionSchema (slot indices resolved once)
 * - Caches diffusion flags / effective diffusion options
 */
class PolicyConfig
{
public:
    PolicyConfig() = default;

    /**
     * @brief Parse a PolicyConfig textproto or binary file.
     * @param[in] path File path.
     * @param[out] out Destination config.
     * @param[out] error Optional failure reason.
     * @return true on success.
     */
    static bool LoadFromFile(const std::string& path, PolicyConfig* out,
                             std::string* error = nullptr);

    /**
     * @brief Validate and normalize an in-memory proto.
     * @param[in] proto Raw sidecar message.
     * @param[out] out Destination config.
     * @param[out] error Optional failure reason.
     * @return true on success.
     */
    static bool LoadFromProto(const proto::PolicyConfig& proto,
                              PolicyConfig* out,
                              std::string* error = nullptr);

    /**
     * @brief Underlying proto (inputs expanded if observation was used).
     */
    const proto::PolicyConfig& proto() const { return proto_; }
    /**
     * @brief Resolved action layout / indices for decode.
     */
    const ActionSchema& action_schema() const { return action_schema_; }
    /**
     * @brief Cached: whether this policy uses diffusion inference.
     */
    bool is_diffusion() const { return is_diffusion_; }
    /**
     * @brief Cached diffusion options with defaults filled in.
     */
    const proto::Diffusion& effective_diffusion() const {
        return effective_diffusion_;
    }

    /**
     * @brief Map engine fields to @c ::autonomy::common::network::InferenceOptions.
     */
    ::autonomy::common::network::InferenceOptions ToInferenceOptions() const;

    /**
     * @brief Hard-check required I/O names / shapes against a loaded engine.
     * @return false if a required tensor is missing or shape-incompatible.
     */
    bool ValidateAgainstEngine(
        const std::vector<::autonomy::common::network::ModelTensorInfo>& inputs,
        const std::vector<::autonomy::common::network::ModelTensorInfo>& outputs,
        std::string* error = nullptr) const;

private:
    proto::PolicyConfig proto_;
    ActionSchema action_schema_;
    bool is_diffusion_{false};
    proto::Diffusion effective_diffusion_;
};

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
