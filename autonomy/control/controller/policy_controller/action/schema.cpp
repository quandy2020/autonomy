/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

/**
 * @file schema.cpp
 * @brief ActionSchema::FromConfig implementation.
 */

#include "autonomy/control/controller/policy_controller/action/space.hpp"

#include <algorithm>

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {
namespace {

void ApplyTwistDefaults(ActionSchema* schema) {
    if (schema->tensor_name.empty()) {
        schema->tensor_name = "action";
    }
    // Default remains differential-drive 2DOF; declare a 6-slot layout for 6DOF.
    if (schema->slots.empty()) {
        schema->slots = {"linear_velocity_x", "angular_velocity_z"};
    }
    if (schema->action_dimension <= 0) {
        schema->action_dimension = static_cast<int>(schema->slots.size());
    }
    if (schema->type == proto::ACTION_SPACE_UNSPECIFIED) {
        schema->type = proto::ACTION_SPACE_TWIST;
    }
}

}  // namespace

ActionSchema ActionSchema::FromConfig(const proto::PolicyConfig& config,
                                      std::string* error) {
    ActionSchema schema;
    const auto& action = config.action();
    const bool use_unified_action =
        !action.name().empty() || action.layout_size() > 0 ||
        action.scale_size() > 0 || config.outputs_size() == 0;

    if (use_unified_action) {
        schema.tensor_name = action.name();
        schema.slots.assign(action.layout().begin(), action.layout().end());
        schema.scale.assign(action.scale().begin(), action.scale().end());
    } else {
        int index = action.output_index();
        if (index < 0 || index >= config.outputs_size()) {
            index = 0;
        }
        const auto& output = config.outputs(index);
        schema.tensor_name = output.name();
        schema.slots.assign(output.layout().begin(), output.layout().end());
        schema.scale.assign(output.scale().begin(), output.scale().end());
    }

    schema.type = action.type();
    if (config.has_diffusion()) {
        const auto& diffusion = config.diffusion();
        if (diffusion.horizon() > 0) {
            schema.action_horizon = diffusion.horizon();
        }
        if (diffusion.dim() > 0) {
            schema.action_dimension = diffusion.dim();
        }
        schema.execute_step_index = std::max(0, diffusion.execute_index());
        if ((schema.type == proto::ACTION_SPACE_TWIST ||
             schema.type == proto::ACTION_SPACE_UNSPECIFIED) &&
            (config.mode() == proto::INFERENCE_DIFFUSION ||
             diffusion.steps() > 1)) {
            schema.type = proto::ACTION_SPACE_DIFFUSION_TWIST;
        }
    }

    ApplyTwistDefaults(&schema);
    ResolveActionIndices(&schema);
    if (schema.tensor_name.empty() && error != nullptr) {
        *error = "Policy action name is empty";
    }
    return schema;
}

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
