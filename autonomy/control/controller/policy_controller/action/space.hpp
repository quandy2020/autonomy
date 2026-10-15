/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file space.hpp
 * @brief Action schema and ActionSpace traits (network output → Twist).
 *
 * TwistActionSpace / DiffusionTwistActionSpace support 2DOF through full 6DOF
 * geometry_msgs/Twist depending on the declared layout.
 */

#pragma once

#include <algorithm>
#include <string>
#include <vector>

#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>

#include "autonomy/control/proto/policy_controller.pb.h"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {

/**
 * @struct ActionSchema
 * @brief Resolved action head layout used by Decode on the hot path.
 *
 * Slot name → index mapping is computed once in @ref FromConfig via
 * @ref ResolveActionIndices so Decode never builds string maps.
 */
struct ActionSchema {
    /**
     * @brief Engine output tensor name.
     */
    std::string tensor_name;
    /**
     * @brief Named layout in element order.
     */
    std::vector<std::string> slots;
    /**
     * @brief Per-slot scale (default 1.0).
     */
    std::vector<float> scale;
    /**
     * @brief Action space type (Twist / DiffusionTwist).
     */
    proto::ActionSpace type{proto::ACTION_SPACE_TWIST};
    /**
     * @brief Diffusion sequence length.
     */
    int action_horizon{1};
    /**
     * @brief Per-step action size (0 → slots.size).
     */
    int action_dimension{0};
    /**
     * @brief Horizon index executed as Twist.
     */
    int execute_step_index{0};

    /**
     * @brief Index of linear.x slot, or -1.
     */
    int linear_x_index{-1};
    /**
     * @brief Index of linear.y slot, or -1.
     */
    int linear_y_index{-1};
    /**
     * @brief Index of linear.z slot, or -1.
     */
    int linear_z_index{-1};
    /**
     * @brief Index of angular.x slot, or -1.
     */
    int angular_x_index{-1};
    /**
     * @brief Index of angular.y slot, or -1.
     */
    int angular_y_index{-1};
    /**
     * @brief Index of angular.z slot, or -1.
     */
    int angular_z_index{-1};

    /**
     * @brief Build schema from a policy config proto (unified action or outputs[]).
     * @param[out] error Set when the action tensor name cannot be resolved.
     */
    static ActionSchema FromConfig(const proto::PolicyConfig& config,
                                   std::string* error = nullptr);

    /**
     * @brief Flattened diffusion sequence length = horizon * dimension.
     */
    std::size_t SequenceSize() const {
        const int dimension =
            action_dimension > 0 ? action_dimension
                                 : static_cast<int>(slots.size());
        return static_cast<std::size_t>(std::max(1, action_horizon)) *
               static_cast<std::size_t>(std::max(1, dimension));
    }
};

/**
 * @brief Per-slot scale with 1.0 default when @p scale is short.
 */
inline float ScaleAt(const std::vector<float>& scale, std::size_t index) {
    return index < scale.size() ? scale[index] : 1.0f;
}

/**
 * @brief True if @p slot matches any of @p aliases.
 */
inline bool SlotNameIn(const std::string& slot,
                       std::initializer_list<const char*> aliases) {
    for (const char* name : aliases) {
        if (slot == name) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Resolve Twist 6DOF indices from @c slots.
 */
inline void ResolveActionIndices(ActionSchema* schema) {
    if (schema == nullptr) {
        return;
    }
    schema->linear_x_index = -1;
    schema->linear_y_index = -1;
    schema->linear_z_index = -1;
    schema->angular_x_index = -1;
    schema->angular_y_index = -1;
    schema->angular_z_index = -1;
    for (std::size_t i = 0; i < schema->slots.size(); ++i) {
        const auto& slot = schema->slots[i];
        const int index = static_cast<int>(i);
        if (schema->linear_x_index < 0 &&
            SlotNameIn(slot, {"linear_velocity_x", "linear_x", "vx", "v"})) {
            schema->linear_x_index = index;
        } else if (schema->linear_y_index < 0 &&
                   SlotNameIn(slot, {"linear_velocity_y", "linear_y", "vy"})) {
            schema->linear_y_index = index;
        } else if (schema->linear_z_index < 0 &&
                   SlotNameIn(slot, {"linear_velocity_z", "linear_z", "vz"})) {
            schema->linear_z_index = index;
        } else if (schema->angular_x_index < 0 &&
                   SlotNameIn(slot, {"angular_velocity_x", "angular_x", "wx",
                                     "roll_rate"})) {
            schema->angular_x_index = index;
        } else if (schema->angular_y_index < 0 &&
                   SlotNameIn(slot, {"angular_velocity_y", "angular_y", "wy",
                                     "pitch_rate"})) {
            schema->angular_y_index = index;
        } else if (schema->angular_z_index < 0 &&
                   SlotNameIn(slot, {"angular_velocity_z", "angular_z", "wz",
                                     "w", "omega", "yaw_rate"})) {
            schema->angular_z_index = index;
        }
    }
}

/**
 * @brief Highest resolved Twist index, or -1 if none.
 */
inline int MaxResolvedActionIndex(const ActionSchema& schema) {
    int max_index = -1;
    const int indices[] = {
        schema.linear_x_index,  schema.linear_y_index,  schema.linear_z_index,
        schema.angular_x_index, schema.angular_y_index, schema.angular_z_index};
    for (const int index : indices) {
        max_index = std::max(max_index, index);
    }
    return max_index;
}

/**
 * @brief Read one scaled slot at @p index + @p offset.
 */
inline bool ReadScaledSlotAt(const std::vector<float>& raw,
                             const ActionSchema& schema, int index,
                             std::size_t offset, double* value) {
    if (value == nullptr || index < 0) {
        return false;
    }
    const std::size_t absolute = offset + static_cast<std::size_t>(index);
    if (raw.size() <= absolute) {
        return false;
    }
    *value = static_cast<double>(
        raw[absolute] * ScaleAt(schema.scale, static_cast<std::size_t>(index)));
    return true;
}

/**
 * @brief Write all resolved Twist components from @p raw into @p twist.
 * @param offset Element offset into @p raw (diffusion horizon step).
 * @return false if no Twist slot is present or @p raw is too short.
 */
inline bool DecodeTwistComponents(const std::vector<float>& raw,
                                  const ActionSchema& schema,
                                  std::size_t offset,
                                  automsgs::msgs::geometry_msgs::Twist* twist,
                                  std::string* error) {
    if (twist == nullptr) {
        return false;
    }
    const int max_index = MaxResolvedActionIndex(schema);
    if (max_index < 0 ||
        (schema.linear_x_index < 0 && schema.linear_y_index < 0 &&
         schema.linear_z_index < 0 && schema.angular_x_index < 0 &&
         schema.angular_y_index < 0 && schema.angular_z_index < 0)) {
        if (error != nullptr) {
            *error =
                "Twist action space requires at least one of "
                "linear_velocity_{x,y,z} / angular_velocity_{x,y,z}";
        }
        return false;
    }
    if (raw.size() < offset + static_cast<std::size_t>(max_index + 1)) {
        if (error != nullptr) {
            *error = "Action vector shorter than schema slots";
        }
        return false;
    }

    twist->Clear();
    double value = 0.0;
    if (ReadScaledSlotAt(raw, schema, schema.linear_x_index, offset, &value)) {
        twist->mutable_linear()->set_x(value);
    }
    if (ReadScaledSlotAt(raw, schema, schema.linear_y_index, offset, &value)) {
        twist->mutable_linear()->set_y(value);
    }
    if (ReadScaledSlotAt(raw, schema, schema.linear_z_index, offset, &value)) {
        twist->mutable_linear()->set_z(value);
    }
    if (ReadScaledSlotAt(raw, schema, schema.angular_x_index, offset, &value)) {
        twist->mutable_angular()->set_x(value);
    }
    if (ReadScaledSlotAt(raw, schema, schema.angular_y_index, offset, &value)) {
        twist->mutable_angular()->set_y(value);
    }
    if (ReadScaledSlotAt(raw, schema, schema.angular_z_index, offset, &value)) {
        twist->mutable_angular()->set_z(value);
    }
    return true;
}

/**
 * @brief Copy Twist into a TwistStamped command (header left to caller).
 */
inline void WriteTwistStamped(
    const automsgs::msgs::geometry_msgs::Twist& twist,
    automsgs::msgs::geometry_msgs::TwistStamped* command_velocity) {
    if (command_velocity == nullptr) {
        return;
    }
    *command_velocity->mutable_twist() = twist;
}

/**
 * @struct TwistActionSpace
 * @brief geometry_msgs/Twist action: 2DOF (lx, az) up to full 6DOF.
 *
 * Default layout is differential-drive `[linear_velocity_x, angular_velocity_z]`.
 * Declare additional layout slots for omnidirectional / aerial 6DOF.
 */
struct TwistActionSpace {
    static constexpr proto::ActionSpace kType = proto::ACTION_SPACE_TWIST;

    /**
     * @brief Decode a flat action vector using pre-resolved schema indices.
     * @return false if no Twist slot is present or @p raw is too short.
     */
    static bool Decode(const std::vector<float>& raw, const ActionSchema& schema,
                       automsgs::msgs::geometry_msgs::Twist* twist,
                       std::string* error) {
        return DecodeTwistComponents(raw, schema, /*offset=*/0, twist, error);
    }
};

/**
 * @struct DiffusionTwistActionSpace
 * @brief Pick Twist (2–6 DOF) from a flattened [horizon * dim] sequence.
 */
struct DiffusionTwistActionSpace {
    static constexpr proto::ActionSpace kType =
        proto::ACTION_SPACE_DIFFUSION_TWIST;

    /**
     * @brief Decode the step at @c schema.execute_step_index.
     */
    static bool Decode(const std::vector<float>& raw, const ActionSchema& schema,
                       automsgs::msgs::geometry_msgs::Twist* twist,
                       std::string* error) {
        const int dimension =
            schema.action_dimension > 0
                ? schema.action_dimension
                : static_cast<int>(schema.slots.size());
        const int horizon = std::max(1, schema.action_horizon);
        if (dimension <= 0) {
            if (error != nullptr) {
                *error = "Diffusion action_dimension is invalid";
            }
            return false;
        }
        const int execute =
            std::max(0, std::min(schema.execute_step_index, horizon - 1));
        const std::size_t offset =
            static_cast<std::size_t>(execute) *
            static_cast<std::size_t>(dimension);
        return DecodeTwistComponents(raw, schema, offset, twist, error);
    }
};

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
