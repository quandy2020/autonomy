/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file buffer_utils.hpp
 * @brief Helpers to apply an Automsgs @c TFMessage into the Autoviz @ref Buffer.
 *
 * Used by @ref Listener after protobuf parse of @c /tf and @c /tf_static
 * payloads.
 *
 * @see Buffer::setTransform
 * @see Listener::applyPayload
 */

#pragma once

#include <string>

#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>

#include "autoviz/transform/buffer.hpp"

namespace autoviz {
namespace transform {

/**
 * @brief Inserts every transform in @p message into @p buffer.
 *
 * @param buffer Non-null Autoviz TF buffer.
 * @param message Parsed @c tf2_msgs/TFMessage (dynamic or static).
 * @param authority Broadcaster / authority string recorded in frame stats.
 * @param is_static When @c true, transforms are stored as static TF.
 *
 * @note No-op fields are still passed through to @ref Buffer::setTransform;
 *       invalid individual transforms follow BufferCore error handling.
 */
void ApplyTfMessageToBuffer(
    Buffer* buffer, const automsgs::msgs::tf2_msgs::TFMessage& message,
    const std::string& authority, bool is_static = false);

}  // namespace transform
}  // namespace autoviz
