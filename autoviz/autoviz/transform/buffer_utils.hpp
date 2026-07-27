/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>

#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>

#include "autoviz/transform/buffer.hpp"

namespace autoviz {
namespace transform {

void ApplyTfMessageToBuffer(
    Buffer* buffer, const automsgs::msgs::tf2_msgs::TFMessage& message,
    const std::string& authority, bool is_static = false);

}  // namespace transform
}  // namespace autoviz
