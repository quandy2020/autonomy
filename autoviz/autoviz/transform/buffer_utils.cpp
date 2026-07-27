/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/transform/buffer_utils.hpp"

#include "autoviz/transform/buffer.hpp"

namespace autoviz {
namespace transform {

void ApplyTfMessageToBuffer(
    Buffer* buffer, const automsgs::msgs::tf2_msgs::TFMessage& message,
    const std::string& authority, const bool is_static) {
  if (buffer == nullptr) {
    return;
  }
  for (const auto& transform : message.transforms()) {
    buffer->setTransform(transform, authority, is_static);
  }
}

}  // namespace transform
}  // namespace autoviz
