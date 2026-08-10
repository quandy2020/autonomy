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
  // One TFMessage can contain dozens of transforms; each setTransform used to
  // fire transforms-changed. That floods UI listeners and freezes Autoviz when
  // returning from the background.
  buffer->_setTransformsChangedSuppressed(true);
  for (const auto& transform : message.transforms()) {
    buffer->setTransform(transform, authority, is_static);
  }
  buffer->_setTransformsChangedSuppressed(false);
  if (!message.transforms().empty()) {
    buffer->_emitTransformsChanged();
  }
}

}  // namespace transform
}  // namespace autoviz
