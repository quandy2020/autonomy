/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/transformation/frame_transformer.hpp"

#include "autoviz/transform/buffer.hpp"

namespace autoviz {
namespace common {
namespace transformation {

AutolinkTfFrameTransformer::AutolinkTfFrameTransformer(
    transform::Buffer* buffer)
    : buffer_(buffer) {}

PluginInfo AutolinkTfFrameTransformer::info() const {
  PluginInfo info;
  info.class_id = "autoviz/AutolinkTf";
  info.name = "Autolink TF";
  info.description = "Transform using the Autolink TF buffer (default).";
  info.package = "autoviz";
  return info;
}

transform::Buffer* AutolinkTfFrameTransformer::buffer() const {
  return buffer_;
}

}  // namespace transformation
}  // namespace common
}  // namespace autoviz
