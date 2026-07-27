/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/frame_transformer.hpp"

#include "autoviz/transform/buffer.hpp"

namespace autoviz {
namespace common {

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

PluginInfo IdentityFrameTransformer::info() const {
  PluginInfo info;
  info.class_id = "autoviz/Identity";
  info.name = "Identity";
  info.description =
      "Always use identity transforms (ignore TF data).";
  info.package = "autoviz";
  return info;
}

}  // namespace common
}  // namespace autoviz
