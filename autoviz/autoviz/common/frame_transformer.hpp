/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <functional>
#include <memory>
#include <string>

#include "autoviz/common/plugin_info.hpp"

namespace autoviz {
namespace transform {
class Buffer;
}

namespace common {

/** rviz_common::FrameTransformer — pluggable TF backend. */
class FrameTransformer {
 public:
  virtual ~FrameTransformer() = default;

  virtual PluginInfo info() const = 0;
  virtual transform::Buffer* buffer() const = 0;
  virtual bool identityMode() const { return false; }
};

class AutolinkTfFrameTransformer : public FrameTransformer {
 public:
  explicit AutolinkTfFrameTransformer(transform::Buffer* buffer);

  PluginInfo info() const override;
  transform::Buffer* buffer() const override;

 private:
  transform::Buffer* buffer_ = nullptr;
};

class IdentityFrameTransformer : public FrameTransformer {
 public:
  PluginInfo info() const override;
  transform::Buffer* buffer() const override { return nullptr; }
  bool identityMode() const override { return true; }
};

using FrameTransformerCreator =
    std::function<std::unique_ptr<FrameTransformer>(transform::Buffer* buffer)>;

}  // namespace common
}  // namespace autoviz
