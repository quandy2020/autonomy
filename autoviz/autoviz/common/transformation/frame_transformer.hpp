/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file frame_transformer.hpp
 * @brief Pluggable TF backend interface (RViz @c rviz_common::transformation).
 *
 * @see TransformationManager
 * @see FrameManager
 * @see PluginInfo
 */

#pragma once

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "autoviz/common/plugin_info.hpp"

namespace autoviz {
namespace transform {
class Buffer;
}

namespace common {
namespace transformation {

/**
 * @class FrameTransformerException
 * @brief Thrown by frame transformers when a lookup / transform fails.
 */
class FrameTransformerException : public std::runtime_error {
 public:
  explicit FrameTransformerException(const std::string& description)
      : std::runtime_error(description) {}
};

/**
 * @class TransformationLibraryConnector
 * @brief Marker base for the active TF library (RViz connector iface subset).
 */
class TransformationLibraryConnector {
 public:
  virtual ~TransformationLibraryConnector() = default;

  using WeakPtr = std::weak_ptr<TransformationLibraryConnector>;
};

/**
 * @class FrameTransformer
 * @brief Abstract TF backend used by @ref TransformationManager.
 */
class FrameTransformer {
 public:
  virtual ~FrameTransformer() = default;

  virtual PluginInfo info() const = 0;

  virtual transform::Buffer* buffer() const = 0;

  virtual bool identityMode() const { return false; }
};

/**
 * @class AutolinkTfFrameTransformer
 * @brief Legacy thin buffer wrapper.
 *
 * Prefer @c display::transformation::TFFrameTransformer (registered as
 * @c "autoviz/AutolinkTf"). Kept for direct construction / tests.
 */
class AutolinkTfFrameTransformer : public FrameTransformer {
 public:
  explicit AutolinkTfFrameTransformer(transform::Buffer* buffer);

  PluginInfo info() const override;

  transform::Buffer* buffer() const override;

 private:
  transform::Buffer* buffer_ = nullptr;
};

using FrameTransformerCreator =
    std::function<std::unique_ptr<FrameTransformer>(transform::Buffer* buffer)>;

}  // namespace transformation
}  // namespace common
}  // namespace autoviz
