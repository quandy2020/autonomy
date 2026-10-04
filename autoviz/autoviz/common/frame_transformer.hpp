/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file frame_transformer.hpp
 * @brief Pluggable TF backend interface and built-in implementations.
 *
 * Mirrors @c rviz_common::FrameTransformer: @ref TransformationManager selects
 * an active implementation that exposes a @c transform::Buffer (or identity
 * mode with no buffer).
 *
 * @see TransformationManager
 * @see FrameManager
 * @see PluginInfo
 */

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

/**
 * @class FrameTransformer
 * @brief Abstract TF backend used by @ref TransformationManager.
 */
class FrameTransformer {
 public:
  virtual ~FrameTransformer() = default;

  /**
   * @brief Plugin metadata for UI and session persistence.
   * @return @ref PluginInfo describing this transformer.
   */
  virtual PluginInfo info() const = 0;

  /**
   * @brief Underlying TF buffer, or @c nullptr in identity mode.
   * @return Non-owning buffer pointer.
   */
  virtual transform::Buffer* buffer() const = 0;

  /**
   * @brief Whether transforms are treated as identity (no TF lookup).
   * @return @c true for @ref IdentityFrameTransformer; default @c false.
   */
  virtual bool identityMode() const { return false; }
};

/**
 * @class AutolinkTfFrameTransformer
 * @brief Default transformer wrapping Autoviz / Autolink @c transform::Buffer.
 */
class AutolinkTfFrameTransformer : public FrameTransformer {
 public:
  /**
   * @brief Constructs a transformer around an existing buffer.
   * @param buffer Non-owning TF buffer (must outlive this object).
   */
  explicit AutolinkTfFrameTransformer(transform::Buffer* buffer);

  /**
   * @brief Returns Autolink TF plugin info (@c "autoviz/AutolinkTf").
   * @return Plugin metadata.
   */
  PluginInfo info() const override;

  /**
   * @brief Returns the wrapped buffer.
   * @return Buffer passed to the constructor.
   */
  transform::Buffer* buffer() const override;

 private:
  transform::Buffer* buffer_ = nullptr; /**< Non-owning TF buffer. */
};

/**
 * @class IdentityFrameTransformer
 * @brief No-op transformer: all frames are treated as coincident.
 *
 * Useful for datasets without TF or for debugging. @ref buffer() is always
 * @c nullptr and @ref identityMode() is @c true.
 */
class IdentityFrameTransformer : public FrameTransformer {
 public:
  /**
   * @brief Returns identity-mode plugin info.
   * @return Plugin metadata.
   */
  PluginInfo info() const override;

  /**
   * @brief Always @c nullptr (no buffer).
   * @return @c nullptr.
   */
  transform::Buffer* buffer() const override { return nullptr; }

  /**
   * @brief Always @c true.
   * @return @c true.
   */
  bool identityMode() const override { return true; }
};

/**
 * @brief Factory that creates a @ref FrameTransformer for a given buffer.
 */
using FrameTransformerCreator =
    std::function<std::unique_ptr<FrameTransformer>(transform::Buffer* buffer)>;

}  // namespace common
}  // namespace autoviz
