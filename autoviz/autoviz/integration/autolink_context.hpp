/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file autolink_context.hpp
 * @brief Owns Autolink init / shutdown and the process @c Node for Autoviz.
 *
 * Constructed early in @c main; after @ref initialize(), registries
 * (@ref ChannelReaderRegistry, @ref ChannelWriterRegistry, …) receive
 * @ref node() via @c setNode().
 *
 * @see ChannelReaderRegistry
 * @see ChannelWriterRegistry
 * @see ServiceClientRegistry
 * @see PlaybackController
 */

#pragma once

#include <memory>
#include <string>

#include "autolink/node/node.hpp"

namespace autoviz {
namespace integration {

/**
 * @class AutolinkContext
 * @brief RAII-ish wrapper around Autolink library init and the app node.
 *
 * Non-copyable. Call @ref initialize() once with the process binary name and
 * desired node name; @ref shutdown() (also invoked from the destructor path
 * when implemented) tears down Autolink.
 *
 * @note Does not create readers/writers itself — only exposes @ref node().
 */
class AutolinkContext {
 public:
  /** @brief Constructs an uninitialized context. */
  AutolinkContext() = default;

  /**
   * @brief Shuts down Autolink if still initialized.
   */
  ~AutolinkContext();

  AutolinkContext(const AutolinkContext&) = delete;
  AutolinkContext& operator=(const AutolinkContext&) = delete;

  /**
   * @brief Initializes Autolink and creates the application node.
   *
   * @param binary_name @c argv[0] (or equivalent) for Autolink process identity.
   * @param node_name Autolink node name visible in topology (e.g. @c "autoviz").
   * @return @c true on success; @c false if init / node creation failed.
   *
   * @see ok()
   * @see node()
   */
  bool initialize(const char* binary_name, const std::string& node_name);

  /**
   * @brief Tears down the node and Autolink runtime.
   *
   * Safe to call multiple times; subsequent @ref ok() returns @c false.
   */
  void shutdown();

  /**
   * @brief Whether Autolink was initialized and a node is available.
   * @return @c true when @c initialized_ and @c node_ is non-null.
   */
  bool ok() const;

  /**
   * @brief Returns the shared Autolink node for registries and tools.
   * @return Shared pointer (may be empty if not initialized).
   */
  std::shared_ptr<::autolink::Node> node() const { return node_; }

 private:
  /** Application Autolink node (shared with registries / tools). */
  std::shared_ptr<::autolink::Node> node_;

  /** @c true after a successful @ref initialize() until @ref shutdown(). */
  bool initialized_ = false;
};

}  // namespace integration
}  // namespace autoviz
