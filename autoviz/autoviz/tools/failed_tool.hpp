/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file failed_tool.hpp
 * @brief Placeholder tool when a plugin fails to load (RViz FailedTool).
 *
 * Registered in place of a missing / unloadable tool so the toolbar and
 * session config still have a stable @ref id() while surfacing @c reason_
 * in the label and status text.
 *
 * @see common::Tool
 */

#pragma once

#include <string>

#include "autoviz/common/tool.hpp"

namespace autoviz {
namespace tools {

/**
 * @class FailedTool
 * @brief Non-interactive stub representing a tool that failed to instantiate.
 *
 * Mimics @c rviz_common::FailedTool: mouse events fall through to the base
 * (unconsumed); UI shows the original id and failure reason.
 */
class FailedTool : public common::Tool {
 public:
  /**
   * @brief Constructs a failed-tool placeholder.
   *
   * @param id Intended tool id that failed to load.
   * @param reason Human-readable failure explanation (shown in UI).
   */
  FailedTool(std::string id, std::string reason);

  /**
   * @brief Returns the original / intended tool id.
   * @return Copy of @c id_.
   */
  std::string id() const override { return id_; }

  /**
   * @brief Label indicating failure, typically including @c id_ / @c reason_.
   * @return Localized failure label for the toolbar.
   */
  QString label() const override;

  /**
   * @brief Status-bar text describing why the tool is unavailable.
   * @return Failure reason for the frame chrome.
   */
  QString statusText() const override;

 private:
  /** Intended tool id that could not be loaded. */
  std::string id_;

  /** Explanation shown in label / status (plugin error, missing class, …). */
  std::string reason_;
};

}  // namespace tools
}  // namespace autoviz
