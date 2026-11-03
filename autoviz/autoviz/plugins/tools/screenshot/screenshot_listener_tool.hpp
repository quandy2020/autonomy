/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

/**
 * @file screenshot_listener_tool.hpp
 * @brief screenshot listener — notes screenshot path on activation (stub).
 */

#pragma once

#include <string>

#include "autoviz/common/tool.hpp"

namespace autoviz {
namespace tools {

/**
 * @class ScreenshotListenerTool
 * @brief One-shot tool that records the configured screenshot output path.
 *
 * Full frame capture is handled by @ref FrameSession::captureScreenshot(); this
 * tool only surfaces the @c screenshot_path property when activated.
 */
class ScreenshotListenerTool : public common::Tool {
 public:
  std::string id() const override { return "ScreenshotListener"; }
  QString label() const override {
    return QStringLiteral("Screenshot Listener");
  }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

  void activate(common::ToolContext* context) override;

  QString statusText() const override;

 private:
  std::string noted_path_;
};

}  // namespace tools
}  // namespace autoviz
