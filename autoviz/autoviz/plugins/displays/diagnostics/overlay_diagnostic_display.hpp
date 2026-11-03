/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <QColor>

#include <automsgs/msgs/diagnostic_msgs/diagnostic_array.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/channel_display.hpp"
#include "autoviz/plugins/displays/common/overlay_object.hpp"

namespace autoviz {
namespace display {

/**
 * @class OverlayDiagnosticDisplay
 * @brief Viewport HUD summarizing @c diagnostic_msgs/DiagnosticArray.
 */
class OverlayDiagnosticDisplay
    : public ChannelDisplay<automsgs::msgs::diagnostic_msgs::DiagnosticArray> {
 public:
  explicit OverlayDiagnosticDisplay(std::string channel);

  std::string typeId() const override { return "OverlayDiagnostic"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::diagnostic_msgs::DiagnosticArray& message)
      override;
  void clearReceivedData() override;
  void onEnable() override;
  void onDisable() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  void ensureOverlay();
  void refreshTexture();

  struct Line {
    QString text;
    QColor color;
  };

  viz::OverlayObject::Ptr overlay_;
  std::vector<Line> lines_;
  bool require_update_texture_ = false;
};

}  // namespace display
}  // namespace autoviz
