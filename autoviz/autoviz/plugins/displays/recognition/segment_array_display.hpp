/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

#include <QColor>
#include <QVector3D>

#include <automsgs/msgs/visualization_msgs/segment_array.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/channel_display.hpp"

namespace autoviz {
namespace display {

/**
 * @class SegmentArrayDisplay
 * @brief Draws @c visualization_msgs/SegmentArray as colored line segments.
 */
class SegmentArrayDisplay
    : public ChannelDisplay<
          automsgs::msgs::visualization_msgs::SegmentArray> {
 public:
  explicit SegmentArrayDisplay(std::string channel);

  std::string typeId() const override { return "SegmentArray"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::visualization_msgs::SegmentArray& message)
      override;
  void clearReceivedData() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  struct SegmentVisual {
    QVector3D start;
    QVector3D end;
    QColor color;
  };

  std::vector<SegmentVisual> segments_;
};

}  // namespace display
}  // namespace autoviz
