/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/grid_cells_display.hpp"

#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/transform_utils.hpp"

namespace autoviz {
namespace display {

GridCellsDisplay::GridCellsDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::map_msgs::GridCells>(
          "GridCells", std::move(channel),
          "automsgs.msgs.map_msgs.GridCells") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> GridCellsDisplay::propertySpecs() const {
  return {{"color", "Color", "255;200;80"}, {"alpha", "Alpha", "0.8"}};
}

void GridCellsDisplay::processMessage(
    const automsgs::msgs::map_msgs::GridCells& message) {
  cells_.clear();
  if (context_ == nullptr) {
    return;
  }
  const float width = message.cell_width();
  const float height = message.cell_height();
  if (width <= 0.f || height <= 0.f) {
    return;
  }

  const auto zero_time = autoviz::commsgs::ZeroTime();
  const std::string frame = message.header().frame_id().empty()
                                ? context_->fixed_frame
                                : message.header().frame_id();
  try {
    const auto tf = context_->tf_buffer->lookupTransform(context_->fixed_frame,
                                                          frame, zero_time);
    for (const auto& point : message.cells()) {
      const QVector3D local(static_cast<float>(point.x()),
                            static_cast<float>(point.y()),
                            static_cast<float>(point.z()) + 0.01f);
      cells_.push_back({transformPoint(tf, local), width, height});
    }
  } catch (...) {
    for (const auto& point : message.cells()) {
      cells_.push_back({QVector3D(static_cast<float>(point.x()),
                                  static_cast<float>(point.y()),
                                  static_cast<float>(point.z()) + 0.01f),
                        width,
                        height});
    }
  }
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void GridCellsDisplay::clearReceivedData() { cells_.clear(); }

void GridCellsDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (cells_.empty()) {
    return;
  }
  QColor color =
      common::ParseColorProperty(propertyValue("color", "255;200;80"));
  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "0.8"), 0.8f);
  color.setAlphaF(alpha);

  for (const Cell& cell : cells_) {
    const float hw = cell.width* 0.5f;
    const float hh = cell.height* 0.5f;
    const QVector3D c = cell.center;
    const QVector3D p0(c.x() - hw, c.y() - hh, c.z());
    const QVector3D p1(c.x() + hw, c.y() - hh, c.z());
    const QVector3D p2(c.x() + hw, c.y() + hh, c.z());
    const QVector3D p3(c.x() - hw, c.y() + hh, c.z());
    scene.addLine(p0, p1, color);
    scene.addLine(p1, p2, color);
    scene.addLine(p2, p3, color);
    scene.addLine(p3, p0, color);
  }
}

}  // namespace display
}  // namespace autoviz
