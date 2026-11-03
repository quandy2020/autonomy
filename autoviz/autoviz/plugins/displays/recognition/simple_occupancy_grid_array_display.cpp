/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/displays/recognition/simple_occupancy_grid_array_display.hpp"

#include <utility>

#include <QMatrix4x4>

#include "autoviz/plugins/displays/common/color_category.hpp"
#include "autoviz/plugins/displays/common/display_tf_utils.hpp"

namespace autoviz {
namespace display {

SimpleOccupancyGridArrayDisplay::SimpleOccupancyGridArrayDisplay(
    std::string channel)
    : ChannelDisplay<
          automsgs::msgs::visualization_msgs::SimpleOccupancyGridArray>(
          "SimpleOccupancyGridArray", std::move(channel),
          "automsgs.msgs.visualization_msgs.SimpleOccupancyGridArray") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec>
SimpleOccupancyGridArrayDisplay::propertySpecs() const {
  return {{"coloring", "Coloring", "Auto", {"Auto", "Flat"}},
          {"color", "Color", "127;127;127", {},
           common::DisplayPropertyKind::kColor},
          {"alpha", "Alpha", "0.9"},
          {"style", "Style", "Points", {"Points", "Boxes"}},
          {"cell_scale", "Cell Scale", "1.0"}};
}

void SimpleOccupancyGridArrayDisplay::clearReceivedData() { cells_.clear(); }

void SimpleOccupancyGridArrayDisplay::processMessage(
    const automsgs::msgs::visualization_msgs::SimpleOccupancyGridArray&
        message) {
  cells_.clear();
  if (context_ == nullptr) {
    return;
  }
  automsgs::msgs::geometry_msgs::TransformStamped array_tf;
  const bool have_array_tf = viz::LookupFixedFrameTransform(
      context_, message.header().frame_id(), &array_tf);

  const bool auto_color = propertyValue("coloring", "Auto") == "Auto";
  QColor flat = common::ParseColorProperty(
      propertyValue("color", "127;127;127"), QColor(127, 127, 127));
  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "0.9"), 0.9f);
  flat.setAlphaF(alpha);
  const float cell_scale =
      common::ParseFloatProperty(propertyValue("cell_scale", "1.0"), 1.f);

  std::size_t grid_index = 0;
  for (int g = 0; g < message.grids_size(); ++g) {
    const auto& grid = message.grids(g);
    automsgs::msgs::geometry_msgs::TransformStamped tf = array_tf;
    bool have_tf = have_array_tf;
    if (!grid.header().frame_id().empty()) {
      have_tf = viz::LookupFixedFrameTransform(context_, grid.header().frame_id(),
                                               &tf);
    }
    const float res =
        grid.resolution() > 0.f ? grid.resolution() : 0.05f;
    const float half = 0.5f * res * cell_scale;
    const QVector3D half_size(half, half, half);
    QColor grid_color =
        auto_color ? viz::ColorCategory20(grid_index++) : flat;
    grid_color.setAlphaF(alpha);

    for (int c = 0; c < grid.cells_size(); ++c) {
      const auto& cell = grid.cells(c);
      CellVisual visual;
      visual.center = viz::TransformLocalPoint(
          tf, have_tf,
          QVector3D(static_cast<float>(cell.x()), static_cast<float>(cell.y()),
                    static_cast<float>(cell.z())));
      visual.half_size = half_size;
      visual.color = grid_color;
      cells_.push_back(visual);
    }
  }
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void SimpleOccupancyGridArrayDisplay::onDraw(
    rendering::SceneOverlay& scene) {
  const bool boxes = propertyValue("style", "Points") == "Boxes";
  if (boxes) {
    for (const CellVisual& cell : cells_) {
      QMatrix4x4 transform;
      transform.translate(cell.center);
      scene.addBoxSolid(QVector3D(0.f, 0.f, 0.f), cell.half_size, transform,
                        cell.color);
    }
    return;
  }
  for (const CellVisual& cell : cells_) {
    scene.addPoint(cell.center, cell.color);
  }
}

}  // namespace display
}  // namespace autoviz
