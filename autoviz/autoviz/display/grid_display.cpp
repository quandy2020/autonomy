/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/grid_display.hpp"

#include <QColor>

#include "autoviz/common/display_property.hpp"
#include "autoviz/display/ogre_overlay_draw.hpp"

namespace autoviz {
namespace display {
namespace {

QVector3D mapGridPoint(const QVector3D& point, const std::string& plane) {
  if (plane == "XZ") {
    return QVector3D(point.x(), point.z(), point.y());
  }
  if (plane == "YZ") {
    return QVector3D(point.y(), point.z(), point.x());
  }
  return point;
}

}  // namespace

GridDisplay::GridDisplay() {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> GridDisplay::propertySpecs() const {
  return {{"reference_frame", "Reference Frame", "<Fixed Frame>"},
          {"cell_count", "Plane Cell Count", "20"},
          {"normal_cell_count", "Normal Cell Count", "0"},
          {"cell_size", "Cell Size", "1.0"},
          {"line_style", "Line Style", "Lines", {"Lines", "Billboards"}},
          {"line_width", "Line Width", "0.03"},
          {"color", "Color", "160;160;160", {}, common::DisplayPropertyKind::kColor},
          {"alpha", "Alpha", "0.5"},
          {"plane", "Plane", "XY", {"XY", "XZ", "YZ"}},
          {"offset", "Offset", "0;0;0"}};
}

void GridDisplay::onDraw(rendering::SceneOverlay& scene) {
  const int half_cells = std::max(
      1, common::ParseIntProperty(propertyValue("cell_count", "20"), 20));
  const float cell_size =
      common::ParseFloatProperty(propertyValue("cell_size", "1.0"), 1.f);
  const QColor base =
      common::ParseColorProperty(propertyValue("color", "160;160;160"),
                                 QColor(160, 160, 160));
  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "0.5"), 0.5f);
  const std::string line_style = propertyValue("line_style", "Lines");
  const float line_width =
      common::ParseFloatProperty(propertyValue("line_width", "0.03"), 0.03f);
  const std::string plane = propertyValue("plane", "XY");
  const QVector3D offset = common::ParseVector3Property(
      propertyValue("offset", "0;0;0"), QVector3D());

  QColor color = base;
  color.setAlphaF(alpha);

  const float extent = static_cast<float>(half_cells) * cell_size;
  const bool use_billboards = line_style == "Billboards";

  auto draw_grid_line = [&](const QVector3D& a, const QVector3D& b,
                            const std::string& suffix) {
    const QVector3D mapped_a = mapGridPoint(a, plane) + offset;
    const QVector3D mapped_b = mapGridPoint(b, plane) + offset;
    if (use_billboards) {
      drawBillboardStripOgreOrGl(context_, scene, name() + suffix,
                               {mapped_a, mapped_b}, color, line_width);
      return;
    }
    scene.addLine(mapped_a, mapped_b, color);
  };

  for (int i = -half_cells; i <= half_cells; ++i) {
    const float p = static_cast<float>(i) * cell_size;
    draw_grid_line(QVector3D(-extent, p, 0.f), QVector3D(extent, p, 0.f),
                   "/line/y/" + std::to_string(i));
    draw_grid_line(QVector3D(p, -extent, 0.f), QVector3D(p, extent, 0.f),
                   "/line/x/" + std::to_string(i));
  }

  const int normal_cells = std::max(
      0, common::ParseIntProperty(propertyValue("normal_cell_count", "0"), 0));
  if (normal_cells > 0) {
    const float normal_extent = static_cast<float>(normal_cells) * cell_size;
    for (int i = -normal_cells; i <= normal_cells; ++i) {
      const float p = static_cast<float>(i) * cell_size;
      draw_grid_line(QVector3D(-extent, 0.f, p), QVector3D(extent, 0.f, p),
                     "/normal/y/" + std::to_string(i));
      draw_grid_line(QVector3D(p, 0.f, -extent), QVector3D(p, 0.f, normal_extent),
                     "/normal/x/" + std::to_string(i));
    }
  }
}

}  // namespace display
}  // namespace autoviz
