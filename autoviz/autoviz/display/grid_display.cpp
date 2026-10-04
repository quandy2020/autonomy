/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/grid_display.hpp"

#include <QColor>
#include <QMatrix4x4>

#include "autoviz/common/display_property.hpp"
#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/display/ogre_overlay_draw.hpp"
#include "autoviz/display/transform_utils.hpp"

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

std::string resolveReferenceFrame(const common::DisplayContext* context,
                                  const std::string& reference_frame) {
  if (context == nullptr) {
    return {};
  }
  if (reference_frame.empty() || reference_frame == "<Fixed Frame>") {
    return context->fixed_frame;
  }
  return reference_frame;
}

}  // namespace

GridDisplay::GridDisplay() {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> GridDisplay::propertySpecs() const {
  // Property set / defaults match rviz_default_plugins GridDisplay.
  return {{"reference_frame", "Reference Frame", "<Fixed Frame>"},
          {"cell_count", "Plane Cell Count", "10", {},
           common::DisplayPropertyKind::kInt},
          {"normal_cell_count", "Normal Cell Count", "0", {},
           common::DisplayPropertyKind::kInt},
          {"cell_size", "Cell Size", "1.0"},
          {"line_style", "Line Style", "Lines", {"Lines", "Billboards"}},
          {"line_width", "Line Width", "0.03", {},
           common::DisplayPropertyKind::kAuto, "line_style", "Billboards",
           "line_style"},
          {"color", "Color", "160;160;160", {}, common::DisplayPropertyKind::kColor},
          {"alpha", "Alpha", "0.5"},
          {"plane", "Plane", "XY", {"XY", "XZ", "YZ"}},
          {"offset", "Offset", "0;0;0", {}, common::DisplayPropertyKind::kAuto, "",
           "", "", true}};
}

void GridDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (context_ == nullptr) {
    return;
  }

  const int cell_count = std::max(
      1, common::ParseIntProperty(propertyValue("cell_count", "10"), 10));
  const int normal_cells = std::max(
      0, common::ParseIntProperty(propertyValue("normal_cell_count", "0"), 0));
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

  const std::string frame =
      resolveReferenceFrame(context_, propertyValue("reference_frame",
                                                    "<Fixed Frame>"));
  QMatrix4x4 frame_to_fixed;
  frame_to_fixed.setToIdentity();
  if (frame != context_->fixed_frame) {
    if (context_->tf_buffer == nullptr) {
      setStatusWarn("TF buffer not ready");
      return;
    }
    try {
      const auto tf = context_->tf_buffer->lookupTransform(
          context_->fixed_frame, frame, autoviz::commsgs::ZeroTime());
      frame_to_fixed = transformToMatrix(tf);
    } catch (...) {
      setStatusWarn("No transform from [" + frame + "] to [" +
                    context_->fixed_frame + "]");
      return;
    }
  }

  const float extent = static_cast<float>(cell_count) * cell_size * 0.5f;
  const bool use_billboards = line_style == "Billboards";
  int line_index = 0;

  auto draw_grid_line = [&](const QVector3D& a, const QVector3D& b) {
    const QVector3D mapped_a =
        frame_to_fixed.map(mapGridPoint(a, plane) + offset);
    const QVector3D mapped_b =
        frame_to_fixed.map(mapGridPoint(b, plane) + offset);
    const std::string suffix = "/line/" + std::to_string(line_index++);
    if (use_billboards) {
      drawBillboardStripOgreOrGl(context_, scene, name() + suffix,
                                 {mapped_a, mapped_b}, color, line_width);
      return;
    }
    scene.addLine(mapped_a, mapped_b, color);
  };

  // Match rviz_rendering::Grid: (normal_cells + 1) planes along the normal,
  // plus verticals between the outermost planes when normal_cells > 0.
  for (int height = 0; height <= normal_cells; ++height) {
    const float real_height =
        (static_cast<float>(normal_cells) * 0.5f -
         static_cast<float>(height)) *
        cell_size;
    for (int i = 0; i <= cell_count; ++i) {
      const float p = extent - static_cast<float>(i) * cell_size;
      draw_grid_line(QVector3D(-extent, p, real_height),
                     QVector3D(extent, p, real_height));
      draw_grid_line(QVector3D(p, -extent, real_height),
                     QVector3D(p, extent, real_height));
    }
  }

  if (normal_cells > 0) {
    const float half_height =
        static_cast<float>(normal_cells) * 0.5f * cell_size;
    for (int x = 0; x <= cell_count; ++x) {
      for (int y = 0; y <= cell_count; ++y) {
        const float x_real = extent - static_cast<float>(x) * cell_size;
        const float y_real = extent - static_cast<float>(y) * cell_size;
        draw_grid_line(QVector3D(x_real, y_real, -half_height),
                       QVector3D(x_real, y_real, half_height));
      }
    }
  }

  setStatusOk();
}

}  // namespace display
}  // namespace autoviz
