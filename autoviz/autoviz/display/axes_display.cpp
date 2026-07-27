/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/axes_display.hpp"

#include <QColor>

#include "autoviz/common/display_property.hpp"
#include "autoviz/display/ogre_overlay_draw.hpp"

namespace autoviz {
namespace display {

AxesDisplay::AxesDisplay() {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> AxesDisplay::propertySpecs() const {
  return {{"length", "Length", "1.0"}, {"radius", "Radius", "0.05"}};
}

void AxesDisplay::onDraw(rendering::SceneOverlay& scene) {
  const float length =
      common::ParseFloatProperty(propertyValue("length", "1.0"), 1.f);
  const QVector3D origin(0.f, 0.f, 0.f);
  drawLineSegmentsOgreOrGl(
      context_, scene, name(),
      {{origin, QVector3D(length, 0.f, 0.f), QColor(220, 60, 60)},
       {origin, QVector3D(0.f, length, 0.f), QColor(60, 220, 60)},
       {origin, QVector3D(0.f, 0.f, length), QColor(60, 120, 220)}});
}

}  // namespace display
}  // namespace autoviz
