/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/mesh_shading.hpp"

#include <cmath>

namespace autoviz {
namespace display {
namespace {

const QVector3D& KeyLight() {
  static const QVector3D light = QVector3D(0.35f, -0.55f, 0.75f).normalized();
  return light;
}

constexpr float kAmbient = 0.4f;

}  // namespace

QColor shadeTriangleColor(const QColor& color, const QVector3D& a,
                          const QVector3D& b, const QVector3D& c) {
  const QVector3D normal = QVector3D::crossProduct(b - a, c - a);
  if (normal.lengthSquared() < 1e-12f) {
    return color;
  }
  const float lambert =
      std::fabs(QVector3D::dotProduct(normal.normalized(), KeyLight()));
  const float scale = kAmbient + (1.f - kAmbient) * lambert;
  QColor shaded = color;
  shaded.setRedF(color.redF() * scale);
  shaded.setGreenF(color.greenF() * scale);
  shaded.setBlueF(color.blueF() * scale);
  return shaded;
}

}  // namespace display
}  // namespace autoviz
