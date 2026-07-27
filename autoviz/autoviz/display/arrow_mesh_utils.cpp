/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/arrow_mesh_utils.hpp"

#include <algorithm>

#include <QtMath>

#include <QMatrix4x4>
#include <QQuaternion>

#include "autoviz/display/primitive_mesh.hpp"

namespace autoviz {
namespace display {
namespace {

const ObjMesh& UnitShaftMesh() {
  static const ObjMesh mesh = buildCylinderMesh(0.5f, 1.f);
  return mesh;
}

const ObjMesh& UnitHeadMesh() {
  static const ObjMesh mesh = buildConeMesh(0.5f, 1.f);
  return mesh;
}

QMatrix4x4 AlignZToDirection(const QVector3D& direction) {
  const QVector3D dir = direction.normalized();
  return QMatrix4x4(QQuaternion::rotationTo(QVector3D(0.f, 0.f, 1.f), dir).toRotationMatrix());
}

}  // namespace

void appendSolidArrowMeshes(std::vector<ColoredMeshInstance>* meshes,
                            const QVector3D& start, const QVector3D& end,
                            const QColor& color, float head_fraction,
                            float shaft_diameter, float head_diameter) {
  if (meshes == nullptr) {
    return;
  }
  const QVector3D delta = end - start;
  const float length = delta.length();
  if (length < 1e-4f) {
    return;
  }
  const QVector3D direction = delta / length;
  const float head_fraction_clamped = std::clamp(head_fraction, 0.05f, 0.95f);
  const float head_len = length * head_fraction_clamped;
  const float shaft_len = length - head_len;
  if (shaft_diameter <= 0.f) {
    shaft_diameter = std::max(0.02f, length * 0.08f);
  }
  if (head_diameter <= 0.f) {
    head_diameter = std::max(shaft_diameter * 1.5f, length * 0.16f);
  }

  QMatrix4x4 shaft;
  shaft.setToIdentity();
  shaft.translate(start + direction * (shaft_len * 0.5f));
  shaft *= AlignZToDirection(direction);
  shaft.scale(shaft_diameter, shaft_diameter, shaft_len);
  meshes->push_back({UnitShaftMesh(), shaft, color, false});

  QMatrix4x4 head;
  head.setToIdentity();
  head.translate(start + direction * shaft_len);
  head *= AlignZToDirection(direction);
  head.scale(head_diameter, head_diameter, head_len);
  meshes->push_back({UnitHeadMesh(), head, color, false});
}

}  // namespace display
}  // namespace autoviz
