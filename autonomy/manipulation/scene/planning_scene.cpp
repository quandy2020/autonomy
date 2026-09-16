/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/scene/planning_scene.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace autonomy {
namespace manipulation {
namespace scene {
namespace {

void RotateVec(const core::Transform& t, double x, double y, double z,
               double* ox, double* oy, double* oz) {
  const double qw = t.qw;
  const double qx = t.qx;
  const double qy = t.qy;
  const double qz = t.qz;
  const double ix = qw * x + qy * z - qz * y;
  const double iy = qw * y + qz * x - qx * z;
  const double iz = qw * z + qx * y - qy * x;
  const double iw = -qx * x - qy * y - qz * z;
  *ox = ix * qw + iw * -qx + iy * -qz - iz * -qy;
  *oy = iy * qw + iw * -qy + iz * -qx - ix * -qz;
  *oz = iz * qw + iw * -qz + ix * -qy - iy * -qx;
}

}  // namespace

void UpdateMeshAabb(CollisionObject* object) {
  if (!object || object->mesh_vertices.empty()) {
    return;
  }
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double min_z = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  double max_z = -std::numeric_limits<double>::infinity();
  for (const auto& v : object->mesh_vertices) {
    min_x = std::min(min_x, v.x);
    min_y = std::min(min_y, v.y);
    min_z = std::min(min_z, v.z);
    max_x = std::max(max_x, v.x);
    max_y = std::max(max_y, v.y);
    max_z = std::max(max_z, v.z);
  }
  object->type = ShapeType::kMesh;
  object->x = 0.5 * (min_x + max_x);
  object->y = 0.5 * (min_y + max_y);
  object->z = 0.5 * (min_z + max_z);
  object->size_x = std::max(1e-4, max_x - min_x);
  object->size_y = std::max(1e-4, max_y - min_y);
  object->size_z = std::max(1e-4, max_z - min_z);
}

CollisionObject TransformAttached(const CollisionObject& object,
                                  const core::Transform& link_tf) {
  CollisionObject out = object;
  double wx = 0.0;
  double wy = 0.0;
  double wz = 0.0;
  RotateVec(link_tf, object.x, object.y, object.z, &wx, &wy, &wz);
  out.x = link_tf.x + wx;
  out.y = link_tf.y + wy;
  out.z = link_tf.z + wz;
  return out;
}

}  // namespace scene
}  // namespace manipulation
}  // namespace autonomy
