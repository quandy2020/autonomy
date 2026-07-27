/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include "autoviz/display/obj_mesh.hpp"

namespace autoviz {
namespace display {

ObjMesh buildCylinderMesh(float radius, float length, int slices = 16);
ObjMesh buildSphereMesh(float radius, int slices = 16, int stacks = 12);
/** Unit cube centered at origin with half-extent 0.5 (matches Marker CUBE scale). */
ObjMesh buildCubeMesh();
/** Cone along +Z: base at z=0 (radius), apex at z=height. */
ObjMesh buildConeMesh(float radius, float height, int slices = 16);
/** Capsule along Z: cylinder + hemispherical caps (rviz_capsule.mesh). */
ObjMesh buildCapsuleMesh(float radius, float length, int slices = 16);

}  // namespace display
}  // namespace autoviz
