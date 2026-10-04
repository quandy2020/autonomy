/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file primitive_mesh.hpp
 * @brief Procedural generators for common solid meshes as @ref ObjMesh.
 *
 * Builds unit / parameterized primitives used by markers, range cones,
 * robot collision shapes, and arrow heads — coordinates in local mesh space.
 *
 * @see ObjMesh
 * @see buildCylinderMesh()
 * @see buildConeMesh()
 * @see RobotModelDisplay
 */

#pragma once

#include "autoviz/display/obj_mesh.hpp"

namespace autoviz {
namespace display {

/**
 * @brief Builds a cylinder mesh along +Z centered on the origin.
 *
 * @param radius Cylinder radius.
 * @param length Cylinder height along Z.
 * @param slices Circumferential tessellation count.
 * @return Indexed triangle mesh.
 */
ObjMesh buildCylinderMesh(float radius, float length, int slices = 16);

/**
 * @brief Builds a UV-sphere mesh centered on the origin.
 *
 * @param radius Sphere radius.
 * @param slices Longitude subdivisions.
 * @param stacks Latitude subdivisions.
 * @return Indexed triangle mesh.
 */
ObjMesh buildSphereMesh(float radius, int slices = 16, int stacks = 12);

/**
 * @brief Builds a unit cube centered at the origin with half-extent 0.5.
 *
 * Matches @c visualization_msgs/Marker CUBE scale semantics (edge length =
 * scale after transform).
 *
 * @return Indexed triangle mesh.
 */
ObjMesh buildCubeMesh();

/**
 * @brief Builds a cone along +Z: base at z=0 (radius), apex at z=@p height.
 *
 * @param radius Base radius at z = 0.
 * @param height Apex height along +Z.
 * @param slices Circumferential tessellation count.
 * @return Indexed triangle mesh.
 */
ObjMesh buildConeMesh(float radius, float height, int slices = 16);

/**
 * @brief Builds a capsule along Z (cylinder + hemispherical caps).
 *
 * Matches @c aviz_capsule.mesh proportions for collision/visual capsules.
 *
 * @param radius Capsule radius (cylinder and caps).
 * @param length Cylinder section length (excluding caps).
 * @param slices Circumferential tessellation count.
 * @return Indexed triangle mesh.
 */
ObjMesh buildCapsuleMesh(float radius, float length, int slices = 16);

}  // namespace display
}  // namespace autoviz
