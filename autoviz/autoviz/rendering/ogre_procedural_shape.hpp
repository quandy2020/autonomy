/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#ifdef AUTOVIZ_USE_OGRE

#include "autoviz/display/obj_mesh.hpp"

namespace autoviz {
namespace rendering {

/** Register rviz-compatible unit primitive meshes (rviz_*.mesh) via MeshManager. */
void ensureRvizPrimitiveMeshes();

/** Upload ObjMesh as a named Ogre mesh if not already present. */
void registerObjMesh(const std::string& mesh_name, const display::ObjMesh& mesh);

}  // namespace rendering
}  // namespace autoviz

#endif
