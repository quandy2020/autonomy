/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#ifdef AUTOVIZ_USE_OGRE

#include <string>

#include <OgreMesh.h>

namespace autoviz {
namespace display {
struct ObjMesh;
}  // namespace display

namespace rendering {

/** rviz_rendering::mesh_loader + ObjMesh cache (Assimp / Ogre .mesh / OBJ / STL). */
class OgreMeshLoader {
 public:
  static void ensurePrimitiveMeshes();

  /** Known rviz_*.mesh name for unit primitives, or empty if not a primitive. */
  static std::string primitiveMeshName(const display::ObjMesh& mesh);

  /** Register mesh under a stable hash name; returns MeshManager resource name. */
  static std::string registerCachedObjMesh(const display::ObjMesh& mesh);

  /** Load OBJ/STL from disk and register; returns empty on failure. */
  static std::string loadAndRegisterMeshFile(const std::string& path);

  /** rviz loadMeshFromResource equivalent (package:// / file:// / absolute). */
  static Ogre::MeshPtr loadMeshFromResource(const std::string& resource_uri);
};

}  // namespace rendering
}  // namespace autoviz

#endif
