/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file ogre_mesh_loader.hpp
 * @brief Load and cache meshes for Ogre (Assimp / .mesh / OBJ / STL).
 *
 * Combines rviz_rendering @c mesh_loader behavior with an @ref display::ObjMesh
 * hash cache used by Entity displays and robot models.
 *
 * @see MeshResourceResolver
 * @see ensureAvizPrimitiveMeshes()
 * @see OgreShape
 */

#pragma once

#include <string>

#include <OgreMesh.h>

namespace autoviz {
namespace display {
struct ObjMesh;
}  // namespace display

namespace rendering {

/**
 * @class OgreMeshLoader
 * @brief Static API for primitive meshes, ObjMesh cache, and URI loading.
 */
class OgreMeshLoader {
 public:
  /**
   * @brief Ensures unit @c aviz_*.mesh primitives are registered.
   * @see ensureAvizPrimitiveMeshes()
   */
  static void ensurePrimitiveMeshes();

  /**
   * @brief Returns the known @c aviz_*.mesh name for a unit primitive ObjMesh.
   *
   * @param mesh Candidate mesh (e.g. unit cube / sphere generated in-process).
   * @return MeshManager name, or empty string if not a known primitive.
   */
  static std::string primitiveMeshName(const display::ObjMesh& mesh);

  /**
   * @brief Registers @p mesh under a stable content-hash name.
   *
   * @param mesh Triangle mesh to upload.
   * @return MeshManager resource name (existing or newly created).
   */
  static std::string registerCachedObjMesh(const display::ObjMesh& mesh);

  /**
   * @brief Loads OBJ/STL from disk and registers it with MeshManager.
   *
   * @param path Absolute filesystem path.
   * @return Mesh resource name, or empty string on failure.
   */
  static std::string loadAndRegisterMeshFile(const std::string& path);

  /**
   * @brief rviz @c loadMeshFromResource equivalent.
   *
   * Accepts @c package://, @c file://, and absolute paths via
   * @ref MeshResourceResolver.
   *
   * @param resource_uri Mesh URI.
   * @return Ogre mesh pointer, or null on failure.
   */
  static Ogre::MeshPtr loadMeshFromResource(const std::string& resource_uri);
};

}  // namespace rendering
}  // namespace autoviz

