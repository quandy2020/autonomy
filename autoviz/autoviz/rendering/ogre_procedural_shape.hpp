/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file ogre_procedural_shape.hpp
 * @brief Register unit primitive and ObjMesh resources with Ogre MeshManager.
 *
 * Ensures unit @c aviz_*.mesh primitives exist and can upload
 * @ref display::ObjMesh instances under stable resource names.
 *
 * @see OgreMeshLoader
 * @see OgreShape
 */

#pragma once

#include "autoviz/display/obj_mesh.hpp"

namespace autoviz {
namespace rendering {

/**
 * @brief Registers unit primitive meshes via MeshManager.
 *
 * Creates @c aviz_cone, @c aviz_cube, @c aviz_cylinder, @c aviz_sphere,
 * @c aviz_capsule (and related) if missing. Idempotent.
 *
 * @see OgreShape::createEntity()
 * @see OgreMeshLoader::ensurePrimitiveMeshes()
 */
void ensureAvizPrimitiveMeshes();

/**
 * @brief Uploads an @ref display::ObjMesh as a named Ogre mesh if absent.
 *
 * @param mesh_name MeshManager resource name (stable key).
 * @param mesh Triangle mesh data to upload.
 *
 * @see OgreMeshLoader::registerCachedObjMesh()
 */
void registerObjMesh(const std::string& mesh_name, const display::ObjMesh& mesh);

}  // namespace rendering
}  // namespace autoviz

