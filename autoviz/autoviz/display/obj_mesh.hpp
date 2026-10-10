/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file obj_mesh.hpp
 * @brief Triangle-mesh container and loaders for OBJ / STL / Marker mesh data.
 *
 * @ref ObjMesh is the shared CPU-side geometry type used by marker drawing,
 * robot-model visuals, and the Ogre/GL mesh draw helpers
 * (@ref ogre_mesh_draw.hpp, @ref ogre_pbr_mesh_draw.hpp, @ref primitive_mesh.hpp).
 *
 * ## Supported inputs
 *
 * - Wavefront OBJ text (@ref parseObjText / @ref loadObjFile)
 * - STL binary and ASCII (@ref parseStlBinary / @ref parseStlAscii /
 *   @ref loadStlFile)
 * - Auto-detect by content or extension (@ref parseMeshData / @ref loadMeshFile)
 * - @c visualization_msgs/Marker mesh resource or embedded data
 *   (@ref parseMarkerMesh)
 *
 * @see primitive_mesh.hpp
 * @see ogre_mesh_draw.hpp
 */

#pragma once

#include <array>
#include <string>
#include <vector>

#include <QColor>
#include <QVector2D>
#include <QVector3D>

#include <automsgs/msgs/visualization_msgs/marker.pb.h>

namespace autoviz {
namespace display {

/**
 * @struct ObjMesh
 * @brief Indexed triangle mesh with optional per-vertex texture coordinates.
 *
 * Vertices and texcoords are parallel arrays (same length after a successful
 * parse or @ref ensureMeshTexcoords). Triangles store 0-based vertex indices.
 *
 * @note Coordinates are in the mesh's local frame; callers apply
 *       @c QMatrix4x4 transforms when drawing.
 */
/**
 * @brief One `usemtl` range inside an OBJ.
 *
 * Triangles are a slice of @ref ObjMesh::triangles. @c texture_path is empty
 * when the MTL material has only a diffuse color (no `map_Kd`).
 */
struct ObjSubmesh {
  std::string name;
  QColor diffuse{255, 255, 255};
  std::string texture_path;
  int triangle_begin = 0;
  int triangle_count = 0;
};

struct ObjMesh {
  std::vector<QVector3D> vertices;           /**< Vertex positions. */
  std::vector<QVector2D> texcoords;          /**< UV coordinates (may be empty). */
  std::vector<std::array<int, 3>> triangles; /**< Triangle index triples. */
  /** Filled by @ref loadObjFile when the OBJ references an MTL library. */
  std::vector<ObjSubmesh> submeshes;
  bool has_material_groups = false;
};

/**
 * @brief Fills missing per-vertex UVs with axis-aligned box projection.
 *
 * When @c mesh->texcoords is empty or shorter than @c vertices, writes a UV
 * for every vertex so textured / PBR paths can sample without crashing.
 *
 * @param mesh Mesh to mutate; no-op if @c nullptr.
 */
void ensureMeshTexcoords(ObjMesh* mesh);

/**
 * @brief Parses Wavefront OBJ text into an @ref ObjMesh.
 *
 * @param text OBJ file contents.
 * @param mesh Output mesh; cleared on entry when non-null.
 * @return @c true on success with at least one triangle.
 */
bool parseObjText(const std::string& text, ObjMesh* mesh);

/**
 * @brief Parses a binary STL blob into an @ref ObjMesh.
 *
 * @param data Raw binary STL bytes (80-byte header + triangle records).
 * @param mesh Output mesh; cleared on entry when non-null.
 * @return @c true on success.
 */
bool parseStlBinary(const std::string& data, ObjMesh* mesh);

/**
 * @brief Parses an ASCII STL document into an @ref ObjMesh.
 *
 * @param text ASCII STL text (`solid` … `endsolid`).
 * @param mesh Output mesh; cleared on entry when non-null.
 * @return @c true on success.
 */
bool parseStlAscii(const std::string& text, ObjMesh* mesh);

/**
 * @brief Auto-detects OBJ vs STL from in-memory bytes and parses.
 *
 * @param data Mesh payload (text or binary).
 * @param mesh Output mesh.
 * @return @c true if any supported parser succeeds.
 * @see parseObjText()
 * @see parseStlBinary()
 * @see parseStlAscii()
 */
bool parseMeshData(const std::string& data, ObjMesh* mesh);

/**
 * @brief Loads an OBJ file from disk.
 *
 * @param path Filesystem path to a `.obj` file.
 * @param mesh Output mesh.
 * @return @c true on successful read and parse.
 */
bool loadObjFile(const std::string& path, ObjMesh* mesh);

/**
 * @brief Loads an STL file from disk (binary or ASCII).
 *
 * @param path Filesystem path to a `.stl` file.
 * @param mesh Output mesh.
 * @return @c true on successful read and parse.
 */
bool loadStlFile(const std::string& path, ObjMesh* mesh);

/**
 * @brief Loads a mesh file by extension / content sniffing.
 *
 * @param path Filesystem path (typically `.obj` or `.stl`).
 * @param mesh Output mesh.
 * @return @c true on successful load.
 * @see loadObjFile()
 * @see loadStlFile()
 */
bool loadMeshFile(const std::string& path, ObjMesh* mesh);

/**
 * @brief Extracts mesh geometry from a @c visualization_msgs/Marker.
 *
 * Resolves resource URI / embedded mesh fields used by MESH_RESOURCE markers
 * into an @ref ObjMesh for drawing.
 *
 * @param marker Marker protobuf (mesh resource or data).
 * @param mesh Output mesh.
 * @return @c true when geometry was obtained.
 * @see MarkerDisplay
 */
bool parseMarkerMesh(
    const automsgs::msgs::visualization_msgs::Marker& marker,
    ObjMesh* mesh);

}  // namespace display
}  // namespace autoviz
