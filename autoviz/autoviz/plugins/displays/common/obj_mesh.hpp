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
 * - COLLADA `.dae` (@ref loadMeshFile), including scene node matrices
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
#include <QImage>
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
 * @brief One material range inside an OBJ / COLLADA mesh.
 *
 * Triangles are a slice of @ref ObjMesh::triangles. Prefer @c texture (embedded
 * or decoded) over @c texture_path when non-null. @c texture_path is empty when
 * the material has only a diffuse color (no `map_Kd` / COLLADA image).
 */
struct ObjSubmesh {
  std::string name;
  QColor diffuse{255, 255, 255};
  std::string texture_path;
  QImage texture; /**< Embedded / decoded albedo; takes precedence over path. */
  int triangle_begin = 0;
  int triangle_count = 0;
};

struct ObjMesh {
  std::vector<QVector3D> vertices;           /**< Vertex positions. */
  std::vector<QVector2D> texcoords;          /**< UV coordinates (may be empty). */
  std::vector<QVector3D> normals;            /**< Per-vertex normals (may be empty). */
  /**
   * Optional per-vertex colors (Marker TRIANGLE_LIST). When size matches
   * @ref vertices, solid draw paths prefer these over a flat tint.
   */
  std::vector<QColor> colors;
  std::vector<std::array<int, 3>> triangles; /**< Triangle index triples. */
  /** Filled by @ref loadObjFile / Assimp COLLADA when materials are present. */
  std::vector<ObjSubmesh> submeshes;
  /**
   * Per-submesh geometry copies with stable addresses for Ogre textured links.
   * Parallel to @ref submeshes when @ref has_material_groups is true.
   */
  std::vector<ObjMesh> parts;
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
 * @brief Ensures @c mesh->normals has one entry per vertex (RViz/Assimp parity).
 *
 * Keeps existing normals when already aligned with @c vertices; otherwise
 * builds area-weighted smooth normals from @c triangles.
 *
 * @param mesh Mesh to mutate; no-op if @c nullptr or empty.
 */
void ensureMeshNormals(ObjMesh* mesh);

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
