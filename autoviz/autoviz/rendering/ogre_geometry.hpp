/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file ogre_geometry.hpp
 * @brief Convert @ref SceneOverlay CPU geometry into Ogre-friendly vertex arrays.
 *
 * Used by the Ogre immediate/manual path to mirror OpenGL overlay lines,
 * points, flat triangles, and PBR vertices.
 *
 * @see SceneOverlay
 * @see OgreSceneHost
 * @see OgreRenderBackend
 */

#pragma once

#include <QMatrix4x4>
#include <QVector3D>
#include <QVector4D>
#include <vector>

#include "autoviz/rendering/scene_overlay.hpp"

namespace autoviz {
namespace rendering {

/**
 * @struct OgreLineVertex
 * @brief Colored position vertex for ManualObject line / triangle uploads.
 */
struct OgreLineVertex {
  QVector3D position; /**< World-space position. */
  QVector4D color;    /**< RGBA color. */
};

/**
 * @struct OgrePbrVertex
 * @brief PBR mesh vertex matching @ref SceneOverlay::PbrVertex layout.
 */
struct OgrePbrVertex {
  QVector3D position;           /**< World-space position. */
  QVector3D normal;             /**< Unit normal. */
  QVector4D albedo;             /**< Base color RGBA. */
  float metallic = 0.08f;       /**< Metallic factor. */
  float roughness = 0.52f;      /**< Roughness factor. */
};

/**
 * @brief Appends default ground-grid line segments into @p out.
 * @param[out] out Destination vertex list (appended).
 */
void BuildGroundGridLines(std::vector<OgreLineVertex>* out);

/**
 * @brief Appends RGB origin axis segments into @p out.
 * @param[out] out Destination vertex list (appended).
 */
void BuildOriginAxisLines(std::vector<OgreLineVertex>* out);

/**
 * @brief Copies overlay line vertices into @p out.
 * @param overlay Source overlay.
 * @param[out] out Destination vertex list (appended).
 */
void AppendSceneOverlayLines(const SceneOverlay& overlay,
                             std::vector<OgreLineVertex>* out);

/**
 * @brief Copies overlay point vertices into @p out (as colored positions).
 * @param overlay Source overlay.
 * @param[out] out Destination vertex list (appended).
 */
void AppendSceneOverlayPoints(const SceneOverlay& overlay,
                              std::vector<OgreLineVertex>* out);

/**
 * @brief Copies raw overlay triangle vertices (no billboard expansion).
 * @param overlay Source overlay.
 * @param[out] out Destination vertex list (appended).
 */
void AppendSceneOverlayTriangles(const SceneOverlay& overlay,
                                 std::vector<OgreLineVertex>* out);

/**
 * @brief Copies expanded overlay triangles (legacy; includes billboards).
 * @param overlay Source overlay.
 * @param view View matrix for billboard expansion.
 * @param[out] out Destination vertex list (appended).
 */
void AppendSceneOverlayTriangles(const SceneOverlay& overlay,
                                 const QMatrix4x4& view,
                                 std::vector<OgreLineVertex>* out);

/**
 * @brief Copies flat shaded triangles + view-facing billboards.
 * @param overlay Source overlay.
 * @param view View matrix for expansion.
 * @param[out] out Destination vertex list (appended).
 * @see SceneOverlay::expandedFlatTriangles()
 */
void AppendSceneOverlayFlatTriangles(const SceneOverlay& overlay,
                                     const QMatrix4x4& view,
                                     std::vector<OgreLineVertex>* out);

/**
 * @brief Converts overlay PBR vertices into @ref OgrePbrVertex list.
 * @param vertices Source @ref SceneOverlay::PbrVertex array.
 * @param[out] out Destination PBR vertices.
 */
void BuildPbrMeshFromPbrVertices(
    const std::vector<SceneOverlay::PbrVertex>& vertices,
    std::vector<OgrePbrVertex>* out);

/**
 * @brief @deprecated Flat-shaded overlay triangles only; prefer flat + PBR split.
 *
 * @param triangles Colored triangle vertices.
 * @param[out] out Destination PBR vertices (normals estimated).
 */
void BuildPbrMeshFromTriangles(const std::vector<OgreLineVertex>& triangles,
                               std::vector<OgrePbrVertex>* out);

}  // namespace rendering
}  // namespace autoviz
