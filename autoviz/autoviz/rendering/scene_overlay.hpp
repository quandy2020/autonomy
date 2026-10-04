/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file scene_overlay.hpp
 * @brief Per-frame dynamic geometry collected by Display plugins (OpenGL path).
 *
 * Displays append lines, points, meshes, PBR batches, and pick markers each
 * update cycle. @ref RenderWindow (and optionally Ogre) draws the overlay after
 * the clear/grid pass. Also drives the pick-color pass and CPU pick samples.
 *
 * @see GridRenderer
 * @see pickNearestScenePoint()
 * @see GlPickFramebuffer
 * @see OgreSceneHost
 */

#pragma once

#include <string>
#include <vector>

#include <cstdint>

#include <algorithm>

#include <memory>

#include <QImage>
#include <QMatrix4x4>
#include <QVector2D>
#include <QVector3D>
#include <QVector4D>

#include "autoviz/common/selection_handler.hpp"
#include "autoviz/rendering/grid_renderer.hpp"

#include "autoviz/display/obj_mesh.hpp"

namespace autoviz {
namespace common {
class HandlerManager;
class PickRegistry;
}

namespace rendering {

/**
 * @class SceneOverlay
 * @brief Dynamic geometry buffer + OpenGL renderer for display overlays.
 *
 * ## Typical frame
 *
 * 1. @ref clear() at the start of the display update.
 * 2. Displays call @c add* helpers (optionally @ref setPickSource()).
 * 3. Viewport calls @ref render() then optionally @ref renderPickPass().
 *
 * ## Pick paths
 *
 * - CPU: @ref pickSamples() / expanded triangles via @ref pickNearestScenePoint().
 * - GPU color: @ref addPickPoint() / @ref addPickMarker() + @ref renderPickPass().
 * - Ogre-native: @ref registerPickEntry() without GL pick geometry.
 */
class SceneOverlay {
 public:
  /**
   * @enum TextureFilterMode
   * @brief Sampling filter for textured batches.
   */
  enum class TextureFilterMode {
    kLinear,  /**< Bilinear filtering. */
    kNearest  /**< Nearest-neighbor (pixel art / indexed maps). */
  };

  /**
   * @struct PickVertex
   * @brief Vertex for the GPU pick-color pass (position + packed handle).
   */
  struct PickVertex {
    QVector3D position; /**< World position. */
    uint32_t handle = 0; /**< Encoded pick handle bits. */
  };

  /**
   * @struct ColoredVertex
   * @brief Position + RGBA for lines, points, and flat triangles.
   */
  struct ColoredVertex {
    QVector3D position; /**< World position. */
    QVector4D color;    /**< RGBA. */
  };

  /**
   * @struct PickSample
   * @brief Tagged CPU pick sample (nearest-pixel search).
   */
  struct PickSample {
    QVector3D position;       /**< World sample position. */
    std::string display_name; /**< Display instance name. */
    std::string display_type; /**< Display type id. */
  };

  /**
   * @struct TexturedVertex
   * @brief Textured triangle vertex with optional tint.
   */
  struct TexturedVertex {
    QVector3D position; /**< World position. */
    QVector2D uv;       /**< Texture coordinates. */
    QVector4D color = QVector4D(1.f, 1.f, 1.f, 1.f); /**< Vertex tint. */
  };

  /**
   * @struct TexturedBatch
   * @brief One texture + triangle list sharing that image.
   */
  struct TexturedBatch {
    QImage image; /**< Source texture (CPU). */
    std::vector<TexturedVertex> vertices; /**< Triangle vertices. */
    TextureFilterMode filter_mode = TextureFilterMode::kLinear; /**< Filter. */
  };

  /**
   * @struct PbrVertex
   * @brief Untextured PBR triangle vertex.
   */
  struct PbrVertex {
    QVector3D position;      /**< World position. */
    QVector3D normal;        /**< Unit normal. */
    QVector4D albedo;        /**< Base color. */
    float metallic = 0.08f;  /**< Metallic. */
    float roughness = 0.52f; /**< Roughness. */
  };

  /**
   * @struct PbrTexturedVertex
   * @brief Textured PBR triangle vertex.
   */
  struct PbrTexturedVertex {
    QVector3D position;      /**< World position. */
    QVector3D normal;        /**< Unit normal. */
    QVector2D uv;            /**< Texture UV. */
    QVector4D tint;          /**< Color tint. */
    float metallic = 0.08f;  /**< Metallic. */
    float roughness = 0.52f; /**< Roughness. */
  };

  /**
   * @struct PbrTexturedBatch
   * @brief Textured PBR vertices sharing one image.
   */
  struct PbrTexturedBatch {
    QImage image; /**< Albedo texture. */
    std::vector<PbrTexturedVertex> vertices; /**< Triangle vertices. */
  };

  /**
   * @brief Clears all geometry and pick samples for a new frame.
   */
  void clear();

  /**
   * @brief Attaches the pick registry used when allocating handles.
   * @param registry Non-owning; may be @c nullptr.
   */
  void setPickRegistry(common::PickRegistry* registry) {
    pick_registry_ = registry;
  }

  /**
   * @brief Returns the attached pick registry.
   * @return Registry pointer or @c nullptr.
   */
  common::PickRegistry* pickRegistry() const { return pick_registry_; }

  /**
   * @brief Attaches the selection handler manager.
   * @param manager Non-owning; may be @c nullptr.
   */
  void setHandlerManager(common::HandlerManager* manager) {
    handler_manager_ = manager;
  }

  /**
   * @brief Sets the display name/type stamped onto subsequent pick samples.
   *
   * @param display_name Pointer to name string (copied when recording); may be null.
   * @param display_type Pointer to type string; may be null.
   */
  void setPickSource(const std::string* display_name,
                     const std::string* display_type);

  /**
   * @brief Appends a colored line segment.
   *
   * @param a Segment start.
   * @param b Segment end.
   * @param color Line color.
   * @param for_pick When @c false, skip CPU pick samples (TF/axes overlays).
   */
  void addLine(const QVector3D& a, const QVector3D& b, const QColor& color,
               bool for_pick = true);

  /**
   * @brief Adds a visible point and registers it for GPU/CPU picking.
   *
   * @param p World position.
   * @param color Point color.
   * @param point_index Index within the display's point set.
   * @param handler Optional selection handler for property expansion.
   */
  void addPickPoint(const QVector3D& p, const QColor& color, int point_index,
                    const std::shared_ptr<common::SelectionHandler>& handler =
                        nullptr);

  /**
   * @brief Pick FBO only (no visible point geometry) — use with Ogre PointCloud.
   *
   * @param p World position.
   * @param point_index Point index.
   * @param handler Optional selection handler.
   */
  void addPickMarker(
      const QVector3D& p, int point_index,
      const std::shared_ptr<common::SelectionHandler>& handler = nullptr);

  /**
   * @brief Registers a pick handle without GL pick geometry (Ogre native path).
   *
   * @param position World position stored in the registry.
   * @param point_index Point index.
   * @param handler Optional selection handler.
   * @return Allocated @ref common::PickHandle.
   */
  common::PickHandle registerPickEntry(
      const QVector3D& position, int point_index,
      const std::shared_ptr<common::SelectionHandler>& handler = nullptr);

  /**
   * @brief Appends an ellipsoid wireframe (axis-aligned radii).
   */
  void addEllipsoidWireframe(const QVector3D& center, float radius_x,
                             float radius_y, float radius_z,
                             const QColor& color);

  /**
   * @brief Appends a single colored point sprite vertex.
   */
  void addPoint(const QVector3D& p, const QColor& color);

  /**
   * @brief Appends many points with a uniform color.
   */
  void addPoints(const std::vector<QVector3D>& points, const QColor& color);

  /**
   * @brief Appends an oriented box wireframe.
   */
  void addBoxWireframe(const QVector3D& center, const QVector3D& half_extents,
                       const QMatrix4x4& transform, const QColor& color);

  /**
   * @brief Appends an oriented solid box (flat shaded triangles).
   */
  void addBoxSolid(const QVector3D& center, const QVector3D& half_extents,
                   const QMatrix4x4& transform, const QColor& color);

  /**
   * @brief Per-vertex colored triangle (grid_map elevation mesh).
   */
  void addColoredTriangle(const QVector3D& a, const QVector3D& b,
                          const QVector3D& c, const QColor& ca, const QColor& cb,
                          const QColor& cc);

  /**
   * @brief Appends an ObjMesh as wireframe lines under @p transform.
   */
  void addTriangleMeshWireframe(const display::ObjMesh& mesh,
                                const QMatrix4x4& transform,
                                const QColor& color);

  /**
   * @brief Appends an ObjMesh as flat solid triangles.
   */
  void addTriangleMeshSolid(const display::ObjMesh& mesh,
                            const QMatrix4x4& transform, const QColor& color);

  /**
   * @brief Appends an ObjMesh as untextured PBR triangles.
   */
  void addTriangleMeshSolidPbr(const display::ObjMesh& mesh,
                               const QMatrix4x4& transform, const QColor& color,
                               float metallic, float roughness);

  /**
   * @brief Appends an ObjMesh as textured PBR triangles.
   */
  void addTriangleMeshTexturedPbr(const display::ObjMesh& mesh,
                                  const QMatrix4x4& transform, const QImage& image,
                                  const QColor& tint, float metallic,
                                  float roughness);

  /**
   * @brief Appends a solid box with PBR shading.
   */
  void addBoxSolidPbr(const QVector3D& center, const QVector3D& half_extents,
                      const QMatrix4x4& transform, const QColor& color,
                      float metallic, float roughness);

  /**
   * @brief Appends a textured quad (two triangles).
   */
  void addTexturedQuad(const QVector3D& top_left, const QVector3D& top_right,
                       const QVector3D& bottom_right,
                       const QVector3D& bottom_left, const QImage& image,
                       TextureFilterMode filter_mode = TextureFilterMode::kLinear);

  /**
   * @brief Screen-aligned quad (TEXT markers, billboards). Expanded at render time.
   */
  void addViewFacingQuad(const QVector3D& center, float half_extent,
                         const QColor& color);

  /**
   * @brief Camera-facing polyline strip (rviz Path Billboards / BillboardChain).
   *
   * @param points Polyline vertices.
   * @param line_width Width in world meters; expanded each frame to face the camera.
   * @param color Strip color.
   */
  void addViewFacingPolylineStrip(const std::vector<QVector3D>& points,
                                  float line_width, const QColor& color);

  /**
   * @brief Screen-space wide closed loop using GL line width.
   */
  void addLineLoop(const std::vector<QVector3D>& points, const QColor& color,
                   float line_width);

  /**
   * @brief Screen-aligned textured quad (TEXT markers). Expanded at render time.
   */
  void addViewFacingTexturedQuad(const QVector3D& center, float half_width,
                                 float half_height, const QImage& image);

  /**
   * @brief Compiles GL programs and creates VAOs (requires current context).
   */
  void initialize();

  /**
   * @brief Whether @ref initialize() has completed.
   * @return @c true if GL resources exist.
   */
  bool isInitialized() const { return initialized_; }

  /**
   * @brief Draws all overlay geometry with the given matrices.
   */
  void render(const QMatrix4x4& view, const QMatrix4x4& projection);

  /**
   * @brief Convenience render using @ref CameraState.
   */
  void render(const CameraState& camera, float aspect_ratio);

  /**
   * @brief RViz-style pick-color pass into the currently bound framebuffer.
   */
  void renderPickPass(const QMatrix4x4& view, const QMatrix4x4& projection);

  /**
   * @brief Releases GL programs and buffers.
   */
  void shutdown();

  /** @brief Line vertex buffer (read-only). */
  const std::vector<ColoredVertex>& lineVertices() const {
    return line_vertices_;
  }

  /** @brief Point vertex buffer (read-only). */
  const std::vector<ColoredVertex>& pointVertices() const {
    return point_vertices_;
  }

  /** @brief Flat triangle vertex buffer (read-only). */
  const std::vector<ColoredVertex>& triangleVertices() const {
    return triangle_vertices_;
  }

  /** @brief Untextured PBR vertices (read-only). */
  const std::vector<PbrVertex>& pbrVertices() const { return pbr_vertices_; }

  /** @brief Textured PBR batches (read-only). */
  const std::vector<PbrTexturedBatch>& pbrTexturedBatches() const {
    return pbr_textured_batches_;
  }

  /** @brief CPU pick samples (read-only). */
  const std::vector<PickSample>& pickSamples() const { return pick_samples_; }

  /**
   * @brief Colored triangles + view-facing billboards (excludes point sprites).
   * @param view View matrix for billboard expansion.
   * @return Expanded triangle list.
   */
  std::vector<ColoredVertex> expandedFlatTriangles(const QMatrix4x4& view) const;

  /**
   * @brief @deprecated Use @ref expandedFlatTriangles; kept for pick helpers.
   */
  std::vector<ColoredVertex> expandedTriangles(const QMatrix4x4& view) const;

  /** @brief Textured batches before view-facing expansion. */
  const std::vector<TexturedBatch>& texturedBatches() const {
    return textured_batches_;
  }

  /**
   * @brief World-space textured batches including view-facing labels.
   * @param view View matrix for expansion.
   * @return Expanded batch list.
   */
  std::vector<TexturedBatch> expandedTexturedBatches(
      const QMatrix4x4& view) const;

  /**
   * @brief Sets GL point size (clamped to ≥ 1).
   * @param size Point size in pixels.
   */
  void setPointSize(float size) { point_size_ = std::max(1.f, size); }

  /**
   * @brief Returns the current point size.
   * @return Point size in pixels.
   */
  float pointSize() const { return point_size_; }

  /**
   * @brief Whether all geometry buffers and deferred requests are empty.
   * @return @c true if nothing to draw.
   */
  bool empty() const {
    return line_vertices_.empty() && point_vertices_.empty() &&
           triangle_vertices_.empty() && textured_batches_.empty() &&
           billboard_requests_.empty() &&
           polyline_strip_requests_.empty() &&
           wide_line_loop_requests_.empty() &&
           view_facing_textured_requests_.empty() &&
           pbr_vertices_.empty() && pbr_textured_batches_.empty();
  }

  /**
   * @brief Whether any GPU pick-point geometry exists.
   * @return @c true if pick FBO has content.
   */
  bool hasPickGeometry() const { return !pick_point_vertices_.empty(); }

 private:
  struct BillboardRequest {
    QVector3D center;
    float half_extent = 0.1f;
    QVector4D color;
  };

  struct ViewFacingTexturedRequest {
    QVector3D center;
    float half_width = 0.1f;
    float half_height = 0.1f;
    QImage image;
  };

  struct PolylineStripRequest {
    std::vector<QVector3D> points;
    float half_width = 0.04f;
    QVector4D color;
  };

  struct WideLineLoopRequest {
    std::vector<QVector3D> points;
    float line_width = 1.f;
    QVector4D color;
  };

  void recordPick(const QVector3D& position);
  void uploadIfNeeded();
  void appendViewFacingBillboards(const QMatrix4x4& view,
                                  std::vector<ColoredVertex>* triangles) const;
  void appendViewFacingPolylineStrips(const QMatrix4x4& view,
                                      std::vector<ColoredVertex>* triangles) const;
  void appendWideLineLoops(const QMatrix4x4& view,
                           const QMatrix4x4& projection, int viewport_height,
                           std::vector<ColoredVertex>* triangles) const;
  void appendPointSpriteBatch(const QMatrix4x4& view,
                              std::vector<TexturedBatch>* batches) const;
  void appendPickPointSpriteBatch(const QMatrix4x4& view,
                                  std::vector<PickVertex>* triangles) const;
  void uploadPickVerticesIfNeeded();
  static QImage pointDiscImage();
  void renderTexturedBatches(const QMatrix4x4& view,
                             const QMatrix4x4& mvp);
  void renderPbrMesh(const QMatrix4x4& mvp, const QMatrix4x4& view);
  void renderPbrTexturedMeshes(const QMatrix4x4& mvp, const QMatrix4x4& view);
  PbrTexturedBatch* findOrCreatePbrTexturedBatch(const QImage& image);

  std::string pick_display_name_;
  std::string pick_display_type_;
  bool has_pick_display_name_ = false;
  bool has_pick_display_type_ = false;
  common::PickRegistry* pick_registry_ = nullptr;
  common::HandlerManager* handler_manager_ = nullptr;
  std::vector<ColoredVertex> line_vertices_;
  std::vector<ColoredVertex> point_vertices_;
  std::vector<PickVertex> pick_point_vertices_;
  std::vector<ColoredVertex> triangle_vertices_;
  std::vector<PickSample> pick_samples_;
  std::vector<TexturedBatch> textured_batches_;
  std::vector<BillboardRequest> billboard_requests_;
  std::vector<PolylineStripRequest> polyline_strip_requests_;
  std::vector<WideLineLoopRequest> wide_line_loop_requests_;
  std::vector<ViewFacingTexturedRequest> view_facing_textured_requests_;
  std::vector<PbrVertex> pbr_vertices_;
  std::vector<PbrTexturedBatch> pbr_textured_batches_;
  int line_program_ = 0;
  int point_program_ = 0;
  int triangle_program_ = 0;
  int textured_program_ = 0;
  int pbr_program_ = 0;
  int pbr_textured_program_ = 0;
  int pick_program_ = 0;
  int line_vao_ = 0;
  int line_vbo_ = 0;
  int point_vao_ = 0;
  int point_vbo_ = 0;
  int triangle_vao_ = 0;
  int triangle_vbo_ = 0;
  int textured_vao_ = 0;
  int textured_vbo_ = 0;
  int pbr_vao_ = 0;
  int pbr_vbo_ = 0;
  int pbr_textured_vao_ = 0;
  int pbr_textured_vbo_ = 0;
  int pick_vao_ = 0;
  int pick_vbo_ = 0;
  bool line_dirty_ = false;
  bool point_dirty_ = false;
  bool triangle_dirty_ = false;
  bool textured_dirty_ = false;
  bool pbr_dirty_ = true;
  bool pick_point_dirty_ = true;
  bool initialized_ = false;
  float point_size_ = 4.f;
};

}  // namespace rendering
}  // namespace autoviz
