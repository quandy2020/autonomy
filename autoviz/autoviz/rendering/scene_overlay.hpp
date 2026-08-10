/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

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

/** Dynamic geometry collected by Display plugins each frame. */
class SceneOverlay {
 public:
  struct PickVertex {
    QVector3D position;
    uint32_t handle = 0;
  };

  struct ColoredVertex {
    QVector3D position;
    QVector4D color;
  };

  struct PickSample {
    QVector3D position;
    std::string display_name;
    std::string display_type;
  };

  struct TexturedVertex {
    QVector3D position;
    QVector2D uv;
    QVector4D color = QVector4D(1.f, 1.f, 1.f, 1.f);
  };

  struct TexturedBatch {
    QImage image;
    std::vector<TexturedVertex> vertices;
  };

  struct PbrVertex {
    QVector3D position;
    QVector3D normal;
    QVector4D albedo;
    float metallic = 0.08f;
    float roughness = 0.52f;
  };

  struct PbrTexturedVertex {
    QVector3D position;
    QVector3D normal;
    QVector2D uv;
    QVector4D tint;
    float metallic = 0.08f;
    float roughness = 0.52f;
  };

  struct PbrTexturedBatch {
    QImage image;
    std::vector<PbrTexturedVertex> vertices;
  };

  void clear();
  void setPickRegistry(common::PickRegistry* registry) {
    pick_registry_ = registry;
  }
  common::PickRegistry* pickRegistry() const { return pick_registry_; }
  void setHandlerManager(common::HandlerManager* manager) {
    handler_manager_ = manager;
  }
  void setPickSource(const std::string* display_name,
                     const std::string* display_type);

  /** @param for_pick When false, skip CPU pick samples (TF/axes overlays). */
  void addLine(const QVector3D& a, const QVector3D& b, const QColor& color,
               bool for_pick = true);
  void addPickPoint(const QVector3D& p, const QColor& color, int point_index,
                    const std::shared_ptr<common::SelectionHandler>& handler =
                        nullptr);
  /** Pick FBO only (no visible point geometry) — use with Ogre PointCloud path. */
  void addPickMarker(
      const QVector3D& p, int point_index,
      const std::shared_ptr<common::SelectionHandler>& handler = nullptr);
  /** Register pick handle without GL pick geometry (Ogre native pick path). */
  common::PickHandle registerPickEntry(
      const QVector3D& position, int point_index,
      const std::shared_ptr<common::SelectionHandler>& handler = nullptr);
  void addEllipsoidWireframe(const QVector3D& center, float radius_x,
                             float radius_y, float radius_z,
                             const QColor& color);
  void addPoint(const QVector3D& p, const QColor& color);
  void addPoints(const std::vector<QVector3D>& points, const QColor& color);
  void addBoxWireframe(const QVector3D& center, const QVector3D& half_extents,
                       const QMatrix4x4& transform, const QColor& color);
  void addBoxSolid(const QVector3D& center, const QVector3D& half_extents,
                   const QMatrix4x4& transform, const QColor& color);
  void addTriangleMeshWireframe(const display::ObjMesh& mesh,
                                const QMatrix4x4& transform,
                                const QColor& color);
  void addTriangleMeshSolid(const display::ObjMesh& mesh,
                            const QMatrix4x4& transform, const QColor& color);
  void addTriangleMeshSolidPbr(const display::ObjMesh& mesh,
                               const QMatrix4x4& transform, const QColor& color,
                               float metallic, float roughness);
  void addTriangleMeshTexturedPbr(const display::ObjMesh& mesh,
                                  const QMatrix4x4& transform, const QImage& image,
                                  const QColor& tint, float metallic,
                                  float roughness);
  void addBoxSolidPbr(const QVector3D& center, const QVector3D& half_extents,
                      const QMatrix4x4& transform, const QColor& color,
                      float metallic, float roughness);
  void addTexturedQuad(const QVector3D& top_left, const QVector3D& top_right,
                       const QVector3D& bottom_right,
                       const QVector3D& bottom_left, const QImage& image);
  /** Screen-aligned quad (TEXT markers, billboards). Expanded at render time. */
  void addViewFacingQuad(const QVector3D& center, float half_extent,
                         const QColor& color);
  /** Camera-facing polyline strip (Path Billboards on OpenGL backend). */
  void addViewFacingPolylineStrip(const std::vector<QVector3D>& points,
                                  float line_width, const QColor& color);
  /** Screen-aligned textured quad (TEXT markers). Expanded at render time. */
  void addViewFacingTexturedQuad(const QVector3D& center, float half_width,
                                 float half_height, const QImage& image);

  void initialize();
  bool isInitialized() const { return initialized_; }
  void render(const QMatrix4x4& view, const QMatrix4x4& projection);
  void render(const CameraState& camera, float aspect_ratio);
  /** RViz-style pick-color pass into the currently bound framebuffer. */
  void renderPickPass(const QMatrix4x4& view, const QMatrix4x4& projection);
  void shutdown();

  const std::vector<ColoredVertex>& lineVertices() const {
    return line_vertices_;
  }
  const std::vector<ColoredVertex>& pointVertices() const {
    return point_vertices_;
  }
  const std::vector<ColoredVertex>& triangleVertices() const {
    return triangle_vertices_;
  }
  const std::vector<PbrVertex>& pbrVertices() const { return pbr_vertices_; }
  const std::vector<PbrTexturedBatch>& pbrTexturedBatches() const {
    return pbr_textured_batches_;
  }
  const std::vector<PickSample>& pickSamples() const { return pick_samples_; }
  /** Colored triangles + view-facing billboards (excludes point sprites). */
  std::vector<ColoredVertex> expandedFlatTriangles(const QMatrix4x4& view) const;
  /** @deprecated Use expandedFlatTriangles; kept for pick helpers. */
  std::vector<ColoredVertex> expandedTriangles(const QMatrix4x4& view) const;
  const std::vector<TexturedBatch>& texturedBatches() const {
    return textured_batches_;
  }
  /** World-space textured batches including view-facing labels. */
  std::vector<TexturedBatch> expandedTexturedBatches(
      const QMatrix4x4& view) const;

  void setPointSize(float size) { point_size_ = std::max(1.f, size); }
  float pointSize() const { return point_size_; }

  bool empty() const {
    return line_vertices_.empty() && point_vertices_.empty() &&
           triangle_vertices_.empty() && textured_batches_.empty() &&
           billboard_requests_.empty() &&
           polyline_strip_requests_.empty() &&
           view_facing_textured_requests_.empty() &&
           pbr_vertices_.empty() && pbr_textured_batches_.empty();
  }

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

  void recordPick(const QVector3D& position);
  void uploadIfNeeded();
  void appendViewFacingBillboards(const QMatrix4x4& view,
                                  std::vector<ColoredVertex>* triangles) const;
  void appendViewFacingPolylineStrips(const QMatrix4x4& view,
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
