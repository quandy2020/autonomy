/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QMatrix4x4>
#include <QVector3D>

#include "autoviz/rendering/render_settings.hpp"

namespace autoviz {
namespace rendering {

struct CameraState {
  float yaw = 0.785f;
  float pitch = 0.5f;
  float distance = 15.0f;
  QVector3D target{0.f, 0.f, 0.f};

  QMatrix4x4 viewMatrix() const;
  QMatrix4x4 projectionMatrix(float aspect_ratio) const;
};

class GridRenderer {
 public:
  void initialize();
  void resize(int width, int height);
  void render(const QMatrix4x4& view, const QMatrix4x4& projection);
  void render(const CameraState& camera);
  void shutdown();

  void setVisible(bool visible) { visible_ = visible; }
  bool visible() const { return visible_; }

  void setBackgroundColor(const QColor& color);
  QColor backgroundColor() const { return background_color_; }

  void setReferenceGridSettings(const ReferenceGridSettings& settings);

 private:
  void ensureProgram();
  void buildGridGeometry();

  ReferenceGridSettings grid_settings_;

  int grid_program_ = 0;
  int axis_program_ = 0;
  int grid_vao_ = 0;
  int grid_vbo_ = 0;
  int grid_vertex_count_ = 0;
  int axis_vao_ = 0;
  int axis_vbo_ = 0;
  int viewport_width_ = 1;
  int viewport_height_ = 1;
  bool initialized_ = false;
  bool visible_ = true;
  QColor background_color_{48, 48, 48};
};

}  // namespace rendering
}  // namespace autoviz
