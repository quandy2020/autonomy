/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file grid_renderer.hpp
 * @brief OpenGL ground-grid / clear-color renderer for @ref RenderWindow.
 *
 * Draws an optional REP-103 XY reference grid and RGB origin axes into the
 * current GL framebuffer. Used as the clear/background pass before
 * @ref SceneOverlay::render().
 *
 * @see ReferenceGridSettings
 * @see CameraState
 * @see RenderWindow
 * @see OgreGrid
 */

#pragma once

#include <QColor>
#include <QMatrix4x4>
#include <QVector3D>

#include "autoviz/rendering/render_settings.hpp"

namespace autoviz {
namespace rendering {

/**
 * @struct CameraState
 * @brief Lightweight orbit camera used by standalone grid preview paths.
 *
 * Prefer @ref ViewController for interactive viewports; this struct remains
 * for simple @ref GridRenderer::render(const CameraState&) callers.
 */
struct CameraState {
  float yaw = 0.785f;                 /**< Yaw about Z (rad). */
  float pitch = 0.5f;                 /**< Pitch (rad). */
  float distance = 15.0f;             /**< Distance from target. */
  QVector3D target{0.f, 0.f, 0.f};    /**< Look-at point. */

  /**
   * @brief Builds a view matrix from yaw/pitch/distance/target.
   * @return 4×4 view matrix.
   */
  QMatrix4x4 viewMatrix() const;

  /**
   * @brief Builds a perspective projection matrix.
   * @param aspect_ratio Width / height.
   * @return 4×4 projection matrix.
   */
  QMatrix4x4 projectionMatrix(float aspect_ratio) const;
};

/**
 * @class GridRenderer
 * @brief OpenGL program that clears the background and optionally draws a grid.
 *
 * ## Lifecycle
 *
 * Call @ref initialize() with a current GL context, @ref resize() on viewport
 * changes, @ref render() each frame, and @ref shutdown() before context loss.
 *
 * @note Grid visibility defaults to @c false (RViz2 uses a Grid Display).
 */
class GridRenderer {
 public:
  /**
   * @brief Compiles shaders and builds initial geometry (requires GL context).
   */
  void initialize();

  /**
   * @brief Records viewport dimensions for aspect and scissor.
   * @param width Viewport width in pixels.
   * @param height Viewport height in pixels.
   */
  void resize(int width, int height);

  /**
   * @brief Clears to @ref backgroundColor() and draws the grid when visible.
   *
   * @param view Camera view matrix.
   * @param projection Camera projection matrix.
   */
  void render(const QMatrix4x4& view, const QMatrix4x4& projection);

  /**
   * @brief Convenience overload using @ref CameraState matrices.
   * @param camera Orbit camera state.
   */
  void render(const CameraState& camera);

  /**
   * @brief Releases GL programs / VAOs / VBOs.
   */
  void shutdown();

  /**
   * @brief Shows or hides the reference grid (background clear always runs).
   * @param visible Grid visibility.
   */
  void setVisible(bool visible) { visible_ = visible; }

  /**
   * @brief Whether the reference grid is drawn.
   * @return Current visibility flag.
   */
  bool visible() const { return visible_; }

  /**
   * @brief Sets the clear / background color.
   * @param color RGB(A) clear color.
   */
  void setBackgroundColor(const QColor& color);

  /**
   * @brief Returns the current background color.
   * @return Clear color.
   */
  QColor backgroundColor() const { return background_color_; }

  /**
   * @brief Updates grid cell count / length / color / axes and rebuilds geometry.
   * @param settings Reference grid configuration.
   * @see ReferenceGridSettings
   */
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
  bool visible_ = false;
  QColor background_color_{48, 48, 48};
};

}  // namespace rendering
}  // namespace autoviz
