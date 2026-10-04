/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file pick_utils.hpp
 * @brief CPU / GPU scene picking helpers for tools and selection.
 *
 * Resolves the nearest pickable sample or geometry under a pixel, optionally
 * preferring a GPU depth hit when available. Results feed Select, Interact,
 * and property inspectors.
 *
 * @see PickResult
 * @see SceneOverlay
 * @see pickWorldPointFromDepthBuffer()
 * @see common::ToolContext
 */

#pragma once

#include <functional>
#include <string>

#include <QMatrix4x4>
#include <QVector3D>

#include "autoviz/rendering/scene_overlay.hpp"

namespace autoviz {
namespace common {
struct ToolContext;
}

namespace rendering {

/**
 * @struct PickResult
 * @brief Outcome of a screen-space scene pick.
 *
 * When @ref hit is @c true, @ref position is the world hit; optional metadata
 * (@ref display_name, @ref pick_handle, @ref properties) describe the object.
 */
struct PickResult {
  /** @brief @c true when a sample or depth hit was found within range. */
  bool hit = false;

  /** @brief World-space hit position. */
  QVector3D position;

  /** @brief Display instance name that contributed the hit (may be empty). */
  std::string display_name;

  /** @brief Display type / class id (may be empty). */
  std::string display_type;

  /** @brief Encoded pick handle, or @ref common::kInvalidPickHandle. */
  common::PickHandle pick_handle = common::kInvalidPickHandle;

  /** @brief Point index within a cloud / set (−1 if not applicable). */
  int point_index = -1;

  /** @brief Key/value properties for the inspector (handler-provided). */
  std::vector<std::pair<std::string, std::string>> properties;

  /** @brief Screen-space distance from cursor to the sample (pixels). */
  float pixel_distance = 0.f;

  /** @brief Depth from the eye / camera along the view ray. */
  float eye_depth = 0.f;

  /** @brief @c true when the position came from GPU depth rather than CPU. */
  bool used_gpu_depth = false;
};

/**
 * @brief CPU screen-space pick: tagged samples first, then expanded geometry.
 *
 * @param overlay Scene overlay with pick samples / geometry.
 * @param view Camera view matrix.
 * @param projection Camera projection matrix.
 * @param viewport_width Viewport width in pixels.
 * @param viewport_height Viewport height in pixels.
 * @param pixel_x Cursor X.
 * @param pixel_y Cursor Y.
 * @param max_pixel_distance Maximum screen distance to accept a sample.
 * @return Best @ref PickResult within range ( @c hit may still be @c false ).
 *
 * @see pickScenePoint()
 */
PickResult pickNearestScenePoint(const SceneOverlay& overlay,
                                 const QMatrix4x4& view,
                                 const QMatrix4x4& projection, int viewport_width,
                                 int viewport_height, int pixel_x, int pixel_y,
                                 float max_pixel_distance = 14.f);

/**
 * @brief GPU depth pick when @p gpu_depth_pick succeeds, else CPU pick.
 *
 * @param overlay Scene overlay for CPU fallback metadata / samples.
 * @param view Camera view matrix.
 * @param projection Camera projection matrix.
 * @param viewport_width Viewport width.
 * @param viewport_height Viewport height.
 * @param pixel_x Cursor X.
 * @param pixel_y Cursor Y.
 * @param gpu_picking_enabled When @c false, skips GPU and uses CPU only.
 * @param gpu_depth_pick Callable that fills @p world on success; must run with
 *        a valid GL context from the last rendered frame.
 * @param max_pixel_distance Max pixel distance for CPU fallback.
 * @return Combined @ref PickResult; @ref PickResult::used_gpu_depth set on GPU hit.
 *
 * @see pickNearestScenePoint()
 * @see pickWorldPointFromDepthBuffer()
 */
PickResult pickScenePoint(
    const SceneOverlay& overlay, const QMatrix4x4& view,
    const QMatrix4x4& projection, int viewport_width, int viewport_height,
    int pixel_x, int pixel_y, bool gpu_picking_enabled,
    const std::function<bool(int x, int y, QVector3D* world)>& gpu_depth_pick,
    float max_pixel_distance = 14.f);

/**
 * @brief Convenience pick using matrices / overlay from a @ref common::ToolContext.
 *
 * @param context Tool context with view, projection, overlay, and GPU hooks.
 * @param pixel_x Cursor X.
 * @param pixel_y Cursor Y.
 * @param max_pixel_distance Max pixel distance for CPU samples.
 * @return @ref PickResult for the tool.
 *
 * @see pickScenePoint()
 */
PickResult pickAtToolContext(const common::ToolContext& context, int pixel_x,
                             int pixel_y, float max_pixel_distance = 14.f);

}  // namespace rendering
}  // namespace autoviz
