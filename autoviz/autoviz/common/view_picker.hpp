/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file view_picker.hpp
 * @brief Screen-space → fixed-frame 3D point picking (RViz ViewPicker).
 *
 * Uses @ref ToolContext GPU depth pick (when available) or CPU fallbacks
 * without depending on ROS.
 *
 * @see ToolContext
 * @see Tool
 */

#pragma once

#include <QVector3D>

namespace autoviz {
namespace common {

struct ToolContext;

/**
 * @class ViewPicker
 * @brief Resolves a pixel coordinate to a world-space 3D point.
 *
 * Attach a @ref ToolContext via @ref setContext() before calling
 * @ref get3DPoint(). Returns @c false on miss / unavailable picking.
 */
class ViewPicker {
 public:
  /**
   * @brief Sets the tool context providing pick callbacks and viewport size.
   *
   * @param context Non-owning pointer; may be @c nullptr to clear.
   */
  void setContext(ToolContext* context) { context_ = context; }

  /**
   * @brief Picks a 3D point under screen pixel (@p pixel_x, @p pixel_y).
   *
   * @param pixel_x Viewport X in pixels.
   * @param pixel_y Viewport Y in pixels.
   * @param[out] point Receives the fixed-frame world point on success.
   * @return @c true if a point was found; @c false on miss.
   */
  bool get3DPoint(int pixel_x, int pixel_y, QVector3D* point) const;

 private:
  /** Non-owning tool context (GPU pick / viewport dims). */
  ToolContext* context_ = nullptr;
};

}  // namespace common
}  // namespace autoviz
