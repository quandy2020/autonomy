/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <cstdint>

#include <QColor>

namespace autoviz {
namespace rendering {

/** Reference ground grid (built-in show_grid), aligned with rviz Grid display props. */
struct ReferenceGridSettings {
  /** Total cells along each axis on the ground plane (rviz «Plane Cell Count»). */
  int plane_cell_count = 10;
  float cell_length = 1.f;
  QColor color{80, 80, 80};
  float alpha = 1.f;
  bool show_axes = true;
  /** RGB axes at the origin (rviz-style, roughly one cell long). */
  float axis_length = 1.f;

  bool operator==(const ReferenceGridSettings& other) const {
    return plane_cell_count == other.plane_cell_count &&
           cell_length == other.cell_length && color == other.color &&
           alpha == other.alpha && show_axes == other.show_axes &&
           axis_length == other.axis_length;
  }

  bool operator!=(const ReferenceGridSettings& other) const {
    return !(*this == other);
  }
};

/** Point cloud draw style (rviz PointCloud::RenderMode). */
enum class PointCloudStyle {
  kPoints = 0,
  kSquares,
  kFlatSquares,
  kSpheres,
  kTiles,
  kBoxes,
};

}  // namespace rendering
}  // namespace autoviz
