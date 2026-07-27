/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QVector3D>

namespace autoviz {
namespace common {

struct ToolContext;

/** rviz_common::ViewPicker — 3D point pick without ROS (uses ToolContext GPU/CPU pick). */
class ViewPicker {
 public:
  void setContext(ToolContext* context) { context_ = context; }

  /** Screen (x,y) → fixed-frame 3D point. Returns false on miss. */
  bool get3DPoint(int pixel_x, int pixel_y, QVector3D* point) const;

 private:
  ToolContext* context_ = nullptr;
};

}  // namespace common
}  // namespace autoviz
