/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

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

struct PickResult {
  bool hit = false;
  QVector3D position;
  std::string display_name;
  std::string display_type;
  common::PickHandle pick_handle = common::kInvalidPickHandle;
  int point_index = -1;
  std::vector<std::pair<std::string, std::string>> properties;
  float pixel_distance = 0.f;
  float eye_depth = 0.f;
  bool used_gpu_depth = false;
};

/** CPU screen-space pick (tagged samples, then geometry). */
PickResult pickNearestScenePoint(const SceneOverlay& overlay,
                                 const QMatrix4x4& view,
                                 const QMatrix4x4& projection, int viewport_width,
                                 int viewport_height, int pixel_x, int pixel_y,
                                 float max_pixel_distance = 14.f);

/**
 * GPU depth pick when @p gpu_depth_pick succeeds, else CPU pick.
 * @p gpu_depth_pick must run with a valid GL context (last frame rendered).
 */
PickResult pickScenePoint(
    const SceneOverlay& overlay, const QMatrix4x4& view,
    const QMatrix4x4& projection, int viewport_width, int viewport_height,
    int pixel_x, int pixel_y, bool gpu_picking_enabled,
    const std::function<bool(int x, int y, QVector3D* world)>& gpu_depth_pick,
    float max_pixel_distance = 14.f);

PickResult pickAtToolContext(const common::ToolContext& context, int pixel_x,
                             int pixel_y, float max_pixel_distance = 14.f);

}  // namespace rendering
}  // namespace autoviz
