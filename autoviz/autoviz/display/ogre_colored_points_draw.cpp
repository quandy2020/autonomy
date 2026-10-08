/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/ogre_colored_points_draw.hpp"

#include "autoviz/common/display_context.hpp"
#include "autoviz/common/selection_handler.hpp"
#include "autoviz/rendering/scene_overlay.hpp"

#include "autoviz/rendering/ogre_indexed_palette.hpp"
#include "autoviz/rendering/ogre_scene_host.hpp"

namespace autoviz {
namespace display {

bool drawColoredPointsOgreOrGl(common::DisplayContext* context,
                               rendering::SceneOverlay& scene,
                               const std::string& display_name,
                               const std::string& display_type, float point_size,
                               rendering::PointCloudStyle style,
                               const std::vector<ColoredPoint3D>& points,
                               bool selectable) {
  if (points.empty()) {
    return false;
  }

  if (context != nullptr && context->ogre_scene_host != nullptr) {
    rendering::OgreIndexedPalette::ensureRainbowPalette();
    std::vector<QVector3D> positions;
    std::vector<QColor> colors;
    positions.reserve(points.size());
    colors.reserve(points.size());
    for (const auto& pt : points) {
      positions.push_back(pt.position);
      colors.push_back(pt.color);
    }
    context->ogre_scene_host->setDisplayPoints(display_name, point_size, style,
                                               positions, colors);
    if (context->active_display_visibility_bits != nullptr) {
      context->ogre_scene_host->setDisplayVisibilityBits(
          display_name, *context->active_display_visibility_bits);
    }
    // One cloud-level pick handle only (RViz Pick1 / color-by-index). Never
    // allocate per-point SelectionHandlers — dense depth/lidar clouds would
    // create hundreds of thousands of heap objects per frame and corrupt the
    // allocator ("corrupted size vs. prev_size").
    if (selectable) {
      scene.setPickSource(&display_name, &display_type);
      auto handler =
          common::CreateSelectionHandler<common::PointCloudSelectionHandler>();
      handler->setDisplayInfo(display_name, display_type);
      handler->setPointIndex(-1);
      const common::PickHandle cloud_handle =
          scene.registerPickEntry(QVector3D(), -1, handler);
      context->ogre_scene_host->setCloudPickHandle(display_name, cloud_handle);
    }
    return true;
  }

  scene.setPointSize(point_size);
  if (selectable) {
    scene.setPickSource(&display_name, &display_type);
    auto handler =
        common::CreateSelectionHandler<common::PointCloudSelectionHandler>();
    handler->setDisplayInfo(display_name, display_type);
    handler->setPointIndex(-1);
    scene.registerPickEntry(QVector3D(), -1, handler);
  }
  // Clear pick source so addPickPoint does not allocate a handler per vertex.
  scene.setPickSource(nullptr, nullptr);
  for (const auto& pt : points) {
    scene.addPickPoint(pt.position, pt.color, -1, nullptr);
  }
  return false;
}

}  // namespace display
}  // namespace autoviz
