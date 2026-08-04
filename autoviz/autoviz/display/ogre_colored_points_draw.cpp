/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/ogre_colored_points_draw.hpp"

#include "autoviz/common/display_context.hpp"
#include "autoviz/common/selection_handler.hpp"
#include "autoviz/rendering/scene_overlay.hpp"

#ifdef AUTOVIZ_USE_OGRE
#include "autoviz/rendering/ogre_indexed_palette.hpp"
#include "autoviz/rendering/ogre_scene_host.hpp"
#endif

namespace autoviz {
namespace display {

bool drawColoredPointsOgreOrGl(common::DisplayContext* context,
                               rendering::SceneOverlay& scene,
                               const std::string& display_name,
                               const std::string& display_type, float point_size,
                               rendering::PointCloudStyle style,
                               const std::vector<ColoredPoint3D>& points,
                               bool per_point_pick) {
  if (points.empty()) {
    return false;
  }

#ifdef AUTOVIZ_USE_OGRE
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
    if (per_point_pick) {
      scene.setPickSource(&display_name, &display_type);
      const common::PickHandle cloud_handle =
          scene.registerPickEntry(QVector3D(), -1, nullptr);
      context->ogre_scene_host->setCloudPickHandle(display_name, cloud_handle);
      for (std::size_t i = 0; i < points.size(); ++i) {
        auto handler =
            common::CreateSelectionHandler<common::PointCloudSelectionHandler>();
        handler->setDisplayInfo(display_name, display_type);
        handler->setPointIndex(static_cast<int>(i));
        scene.registerPickEntry(points[i].position, static_cast<int>(i), handler);
      }
    }
    return true;
  }
#endif

  scene.setPointSize(point_size);
  scene.setPickSource(&display_name, &display_type);
  for (std::size_t i = 0; i < points.size(); ++i) {
    auto handler = per_point_pick
                       ? common::CreateSelectionHandler<
                             common::PointCloudSelectionHandler>()
                       : nullptr;
    if (handler != nullptr) {
      handler->setDisplayInfo(display_name, display_type);
      handler->setPointIndex(static_cast<int>(i));
    }
    scene.addPickPoint(points[i].position, points[i].color, static_cast<int>(i),
                       handler);
  }
  return false;
}

}  // namespace display
}  // namespace autoviz
