/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/ogre_label_draw.hpp"

#include <algorithm>

#include <QString>

#include "autoviz/common/display_context.hpp"
#include "autoviz/rendering/scene_overlay.hpp"
#include "autoviz/rendering/text_raster_utils.hpp"

#ifdef AUTOVIZ_USE_OGRE
#include "autoviz/rendering/ogre_scene_host.hpp"
#endif

namespace autoviz {
namespace display {
namespace {

void syncOgreDisplayVisibility(common::DisplayContext* context,
                               const std::string& display_name) {
#ifdef AUTOVIZ_USE_OGRE
  if (context != nullptr && context->ogre_scene_host != nullptr &&
      context->active_display_visibility_bits != nullptr) {
    context->ogre_scene_host->setDisplayVisibilityBits(
        display_name, *context->active_display_visibility_bits);
  }
#endif
}

void drawLabelsGlFallback(rendering::SceneOverlay& scene,
                        const std::vector<TextLabelInstance>& labels) {
  for (const TextLabelInstance& label : labels) {
    const QString text = QString::fromStdString(label.text).trimmed();
    if (text.isEmpty()) {
      scene.addViewFacingQuad(label.position, std::max(0.04f, label.char_height * 0.5f),
                              label.color);
      continue;
    }
    const int pixel_height = static_cast<int>(
        std::clamp(label.char_height * 120.f, 16.f, 128.f));
    const QImage label_image =
        rendering::RasterizeTextLabel(text, label.color, pixel_height);
    if (label_image.isNull()) {
      continue;
    }
    const float aspect =
        static_cast<float>(label_image.width()) /
        static_cast<float>(std::max(1, label_image.height()));
    const float half_height = label.char_height * 0.5f;
    const float half_width = half_height * aspect;
    scene.addViewFacingTexturedQuad(label.position, half_width, half_height,
                                    label_image);
  }
}

}  // namespace

bool drawLabelsOgreOrGl(common::DisplayContext* context,
                        rendering::SceneOverlay& scene,
                        const std::string& display_name,
                        const std::vector<TextLabelInstance>& labels) {
#ifdef AUTOVIZ_USE_OGRE
  if (context != nullptr && context->ogre_scene_host != nullptr) {
    std::vector<rendering::OgreTextLabel> ogre_labels;
    ogre_labels.reserve(labels.size());
    for (const TextLabelInstance& label : labels) {
      ogre_labels.push_back({label.text, label.position, label.color,
                             label.char_height, label.space_width});
    }
    syncOgreDisplayVisibility(context, display_name);
    context->ogre_scene_host->setDisplayLabels(display_name, ogre_labels);
    return true;
  }
#endif
  drawLabelsGlFallback(scene, labels);
  return !labels.empty();
}

}  // namespace display
}  // namespace autoviz
