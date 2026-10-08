/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/viewport_panel.hpp"

#include "autoviz/rendering/ogre_render_window.hpp"

namespace autoviz {

rendering::ViewController* ViewportPanelEntry::viewController()
    const {
  if (ogre_viewport != nullptr) {
    return &ogre_viewport->viewController();
  }
  return nullptr;
}

}  // namespace autoviz
