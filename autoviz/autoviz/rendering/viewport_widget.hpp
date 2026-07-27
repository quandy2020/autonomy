/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

namespace autoviz {
namespace rendering {
class SceneOverlay;
class ViewController;
}

class ViewportWidget {
 public:
  virtual ~ViewportWidget() = default;

  virtual QWidget* widget() = 0;
  virtual void setSceneOverlay(rendering::SceneOverlay* overlay) = 0;
  virtual void setGridVisible(bool visible) = 0;
  virtual rendering::ViewController& viewController() = 0;
  virtual void tick(float delta_seconds) = 0;
  virtual void requestUpdate() = 0;
};

}  // namespace autoviz
