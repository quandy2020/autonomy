/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file viewport_widget.hpp
 * @brief Abstract interface for a 3D viewport widget (OpenGL or Ogre).
 *
 * Decouples VisualizationFrame / split-panel hosts from the concrete
 * @ref RenderWindow or @ref OgreRenderWindow implementation.
 *
 * @see RenderWindow
 * @see OgreRenderWindow
 * @see ViewController
 * @see SceneOverlay
 */

#pragma once

#include <QWidget>

namespace autoviz {
namespace rendering {
class SceneOverlay;
class ViewController;
}

/**
 * @class ViewportWidget
 * @brief Polymorphic handle to a viewport: Qt widget, overlay, camera, tick.
 *
 * Implementations own their @c QWidget (often @c this) and a
 * @ref rendering::ViewController. Overlay lifetime is owned by the caller
 * (typically VisualizationManager).
 *
 * @note Not a @c QObject; concrete classes inherit @c QWidget separately.
 */
class ViewportWidget {
 public:
  /** @brief Virtual destructor for polymorphic deletion. */
  virtual ~ViewportWidget() = default;

  /**
   * @brief Returns the Qt widget to embed in docks / splitters.
   * @return Non-null widget (usually the concrete render window itself).
   */
  virtual QWidget* widget() = 0;

  /**
   * @brief Attaches the per-frame scene overlay drawn after the clear/grid.
   *
   * @param overlay Non-owning pointer; may be @c nullptr to clear.
   * @see rendering::SceneOverlay
   */
  virtual void setSceneOverlay(rendering::SceneOverlay* overlay) = 0;

  /**
   * @brief Returns the viewport's camera controller.
   * @return Mutable reference to the owned @ref rendering::ViewController.
   */
  virtual rendering::ViewController& viewController() = 0;

  /**
   * @brief Advances time-dependent camera / tool state (e.g. FPS walk).
   *
   * @param delta_seconds Elapsed seconds since the last tick.
   */
  virtual void tick(float delta_seconds) = 0;

  /**
   * @brief Requests a repaint of the viewport (Qt update / Ogre render).
   */
  virtual void requestUpdate() = 0;
};

}  // namespace autoviz
