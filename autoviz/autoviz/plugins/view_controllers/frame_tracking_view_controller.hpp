/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_common::FramePositionTrackingViewController
 * (Willow Garage / Open Source Robotics Foundation).
 *****************************************************************************/

/**
 * @file frame_tracking_view_controller.hpp
 * @brief Base adapter for Autoviz view-controller plugins (no ROS / Ogre).
 *
 * RViz view controllers inherit @c FramePositionTrackingViewController and own
 * an Ogre camera. Autoviz keeps a single @ref rendering::ViewController host
 * (matrices + mouse) and exposes RViz-shaped type wrappers under
 * @c display/view_controllers/* that configure that host.
 *
 * @see rendering::ViewController
 * @see OrbitViewController
 */

#pragma once

#include <QMatrix4x4>
#include <QString>
#include <QVector3D>

#include "autoviz/rendering/view_controller.hpp"

namespace autoviz {
namespace display {
namespace view_controllers {

/**
 * @class FrameTrackingViewController
 * @brief Non-owning facade over @ref rendering::ViewController.
 */
class FrameTrackingViewController {
 public:
  explicit FrameTrackingViewController(rendering::ViewController* host);
  virtual ~FrameTrackingViewController() = default;

  rendering::ViewController* host() const { return host_; }

  /** Canonical type id for the Views panel / session config. */
  virtual QString getClassId() const = 0;

  /** Activates this controller type on the shared host. */
  virtual void activate();

  virtual void reset();

  void setTargetFrame(const QString& frame);
  QString getTargetFrame() const;

  bool handleMouseEvent(const rendering::ViewportMouseEvent& event);
  bool handleKeyEvent(int key, bool pressed);
  void tick(float delta_seconds);

  QMatrix4x4 viewMatrix() const;
  QMatrix4x4 projectionMatrix(float aspect_ratio) const;

 protected:
  void requireHost() const;

  rendering::ViewController* host_ = nullptr;
};

}  // namespace view_controllers
}  // namespace display
}  // namespace autoviz
