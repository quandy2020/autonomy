/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins robot::LinkUpdater
 * (Willow Garage / Open Source Robotics Foundation).
 *****************************************************************************/

/**
 * @file link_updater.hpp
 * @brief Interface that supplies per-link poses for @ref Robot::update.
 *
 * @see TFLinkUpdater
 * @see Robot
 */

#pragma once

#include <string>

#include <QQuaternion>
#include <QVector3D>

namespace autoviz {
namespace display {
namespace robot {

/** Status level for link updater callbacks (RViz StatusProperty subset). */
enum class StatusLevel { kOk = 0, kWarn = 1, kError = 2 };

/**
 * @class LinkUpdater
 * @brief Abstract source of visual/collision transforms for a named link.
 */
class LinkUpdater {
 public:
  virtual ~LinkUpdater() = default;

  /**
   * @brief Looks up transforms for @p link_name.
   * @return @c true when both visual and collision poses are valid.
   */
  virtual bool getLinkTransforms(const std::string& link_name,
                                 QVector3D* visual_position,
                                 QQuaternion* visual_orientation,
                                 QVector3D* collision_position,
                                 QQuaternion* collision_orientation) const = 0;

  virtual void setLinkStatus(StatusLevel level, const std::string& link_name,
                             const std::string& text) const {
    (void)level;
    (void)link_name;
    (void)text;
  }
};

}  // namespace robot
}  // namespace display
}  // namespace autoviz
