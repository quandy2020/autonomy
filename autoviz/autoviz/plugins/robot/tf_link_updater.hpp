/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins robot::TFLinkUpdater
 * (Willow Garage / Bosch / Open Source Robotics Foundation).
 *****************************************************************************/

/**
 * @file tf_link_updater.hpp
 * @brief @ref LinkUpdater that resolves link frames via TF (+ optional prefix).
 *
 * @see LinkUpdater
 * @see Robot
 */

#pragma once

#include <functional>
#include <string>

#include "autoviz/plugins/robot/link_updater.hpp"

namespace autoviz {

namespace common {
class DisplayContext;
}

namespace display {
namespace robot {

/**
 * @class TFLinkUpdater
 * @brief Looks up @c fixed_frame ← tf_prefix/link_name via DisplayContext TF.
 */
class TFLinkUpdater : public LinkUpdater {
 public:
  using StatusCallback =
      std::function<void(StatusLevel, const std::string&, const std::string&)>;

  /**
   * @param context Non-owning display context (TF buffer + fixed frame).
   * @param status_cb Optional status reporter (Display status tree).
   * @param tf_prefix Optional multi-robot TF prefix.
   */
  explicit TFLinkUpdater(common::DisplayContext* context,
                         StatusCallback status_cb = {},
                         std::string tf_prefix = {});

  bool getLinkTransforms(const std::string& link_name,
                         QVector3D* visual_position,
                         QQuaternion* visual_orientation,
                         QVector3D* collision_position,
                         QQuaternion* collision_orientation) const override;

  void setLinkStatus(StatusLevel level, const std::string& link_name,
                     const std::string& text) const override;

 private:
  common::DisplayContext* context_ = nullptr;
  StatusCallback status_callback_;
  std::string tf_prefix_;
};

}  // namespace robot
}  // namespace display
}  // namespace autoviz
