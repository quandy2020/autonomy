/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins InteractiveMarkerNamespaceProperty
 * (Open Source Robotics Foundation).
 *****************************************************************************/

/**
 * @file interactive_marker_namespace_property.hpp
 * @brief Resolves Interactive Marker server namespace from an update channel.
 *
 * RViz exposes an editable enum of discovered namespaces. Autoviz derives the
 * namespace from the Display update channel (typically @c /ns/update → @c /ns).
 *
 * @see InteractiveMarkerDisplay
 */

#pragma once

#include <string>

#include <QString>

namespace autoviz {
namespace display {

/**
 * @class InteractiveMarkerNamespaceProperty
 * @brief Lightweight namespace helper matching RViz's property role.
 */
class InteractiveMarkerNamespaceProperty {
 public:
  /**
   * @brief Derives the server namespace from an update topic.
   *
   * @param update_channel e.g. @c /my_server/update
   * @return Namespace path (e.g. @c /my_server), or the channel itself.
   */
  static QString namespaceFromUpdateChannel(const QString& update_channel);

  /**
   * @brief Same as @ref namespaceFromUpdateChannel for @c std::string.
   */
  static std::string namespaceFromUpdateChannelStd(
      const std::string& update_channel);
};

}  // namespace display
}  // namespace autoviz
