/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins InteractiveMarkerNamespaceProperty
 * (Open Source Robotics Foundation).
 *****************************************************************************/

#include "autoviz/plugins/displays/interactive_markers/interactive_marker_namespace_property.hpp"

namespace autoviz {
namespace display {

QString InteractiveMarkerNamespaceProperty::namespaceFromUpdateChannel(
    const QString& update_channel) {
  QString topic = update_channel.trimmed();
  if (topic.endsWith(QLatin1String("/update"))) {
    topic.chop(7);
  } else if (topic.endsWith(QLatin1String("/init"))) {
    topic.chop(5);
  }
  while (topic.size() > 1 && topic.endsWith(QLatin1Char('/'))) {
    topic.chop(1);
  }
  return topic;
}

std::string InteractiveMarkerNamespaceProperty::namespaceFromUpdateChannelStd(
    const std::string& update_channel) {
  return namespaceFromUpdateChannel(QString::fromStdString(update_channel))
      .toStdString();
}

}  // namespace display
}  // namespace autoviz
