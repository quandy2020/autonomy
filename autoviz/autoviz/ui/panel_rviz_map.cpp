/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/panel_rviz_map.hpp"

namespace autoviz {
namespace {

std::string RvizShortName(const std::string& rviz_class) {
  const auto pos = rviz_class.rfind('/');
  if (pos == std::string::npos) {
    return rviz_class;
  }
  return rviz_class.substr(pos + 1);
}

}  // namespace

std::string NormalizePanelObjectName(const std::string& object_name) {
  static const struct {
    const char* legacy;
    const char* canonical;
  } kAliases[] = {
      {"Drone3DDock", "Vehicle3DDock"},
      {"Drone 3D", "Vehicle3DDock"},
      {"TopicsDock", "ChannelBrowserDock"},
  };
  for (const auto& entry : kAliases) {
    if (object_name == entry.legacy) {
      return entry.canonical;
    }
  }
  return object_name;
}

std::string MapRvizPanelToObjectName(const std::string& rviz_class_or_name) {
  static const struct {
    const char* rviz_key;
    const char* autoviz_object_name;
  } kMap[] = {
      {"Displays", "DisplaysDock"},
      {"Selection", "SelectionDock"},
      {"Tool Properties", "ToolPropertiesDock"},
      {"Views", "ViewsDock"},
      {"Time", "TimeDock"},
      {"Help", "HelpDock"},
      {"Transformation", "TransformationDock"},
  };
  const std::string short_name = RvizShortName(rviz_class_or_name);
  for (const auto& entry : kMap) {
    if (short_name == entry.rviz_key || rviz_class_or_name == entry.rviz_key) {
      return entry.autoviz_object_name;
    }
  }
  return NormalizePanelObjectName(rviz_class_or_name);
}

}  // namespace autoviz
