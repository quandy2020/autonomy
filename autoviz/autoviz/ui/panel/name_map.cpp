/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/panel/name_map.hpp"

namespace autoviz {
namespace {

std::string PanelClassShortName(const std::string& class_id) {
  const auto pos = class_id.rfind('/');
  if (pos == std::string::npos) {
    return class_id;
  }
  return class_id.substr(pos + 1);
}

}  // namespace

std::string NormalizePanelObjectName(const std::string& object_name) {
  static const struct {
    const char* legacy;
    const char* canonical;
  } kAliases[] = {
      {"TopicsDock", "ChannelBrowserDock"},
  };
  for (const auto& entry : kAliases) {
    if (object_name == entry.legacy) {
      return entry.canonical;
    }
  }
  return object_name;
}

std::string MapPanelClassToObjectName(const std::string& class_or_name) {
  static const struct {
    const char* key;
    const char* autoviz_object_name;
  } kMap[] = {
      {"Displays", "DisplaysDock"},
      {"Selection", "SelectionDock"},
      {"Tool Properties", "ToolPropertiesDock"},
      {"Views", "ViewsDock"},
      {"Time", "TimeDock"},
      {"Record", "RecordDock"},
  };
  const std::string short_name = PanelClassShortName(class_or_name);
  for (const auto& entry : kMap) {
    if (short_name == entry.key || class_or_name == entry.key) {
      return entry.autoviz_object_name;
    }
  }
  return NormalizePanelObjectName(class_or_name);
}

}  // namespace autoviz
