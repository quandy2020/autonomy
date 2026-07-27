/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

namespace autoviz {
namespace common {

struct DisplayTypeInfo {
  std::string type;
  std::string package;
  std::string description;
  /** Empty if the display does not subscribe to a channel. */
  std::vector<std::string> message_types;
};

class DisplayCatalog {
 public:
  static std::vector<DisplayTypeInfo> allTypes();
  static DisplayTypeInfo infoForType(const std::string& type);
  static std::vector<std::string> typesForMessageType(
      const std::string& message_type);
};

}  // namespace common
}  // namespace autoviz
