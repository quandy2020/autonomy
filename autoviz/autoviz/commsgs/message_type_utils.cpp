/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/commsgs/message_type_utils.hpp"

#include <algorithm>

namespace autoviz {
namespace commsgs {
namespace {

std::string StripPrefix(std::string value) {
  static const char* kPrefixes[] = {
      "autonomy.commsgs.proto.",
      "automsgs.msgs.",
  };
  for (const char* prefix : kPrefixes) {
    if (value.rfind(prefix, 0) == 0) {
      value.erase(0, std::char_traits<char>::length(prefix));
      break;
    }
  }
  return value;
}

std::string RemapLegacyPackage(std::string short_name) {
  static const struct {
    const char* legacy;
    const char* automsgs;
  } kPackageMap[] = {
      {"planning_msgs.Odometry", "nav_msgs.Odometry"},
      {"planning_msgs.Path", "nav_msgs.Path"},
      {"map_msgs.OccupancyGrid", "nav_msgs.OccupancyGrid"},
  };
  for (const auto& entry : kPackageMap) {
    if (short_name == entry.legacy) {
      return entry.automsgs;
    }
  }
  return short_name;
}

}  // namespace

std::string NormalizeMessageType(const std::string& message_type) {
  if (message_type.empty()) {
    return message_type;
  }
  const std::string short_name = RemapLegacyPackage(StripPrefix(message_type));
  if (message_type.rfind("automsgs.msgs.", 0) == 0) {
    return message_type;
  }
  return "automsgs.msgs." + short_name;
}

bool MessageTypesCompatible(const std::string& left,
                            const std::string& right) {
  if (left.empty() || right.empty()) {
    return false;
  }
  return NormalizeMessageType(left) == NormalizeMessageType(right);
}

}  // namespace commsgs
}  // namespace autoviz
