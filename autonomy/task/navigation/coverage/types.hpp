// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file types.hpp
 * @brief Native complete-coverage types implementation. */
#pragma once
#include <algorithm>
#include <cctype>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "autonomy/common/logging.hpp"
#include "autonomy/planning/planner/fields2cover/fields2cover.hpp"
#include "autonomy/task/navigation/coverage/canceled_exception.hpp"
#include "autonomy/task/navigation/coverage/configuration.hpp"
#include "autonomy/task/navigation/coverage/coverage_exception.hpp"
#include "autonomy/task/navigation/coverage/proto/coverage.pb.h"
#include "autonomy/task/navigation/coverage/request_exception.hpp"

namespace autonomy::task::coverage {
namespace f2c = ::autonomy::planning::planner::fields2cover;
using namespace ::autonomy::planning::planner::fields2cover;
using Field = F2CCell;
using Fields = F2CCells;
using Swaths = F2CSwaths;
using Point = F2CPoint;
using Polygon = F2CLinearRing;
using LineString = F2CLineString;
using Path = F2CPath;
using PathState = F2CPathState;
using NavPath = ::automsgs::msgs::nav_msgs::Path;
using ComputeAction = proto::ComputeCoveragePathAction;
using NavigateAction = proto::NavigateCompleteCoverageAction;
enum class RouteType {
  kUnknown,
  kBoustrophedon,
  kSnake,
  kSpiral,
  kCustom,
  kTsp
};

inline std::string Upper(std::string text) {
  for (char& c : text) c = std::toupper(static_cast<unsigned char>(c));
  return text;
}
inline bool Unknown(const std::string& mode) {
  return mode.empty() || Upper(mode) == "UNKNOWN";
}
inline bool Enabled(bool has, bool value) { return !has || value; }
using CancelChecker = std::function<bool()>;
using Progress = std::function<void(const std::string&, double)>;
inline void CheckCanceled(const CancelChecker& cancel) {
  if (cancel && cancel()) throw CanceledException();
}
inline RouteType RouteTypeFromString(const std::string& text) {
  const auto s = Upper(text);
  if (s == "BOUSTROPHEDON") return RouteType::kBoustrophedon;
  if (s == "SNAKE") return RouteType::kSnake;
  if (s == "SPIRAL") return RouteType::kSpiral;
  if (s == "CUSTOM") return RouteType::kCustom;
  if (s == "TSP") return RouteType::kTsp;
  return RouteType::kUnknown;
}
}  // namespace autonomy::task::coverage
