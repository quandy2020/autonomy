/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file types.hpp
 * @brief Native complete-coverage types implementation.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_TYPES_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_TYPES_HPP_

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
// One Fields2Cover cell (outer ring plus holes).
using Field = F2CCell;

// A field made of several cells.
using Fields = F2CCells;

// A group of parallel coverage passes.
using Swaths = F2CSwaths;

// Metric point in the request frame.
using Point = F2CPoint;

// Closed ring used for fields, holes and headland bands.
using Polygon = F2CLinearRing;

// Open polyline.
using LineString = F2CLineString;

// Ordered path states produced by the path generator.
using Path = F2CPath;

// One path state: point, heading, length, direction and section type.
using PathState = F2CPathState;

// Exported path message.
using NavPath = ::automsgs::msgs::nav_msgs::Path;

// Compute action type of this module.
using ComputeAction = proto::ComputeCoveragePathAction;

// Navigate action type of this module.
using NavigateAction = proto::NavigateCompleteCoverageAction;

/**
 * @brief Route strategy requested by a coverage goal.
 */
enum class RouteType {
  // Unset or unrecognised route name.
  kUnknown,
  // Back-and-forth sweep, the default for rooms.
  kBoustrophedon,
  // Same sweep without reversing the vehicle.
  kSnake,
  // Inward spiral.
  kSpiral,
  // Caller supplied swath order.
  kCustom,
  // TSP order over the swath endpoints.
  kTsp
};

/**
 * @brief Upper-cases a mode string for case-insensitive comparison.
 * @param[in] text Mode name as received from the request.
 * @return Upper-cased copy; the input is left untouched.
 */
inline std::string Upper(std::string text) {
  for (char& c : text) c = std::toupper(static_cast<unsigned char>(c));
  return text;
}
/**
 * @brief Reports whether a mode name means "use the configured default".
 * @param[in] mode Mode name from the request.
 * @return true for an empty name or the literal UNKNOWN, in any case.
 */
inline bool Unknown(const std::string& mode) {
  return mode.empty() || Upper(mode) == "UNKNOWN";
}
/**
 * @brief Resolves an optional boolean that defaults to enabled.
 * @param[in] has Whether the request carried the field.
 * @param[in] value Value carried by the request when @p has is true.
 * @return false only when the field is present and explicitly disabled.
 */
inline bool Enabled(bool has, bool value) { return !has || value; }
using CancelChecker = std::function<bool()>;
using Progress = std::function<void(const std::string&, double)>;
/**
 * @brief Throws when the caller requested cancellation.
 * @param[in] cancel Optional predicate; an empty predicate never cancels.
 * @throws CanceledException when @p cancel reports a cancellation.
 */
inline void CheckCanceled(const CancelChecker& cancel) {
  if (cancel && cancel()) throw CanceledException();
}
/**
 * @brief Maps a route name to its enum value.
 * @param[in] text Route name from the request, any case.
 * @return Matching route type, or @c RouteType::kUnknown when unrecognised.
 */
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

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_TYPES_HPP_
