// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file coverage_server.hpp
 * @brief Native complete-coverage coverage server implementation. */
#pragma once
#include "autonomy/task/navigation/coverage/decomp_generator.hpp"
#include "autonomy/task/navigation/coverage/headland_generator.hpp"
#include "autonomy/task/navigation/coverage/path_generator.hpp"
#include "autonomy/task/navigation/coverage/robot_params.hpp"
#include "autonomy/task/navigation/coverage/route_generator.hpp"
#include "autonomy/task/navigation/coverage/row_swath_generator.hpp"
#include "autonomy/task/navigation/coverage/swath_generator.hpp"
namespace autonomy::task::coverage {
class CoverageServer {
 public:
  explicit CoverageServer(Configuration configuration = {});
  /** @brief Execute the requested coverage planning pipeline. */
  ComputeAction::Result Compute(const proto::CoverageRequest&,
                                const CancelChecker& = {},
                                const Progress& = {}) const;
  const Configuration& Config() const { return config_; }

 private:
  Configuration config_;
};
// Conversion preserves metric segment length, vehicle heading, direction and
// work semantics.
/** @brief Export every pose with its vehicle direction and operational
 * metadata. */
void ExportPath(const F2CPath&, const F2CField&, const std::string&, bool,
                double, ComputeAction::Result&);
/** @brief Export coverage swaths in the requested coordinate frame. */
void ExportSwaths(const F2CSwaths&, const F2CField&, const std::string&, bool,
                  bool, proto::PathComponents&);
}  // namespace autonomy::task::coverage
