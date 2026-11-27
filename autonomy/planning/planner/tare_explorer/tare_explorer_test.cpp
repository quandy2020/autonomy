// Copyright 2026 The Openbot Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
/**
 * @file tare_explorer_test.cpp
 * @brief Ownership regressions for the Vista-backed TARE compatibility id.
 */
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>

#include <memory>
#include <optional>
#include <stdexcept>

#include "autonomy/planning/planner/tare_explorer/tare_explorer_planner.hpp"
#include "autonomy/planning/planner/vista/vista_planner.hpp"
#include "autonomy/planning/proto/planning_options.pb.h"

namespace autonomy::planning::planner::tare_explorer {
namespace {

using automsgs::msgs::geometry_msgs::PoseStamped;
using automsgs::msgs::nav_msgs::Path;

/** @brief Fails a regression without disabling checks in release builds. */
void Require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

/** @brief Both ids must retain exactly the same Vista owner. */
void SharesVistaOwner() {
    proto::PlannerOptions options;
    auto vista = std::make_shared<vista::VistaPlanner>();
    TareExplorerPlanner tare(options, "tare", nullptr, vista);
    Require(tare.SharedVistaPlanner() == vista,
            "TARE compatibility created a second route owner");
}

/** @brief An unconfigured delegate clears outputs and reports that condition.
 */
void UnconfiguredDelegateClearsOutput() {
    proto::PlannerOptions options;
    auto vista = std::make_shared<vista::VistaPlanner>();
    TareExplorerPlanner tare(options, "tare", nullptr, vista);
    PoseStamped start;
    PoseStamped goal;
    Path path;
    path.add_poses();
    const auto result =
        tare.CreatePlan(start, goal, path, [] { return false; });
    Require(result == static_cast<uint32>(
                          proto::PlannerResultCode::PLANNER_NOT_INITIALIZED),
            "TARE compatibility did not preserve Vista initialization status");
    Require(path.poses().empty(),
            "TARE compatibility retained stale output after route failure");
}

/** @brief The retired lifecycle shim performs no work or cancellation call. */
void ExplorationLifecycleIsRejected() {
    proto::PlannerOptions options;
    auto vista = std::make_shared<vista::VistaPlanner>();
    TareExplorerPlanner tare(options, "tare", nullptr, vista);
    PoseStamped robot;
    Path path;
    path.add_poses();
    bool complete = true;
    bool cancel_called = false;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    const auto result =
        tare.CreateExplorationPlan(robot, std::nullopt, path, &complete, [&] {
            cancel_called = true;
            return false;
        });
#pragma GCC diagnostic pop
    Require(result == static_cast<uint32>(
                          proto::PlannerResultCode::PLANNER_INVALID_PLUGIN),
            "retired exploration lifecycle was not rejected");
    Require(path.poses().empty() && !complete && !cancel_called,
            "retired exploration lifecycle retained state or performed work");
}

/** @brief A compatibility id may never exist without its route owner. */
void RejectsMissingVistaOwner() {
    bool threw = false;
    try {
        proto::PlannerOptions options;
        TareExplorerPlanner tare(options, "tare", nullptr,
                                 std::shared_ptr<vista::VistaPlanner>{});
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    Require(threw, "TARE compatibility accepted a missing Vista owner");
}

}  // namespace
}  // namespace autonomy::planning::planner::tare_explorer

int main() {
    autonomy::planning::planner::tare_explorer::SharesVistaOwner();
    autonomy::planning::planner::tare_explorer::
        UnconfiguredDelegateClearsOutput();
    autonomy::planning::planner::tare_explorer::
        ExplorationLifecycleIsRejected();
    autonomy::planning::planner::tare_explorer::RejectsMissingVistaOwner();
    return 0;
}
