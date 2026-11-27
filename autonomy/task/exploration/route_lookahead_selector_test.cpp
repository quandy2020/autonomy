/* Copyright 2026 The Openbot Authors. */
/** @file route_lookahead_selector_test.cpp
 * @brief Regressions for provenance-checked Vista route lookahead.
 */
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "autonomy/task/exploration/routing/route_lookahead_selector.hpp"

namespace {
namespace e = autonomy::task::exploration;
namespace n = autonomy::navigation;
namespace v = autonomy::planning::planner::vista;

void Require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

v::VistaRoute Route() {
    v::VistaRoute route;
    route.path.frame_id = "map";
    route.path.poses = {{0, 0, 0.3}, {3, 0, 0.4}, {3, 4, 1.2}};
    route.length = 7;
    route.map_epoch = 5;
    route.geometry_revision = 9;
    route.success = true;
    v::VistaMapEvidence evidence;
    evidence.map_epoch = 5;
    evidence.geometry_revision = 9;
    evidence.frame_id = "map";
    route.evidence = evidence;
    return route;
}

e::RouteLookaheadResult Select(
    const v::VistaRoute& route, double distance,
    const e::RouteLineOfSightView* line_of_sight = nullptr,
    const std::function<bool()>& canceled = {}) {
    e::RouteLookaheadSelector selector;
    return selector.Select(route, route.path.poses.front(),
                           route.path.poses.back(), distance, 5, 9, "map",
                           line_of_sight ? 11 : 0, line_of_sight, canceled);
}

e::RouteLineOfSightView LineOfSight(
    std::function<e::RouteLineOfSightStatus(const n::Pose2d&, const n::Pose2d&)>
        check) {
    return {5, 9, 11, "map", std::move(check)};
}

void ArcLengthInterpolationAndEndpointYaw() {
    const auto middle = Select(Route(), 5);
    Require(middle.status == e::RouteLookaheadStatus::kSelected &&
                std::abs(middle.pose.x - 3) < 1e-12 &&
                std::abs(middle.pose.y - 2) < 1e-12 &&
                std::abs(middle.pose.yaw - std::acos(-1) / 2) < 1e-12 &&
                !middle.final_endpoint,
            "lookahead was not interpolated by polyline arc length");

    const auto endpoint = Select(Route(), 20);
    Require(endpoint.status == e::RouteLookaheadStatus::kSelected &&
                endpoint.final_endpoint && endpoint.pose.x == 3 &&
                endpoint.pose.y == 4 && endpoint.pose.yaw == 1.2,
            "true endpoint did not preserve its requested final yaw");

    const auto corner = Select(Route(), 3);
    Require(corner.status == e::RouteLookaheadStatus::kSelected &&
                corner.pose.x == 3 && corner.pose.y == 0 &&
                std::abs(corner.pose.yaw - std::acos(-1) / 2) < 1e-12,
            "exact interior vertex did not use its outgoing route tangent");
}

void LineOfSightStopsBeforeFailingNode() {
    int checks = 0;
    auto line_of_sight =
        LineOfSight([&checks](const n::Pose2d&, const n::Pose2d& to) {
            ++checks;
            return to.y < 1 ? e::RouteLineOfSightStatus::kVisible
                            : e::RouteLineOfSightStatus::kBlocked;
        });
    const auto limited = Select(Route(), 6, &line_of_sight);
    Require(limited.status == e::RouteLookaheadStatus::kSelected &&
                limited.line_of_sight_limited && !limited.final_endpoint &&
                limited.route_arc_length == 3 && limited.pose.x == 3 &&
                limited.pose.y == 0 && checks == 2,
            "LOS did not clamp to the node before the first failure");

    checks = 0;
    const auto near = Select(Route(), 2, &line_of_sight);
    Require(near.status == e::RouteLookaheadStatus::kSelected &&
                near.pose.x == 2 && near.pose.y == 0 && checks == 1,
            "LOS inspected route nodes beyond the requested arc");
}

void DuplicateEndpointCannotBypassLineOfSight() {
    auto route = Route();
    route.path.poses = {{0, 0, 0.3}, {1, 0, 0}, {1, 0, 1.3}};
    route.length = 1;
    int checks = 0;
    auto line_of_sight =
        LineOfSight([&checks](const n::Pose2d&, const n::Pose2d&) {
            ++checks;
            return checks == 1 ? e::RouteLineOfSightStatus::kVisible
                               : e::RouteLineOfSightStatus::kBlocked;
        });
    const auto selected = Select(route, 2, &line_of_sight);
    Require(selected.status == e::RouteLookaheadStatus::kSelected &&
                selected.line_of_sight_limited && !selected.final_endpoint &&
                selected.pose.x == 1 && selected.pose.yaw == 0 && checks == 2,
            "duplicate final node bypassed measured LOS");
}

void InvalidAndStaleRoutesFailClosed() {
    auto route = Route();
    route.evidence.reset();
    Require(
        Select(route, 2).status == e::RouteLookaheadStatus::kMissingEvidence,
        "route without Vista evidence was accepted");

    route = Route();
    route.geometry_revision = 8;
    Require(Select(route, 2).status == e::RouteLookaheadStatus::kStaleRoute,
            "stale route geometry was accepted");

    route = Route();
    route.length = 6;
    Require(Select(route, 2).status == e::RouteLookaheadStatus::kInvalidRoute,
            "malformed route length was accepted");

    route = Route();
    route.attempted = true;
    Require(Select(route, 2).status == e::RouteLookaheadStatus::kInvalidRoute,
            "attempted route was accepted as executable lookahead");

    route = Route();
    e::RouteLookaheadSelector selector;
    Require(selector.Select(route, {0, 0, 0.4}, route.path.poses.back(), 2, 5,
                            9, "map")
                    .status == e::RouteLookaheadStatus::kInvalidRoute,
            "different measured start yaw was accepted");
    auto other_goal = route.path.poses.back();
    other_goal.yaw += 0.2;
    Require(selector.Select(route, route.path.poses.front(), other_goal, 2, 5,
                            9, "map")
                    .status == e::RouteLookaheadStatus::kInvalidRoute,
            "route not bound to the requested final goal was accepted");
}

void OverflowAndLosProvenanceFailClosed() {
    auto route = Route();
    const double huge = std::numeric_limits<double>::max();
    route.path.poses = {{huge, 0, 0}, {-huge, 0, 0}};
    route.length = huge;
    Require(Select(route, 2).status == e::RouteLookaheadStatus::kInvalidRoute,
            "overflowing cumulative route length was accepted");

    auto line_of_sight = LineOfSight([](const n::Pose2d&, const n::Pose2d&) {
        return e::RouteLineOfSightStatus::kVisible;
    });
    line_of_sight.field_generation = 10;
    Require(Select(Route(), 2, &line_of_sight).status ==
                e::RouteLookaheadStatus::kLineOfSightUnavailable,
            "stale measured 3-D field generation was accepted");
    line_of_sight.field_generation = 11;
    line_of_sight.check = {};
    Require(Select(Route(), 2, &line_of_sight).status ==
                e::RouteLookaheadStatus::kLineOfSightUnavailable,
            "missing shared-geometry LOS checker failed open");
}

void CancellationFailsClosed() {
    Require(Select(Route(), 2, nullptr, [] { return true; }).status ==
                e::RouteLookaheadStatus::kCanceled,
            "canceled selection returned a waypoint");
    int checks = 0;
    Require(Select(Route(), 2, nullptr, [&checks] { return ++checks >= 5; })
                    .status == e::RouteLookaheadStatus::kCanceled,
            "final commit cancellation returned a waypoint");
}

}  // namespace

int main() {
    try {
        ArcLengthInterpolationAndEndpointYaw();
        LineOfSightStopsBeforeFailingNode();
        DuplicateEndpointCannotBypassLineOfSight();
        InvalidAndStaleRoutesFailClosed();
        OverflowAndLosProvenanceFailClosed();
        CancellationFailsClosed();
        std::cout << "route lookahead selector tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
