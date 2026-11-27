/* Copyright 2026 The Openbot Authors. */
/**
 * @file route_lookahead_selector.hpp
 * @brief Provenance-checked arc-length lookahead over immutable Vista routes.
 */
#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <string_view>

#include "autonomy/planning/planner/vista/route_planner.hpp"

namespace autonomy::task::exploration {

/** @brief Explicit result of a shared measured-geometry LOS query. */
enum class RouteLineOfSightStatus {
    kVisible,
    kBlocked,
    kUnavailable,
    kCanceled,
};

/**
 * @brief Provenance-bound immutable LOS query view.
 * @details Its checker is produced by the shared planar swept-body and rolling
 * 3-D field adapter; generation identifies that immutable field snapshot.
 */
struct RouteLineOfSightView {
    std::uint64_t map_epoch = 0;
    std::uint64_t geometry_revision = 0;
    std::uint64_t field_generation = 0;
    std::string frame_id;
    std::function<RouteLineOfSightStatus(const autonomy::navigation::Pose2d&,
                                         const autonomy::navigation::Pose2d&)>
        check;
};

/** @brief Fail-closed route-lookahead outcome. */
enum class RouteLookaheadStatus {
    kSelected,
    kInvalidRoute,
    kStaleRoute,
    kMissingEvidence,
    kLineOfSightUnavailable,
    kCanceled,
};

/** @brief Immutable selection result; it never certifies route execution. */
struct RouteLookaheadResult {
    RouteLookaheadStatus status = RouteLookaheadStatus::kInvalidRoute;
    autonomy::navigation::Pose2d pose;
    double route_arc_length = 0;  ///< Arc length from the exact route start.
    bool final_endpoint = false;  ///< True only for the exact requested goal.
    bool line_of_sight_limited = false;
};

/**
 * @brief Selects a bounded lookahead without owning a route graph.
 * @details A failing next route node clamps selection to the preceding visible
 * node. Missing or mismatched measured-geometry provenance always fails closed.
 */
class RouteLookaheadSelector
{
public:
    /**
     * @brief Select by polyline arc length after strict Vista provenance
     * checks.
     * @param route Immutable route produced by the sole Vista graph owner.
     * @param start Exact measured start bound to this immutable route leg.
     * @param goal Exact final goal bound to this immutable route leg.
     * @param lookahead_distance Positive desired route arc length in meters.
     * @param expected_epoch Current accepted map epoch.
     * @param expected_geometry_revision Current accepted geometry revision.
     * @param expected_frame Current accepted world frame.
     * @param expected_field_generation Current immutable 3-D field generation;
     * required and nonzero only when LOS is enabled.
     * @param line_of_sight Null to disable LOS; otherwise provenance must
     * match.
     * @param canceled Optional cancellation check; never called under a map
     * lock.
     */
    RouteLookaheadResult Select(
        const planning::planner::vista::VistaRoute& route,
        const autonomy::navigation::Pose2d& start,
        const autonomy::navigation::Pose2d& goal, double lookahead_distance,
        std::uint64_t expected_epoch, std::uint64_t expected_geometry_revision,
        std::string_view expected_frame,
        std::uint64_t expected_field_generation = 0,
        const RouteLineOfSightView* line_of_sight = nullptr,
        const std::function<bool()>& canceled = {}) const;
};

}  // namespace autonomy::task::exploration
