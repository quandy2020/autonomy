/* Copyright 2026 The Openbot Authors. */
/** @file visit_order_optimizer.hpp
 * @brief Orders observation visits using only planner-owned Vista routes.
 */
#pragma once

#include "autonomy/task/exploration/integration/path_provider.hpp"
#include "autonomy/task/exploration/viewpoint/viewpoint.hpp"

namespace autonomy::task::exploration {

/** @brief Bounded directed visit-order improvement policy. */
struct VisitOrderOptimizerOptions {
    std::size_t maximum_nodes = 128;  ///< Hard candidate limit per call.
    std::size_t exact_maximum_nodes =
        12;  ///< Maximum candidates solved by exact directed Held-Karp (<=18).
    bool use_two_opt = true;  ///< Improve larger feasible orders by reversal.
    std::size_t improvement_iterations =
        16;  ///< Maximum accepted/search improvement passes.
};

/** @brief Feasible ordered visits and their immutable Vista route evidence. */
struct ExplorationRoute {
    std::vector<Viewpoint> views;
    /** Route at index i connects the preceding endpoint to views[i]. */
    std::vector<planning::planner::vista::VistaRoute> legs;
    /** Optional checked route from the final visit to the requested return
     * pose. */
    std::optional<planning::planner::vista::VistaRoute> return_leg;
    double distance = 0;
    double total_time = 0;
    bool reachable = true;
};

/** @brief Builds a feasible visit prefix without inventing route costs. */
class VisitOrderOptimizer
{
public:
    /** @brief Order reachable viewpoints using directed Vista route costs.
     * @param start Measured route origin.
     * @param viewpoints Candidate observation endpoints.
     * @param paths Sole owner of route paths, costs, and provenance.
     * @param nominal_speed Positive translational speed in meters per second.
     * @param angular_speed Positive yaw speed in radians per second.
     * @param return_pose Optional required checked endpoint after the last
     * visit.
     * @param keep_first Prefer the first candidate only when its directed legs
     * are feasible.
     * @param cancel_checker Optional cooperative cancellation predicate.
     * @param options Bounded node and directed-order improvement policy.
     * @return A feasible nonempty prefix, or reachable=false when none exists.
     */
    ExplorationRoute Optimize(const Pose2d& start,
                              const std::vector<Viewpoint>& viewpoints,
                              const PathProvider& paths, double nominal_speed,
                              double angular_speed,
                              const std::optional<Pose2d>& return_pose = {},
                              bool keep_first = false,
                              const std::function<bool()>& cancel_checker = {},
                              VisitOrderOptimizerOptions options = {}) const;
};

}  // namespace autonomy::task::exploration
