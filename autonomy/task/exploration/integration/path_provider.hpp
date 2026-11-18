/* Copyright 2026 The Openbot Authors. */
/** @file path_provider.hpp
 * @brief Consume one planner-owned graph for route costs and completion provenance.
 */
#pragma once
#include "autonomy/task/exploration/integration/map_reader.hpp"
#include "autonomy/planning/planner/vista/route_planner.hpp"
namespace autonomy::task::exploration {
/** @brief Evidence copied from the actual graph owner, never invented by exploration. */
struct RouteMapEvidence {
    std::uint64_t map_epoch=0,geometry_revision=0,watermark=0;
    perception::ObservationCut observation_cut{};
    perception::ObservationCut observation_baseline{}; ///< Excluded prefix of the actual graph's evidence.
    std::uint64_t observation_recovery_generation = 0; ///< Replacement baseline retained by graph ownership.
};
/** @brief Read-only route interface; task strategies never own a competing Vista graph. */
class PathProvider {
public:
    virtual ~PathProvider()=default;
    /** @brief Return actual route evidence; absence cannot satisfy completion. */
    virtual std::optional<RouteMapEvidence> Evidence() const { return {}; }
    /** @brief Query a route to an observation pose using the existing planner owner. */
    virtual planning::planner::vista::VistaRoute Route(const Pose2d& start,const Pose2d& goal) const=0;
    /** @brief Allow cancellation-aware providers without inventing a successful route. */
    virtual planning::planner::vista::VistaRoute Route(const Pose2d& start,const Pose2d& goal,
        const std::function<bool()>& canceled) const {
        if(canceled && canceled()) {
            planning::planner::vista::VistaRoute result;result.canceled=true;return result;
        }
        return Route(start,goal);
    }
    /** @brief Return route length in meters, or infinity when no route is available. */
    double Distance(const Pose2d& start,const Pose2d& goal) const {
        auto route=Route(start,goal);return route.success?route.length:std::numeric_limits<double>::infinity();
    }
};
}  // namespace autonomy::task::exploration
