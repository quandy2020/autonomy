/* Copyright 2026 The Openbot Authors. */
#pragma once
#include "autonomy/task/exploration/integration/map_reader.hpp"
#include "autonomy/planning/planner/vista/route_planner.hpp"
namespace autonomy::task::exploration {
struct RouteMapEvidence {
    std::uint64_t map_epoch=0,geometry_revision=0,watermark=0;
    perception::ObservationCut observation_cut{};
};
class PathProvider {
public:
    virtual ~PathProvider()=default;
    virtual std::optional<RouteMapEvidence> Evidence() const { return {}; }
    virtual planning::planner::vista::VistaRoute Route(const Pose2d& start,const Pose2d& goal) const=0;
    virtual planning::planner::vista::VistaRoute Route(const Pose2d& start,const Pose2d& goal,
        const std::function<bool()>& canceled) const {
        if(canceled && canceled()) {
            planning::planner::vista::VistaRoute result;result.canceled=true;return result;
        }
        return Route(start,goal);
    }
    double Distance(const Pose2d& start,const Pose2d& goal) const {
        auto route=Route(start,goal);return route.success?route.length:std::numeric_limits<double>::infinity();
    }
};
}  // namespace autonomy::task::exploration
