/* Copyright 2026 The Openbot Authors. */
/** @file task_domain_path_provider.hpp
 * @brief Fail-closed task-domain validation for planner-owned Vista routes.
 */
#pragma once

#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "autonomy/task/exploration/integration/path_provider.hpp"
#include "autonomy/task/exploration/polygon_boundary.hpp"

namespace autonomy::task::exploration {

/**
 * @brief Decorates one Vista provider with exact task-domain route validation.
 * @details The delegate remains the sole owner of paths, costs and evidence.
 * This class only revokes success when a successful returned polyline is
 * malformed, leaves the allowed bounds or polygon, or contacts a closed no-go
 * polygon.
 */
class TaskDomainPathProvider final : public PathProvider
{
public:
    /** @brief Bind a route owner and immutable task-domain polygons.
     * @throws std::invalid_argument A delegate or polygon is invalid.
     */
    TaskDomainPathProvider(
        std::shared_ptr<PathProvider> delegate,
        std::optional<perception::Bounds2d> allowed_region,
        std::optional<::automsgs::msgs::geometry_msgs::Polygon> allowed_polygon,
        std::vector<::automsgs::msgs::geometry_msgs::Polygon> no_go_areas)
        : delegate_(std::move(delegate)),
          allowed_region_(std::move(allowed_region)),
          allowed_polygon_(std::move(allowed_polygon)),
          no_go_areas_(std::move(no_go_areas)) {
        if (!delegate_)
            throw std::invalid_argument("task-domain delegate is null");
        if (allowed_region_ &&
            (!std::isfinite(allowed_region_->min_x) ||
             !std::isfinite(allowed_region_->min_y) ||
             !std::isfinite(allowed_region_->max_x) ||
             !std::isfinite(allowed_region_->max_y) ||
             allowed_region_->min_x >= allowed_region_->max_x ||
             allowed_region_->min_y >= allowed_region_->max_y)) {
            throw std::invalid_argument(
                "task-domain allowed bounds are invalid");
        }
        if (allowed_polygon_ && !ValidatePolygonBoundary(*allowed_polygon_)) {
            throw std::invalid_argument(
                "task-domain allowed polygon is invalid");
        }
        for (const auto& polygon : no_go_areas_) {
            if (!ValidatePolygonBoundary(polygon)) {
                throw std::invalid_argument(
                    "task-domain no-go polygon is invalid");
            }
        }
    }

    /** @brief Preserve evidence supplied by the underlying Vista owner. */
    std::optional<RouteMapEvidence> Evidence() const override {
        return delegate_->Evidence();
    }

    /** @brief Query the delegate and revoke geometrically invalid success. */
    planning::planner::vista::VistaRoute Route(
        const Pose2d& start, const Pose2d& goal) const override {
        return Validate(start, goal, delegate_->Route(start, goal));
    }

    /** @brief Preserve delegate cancellation while enforcing the same domain.
     */
    planning::planner::vista::VistaRoute Route(
        const Pose2d& start, const Pose2d& goal,
        const std::function<bool()>& canceled) const override {
        return Validate(start, goal, delegate_->Route(start, goal, canceled));
    }

private:
    planning::planner::vista::VistaRoute Validate(
        const Pose2d& start, const Pose2d& goal,
        planning::planner::vista::VistaRoute route) const {
        if (!route.success)
            return route;
        if (!autonomy::navigation::IsFinite(start) ||
            !autonomy::navigation::IsFinite(goal) ||
            !std::isfinite(route.length) || route.length < 0) {
            route.success = false;
            return route;
        }
        Pose2d previous = start;
        for (const auto& pose : route.path.poses) {
            if (!autonomy::navigation::IsFinite(pose) ||
                !SegmentIsValid(previous, pose)) {
                route.success = false;
                return route;
            }
            previous = pose;
        }
        if (!SegmentIsValid(previous, goal))
            route.success = false;
        return route;
    }

    bool SegmentIsValid(const Pose2d& start, const Pose2d& goal) const {
        if (allowed_region_ && (!allowed_region_->Contains(start) ||
                                !allowed_region_->Contains(goal))) {
            return false;
        }
        if (allowed_polygon_ &&
            !polygon_boundary_internal::ValidatedPolygonContainsSegment(
                *allowed_polygon_, start.x, start.y, goal.x, goal.y)) {
            return false;
        }
        for (const auto& polygon : no_go_areas_) {
            if (!polygon_boundary_internal::ValidatedPolygonDisjointFromSegment(
                    polygon, start.x, start.y, goal.x, goal.y)) {
                return false;
            }
        }
        return true;
    }

    std::shared_ptr<PathProvider> delegate_;  ///< Sole path/evidence owner.
    std::optional<perception::Bounds2d>
        allowed_region_;  ///< Closed convex broad/legacy task domain.
    std::optional<::automsgs::msgs::geometry_msgs::Polygon>
        allowed_polygon_;  ///< Closed admissible task domain.
    std::vector<::automsgs::msgs::geometry_msgs::Polygon>
        no_go_areas_;  ///< Closed forbidden task domains.
};

}  // namespace autonomy::task::exploration
