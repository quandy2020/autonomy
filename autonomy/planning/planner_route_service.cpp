/* Copyright 2026 The Openbot Authors. */
/** @file planner_route_service.cpp
 * @brief Exact route queries over the server's configured persistent Vista graph.
 */
#include "autonomy/planning/planner_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include "autonomy/planning/planner/vista/native_route_protocol.hpp"
#include "autonomy/planning/planner/vista/vista_planner.hpp"

namespace autonomy::planning {
namespace {
/** @brief Map native plugin errors to explicit, non-default wire statuses. */
automsgs::srvs::GetPlan::Status QueryStatus(std::uint32_t code) {
    using Query = automsgs::srvs::GetPlan;
    switch (code) {
        case proto::PLANNER_SUCCESS: return Query::SUCCESS;
        case proto::PLANNER_NOT_INITIALIZED: return Query::UNAVAILABLE;
        case proto::PLANNER_CANCELED: return Query::CANCELED;
        case proto::PLANNER_INVALID_START:
        case proto::PLANNER_INVALID_GOAL:
        case proto::PLANNER_TF_ERROR: return Query::INVALID_REQUEST;
        case proto::PLANNER_BLOCKED_START:
        case proto::PLANNER_BLOCKED_GOAL:
        case proto::PLANNER_NO_PATH_FOUND:
        case proto::PLANNER_EMPTY_PATH: return Query::NO_ROUTE;
        default: return Query::INTERNAL_ERROR;
    }
}
}  // namespace

void PlannerServer::QueryRoute(const GetPlanRequest& request, GetPlanResponse& response) {
    using Query = automsgs::srvs::GetPlan;
    response.Clear();
    response.set_request_id(request.request_id());
    const auto id = request.planner_id().empty() ? default_planner_id_ : request.planner_id();
    response.set_planner_id(id);
    response.set_status(Query::INVALID_REQUEST);
    if (request.request_id().size() > 256 || id.size() > 256 ||
        !std::isfinite(request.tolerance()) || request.tolerance() < 0 ||
        ((request.require_graph_evidence() || request.evidence_only()) &&
         (request.request_id().empty() || request.tolerance() != 0))) return;
    const auto configured = GetPlanner(id);
    if (!configured) { response.set_status(Query::UNSUPPORTED); return; }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(
        request.timeout_ms() ? std::min(request.timeout_ms(), 5000U) : 1000U);
    auto canceled = [&] { return std::chrono::steady_clock::now() >= deadline; };
    try {
        const auto vista = std::dynamic_pointer_cast<planner::vista::VistaPlanner>(configured);
        if (!vista) {
            if (request.require_graph_evidence() || request.evidence_only()) {
                response.set_status(Query::UNSUPPORTED); return;
            }
            // Legacy requests can consume the existing navigation path, without a certificate.
            *response.mutable_plan() = GetPlan(request.start(), request.goal(), id, canceled);
            if (canceled()) { response.clear_plan(); response.set_status(Query::CANCELED); return; }
            response.set_status(Query::SUCCESS); return;
        }
        planner::vista::VistaRoute route;
        const auto code = vista->QueryRoute(request.start(), request.goal(), route,
                                            request.evidence_only(), canceled);
        if (canceled()) { response.set_status(Query::CANCELED); return; }
        PublishVistaGraph(id);
        response.set_status(QueryStatus(code));
        if (code != proto::PLANNER_SUCCESS) {
            if (response.status() == Query::NO_ROUTE && route.evidence)
                planner::vista::EncodeEvidence(*route.evidence, response.mutable_graph_evidence());
            return;
        }
        if (!route.evidence) { response.set_status(Query::INTERNAL_ERROR); return; }
        planner::vista::EncodeEvidence(*route.evidence, response.mutable_graph_evidence());
        if (request.evidence_only()) return;
        auto* path = response.mutable_plan();
        path->mutable_header()->CopyFrom(request.start().header());
        path->mutable_header()->set_frame_id(route.path.frame_id);
        for (const auto& pose : route.path.poses) {
            auto* native = path->add_poses();
            *native = planner::vista::RoutePose(pose, route.path.frame_id);
            native->mutable_header()->CopyFrom(path->header());
            native->mutable_pose()->mutable_position()->set_z(request.start().pose().position().z());
        }
        // Retain exact requested endpoints, including the final observation orientation.
        path->mutable_poses(0)->mutable_pose()->CopyFrom(request.start().pose());
        path->mutable_poses(path->poses_size() - 1)->mutable_pose()->CopyFrom(request.goal().pose());
        response.set_length(route.length);
        response.set_used_grid_fallback(route.used_grid_fallback);
    } catch (const std::exception& error) {
        response.clear_plan(); response.clear_graph_evidence();
        response.set_status(Query::INTERNAL_ERROR); response.set_error_msg(error.what());
    } catch (...) {
        response.clear_plan(); response.clear_graph_evidence(); response.set_status(Query::INTERNAL_ERROR);
    }
}

}  // namespace autonomy::planning
