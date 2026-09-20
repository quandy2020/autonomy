/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file navigation_traits.hpp
 * @brief Navigator Action CRTP leaves (ToPose / ThroughPoses).
 *
 * @details
 * Shared NavigatorActionBase supplies CmdId / MakeReject / MakeResult /
 * MakeAcceptFailure; leaf traits supply ConvertToGoal and feedback frames.
 * Used by ActionBackgroundInterface / RunAction on worker threads when an
 * Action-backed navigation path is selected (distinct from GoalChannel
 * NavigatorStub).
 *
 * @par Ownership
 * Stateless policy types; NodeClient ownership lives with the caller.
 *
 * @par Threading
 * Convert / Make* are called from ActionBackground worker / callback paths.
 *
 * @note Used by ActionBackgroundInterface / RunAction on worker threads.
 *
 * @see action_pack.hpp
 * @see ActionBackgroundInterface
 */

#pragma once

#include <algorithm>
#include <chrono>
#include <string>
#include "autolink/action/types.hpp"
#include "autonomy/bridge/grpc/clients/action_background_interface.hpp"
#include "autonomy/bridge/grpc/clients/action_send.hpp"
#include "autonomy/bridge/grpc/rpc_status.hpp"
#include "autonomy/bridge/node_client.hpp"
#include "autonomy/common/macros.hpp"
#include <automsgs/actions/nav_actions.pb.h>
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>
#include <automsgs/rpcs/navigation.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

namespace navigator {
namespace {

/**
 * @brief Alias for status_msgs StatusCode used in MakeFrame.
 */
using StatusCode = ::automsgs::msgs::status_msgs::StatusCode;

/**
 * @brief Shorthand for navigation RPC protobuf namespace.
 */
namespace nav_rpc = ::automsgs::rpcs::navigation;

/**
 * @brief Map action ResultCode to NavigationState for terminal frames.
 *
 * @param[in] code Action result code.
 * @return         ARRIVED / CANCELLED / FAILED / UNKNOWN.
 */
inline nav_rpc::NavigationState ResolveNavigationState(
    autolink::action::ResultCode code) {
    switch (code) {
        case autolink::action::ResultCode::SUCCEEDED:
            return nav_rpc::NAVIGATION_STATE_ARRIVED;
        case autolink::action::ResultCode::CANCELED:
            return nav_rpc::NAVIGATION_STATE_CANCELLED;
        case autolink::action::ResultCode::ABORTED:
            return nav_rpc::NAVIGATION_STATE_FAILED;
        default:
            return nav_rpc::NAVIGATION_STATE_UNKNOWN;
    }
}

/**
 * @brief Build a NavigateResponse with status overlay.
 *
 * @param[in] request         Source navigate request (goal_id / waypoints).
 * @param[in] state           NavigationState to stamp.
 * @param[in] ok              Success bit for status.
 * @param[in] message         Optional detail text.
 * @param[in] total_waypoints Waypoint count override (0 → request size).
 * @return                    Populated NavigateResponse.
 */
inline nav_rpc::NavigateResponse MakeFrame(
    const nav_rpc::NavigateRequest& request, nav_rpc::NavigationState state,
    bool ok, const std::string& message = "", int total_waypoints = 0) {
    nav_rpc::NavigateResponse response;
    response.set_goal_id(request.goal_id());
    response.set_state(state);
    response.set_number_of_waypoints(total_waypoints > 0 ? total_waypoints
                                                         : request.waypoints_size());
    *response.mutable_status() =
        ok ? OkStatus(message)
           : ErrorStatus(StatusCode::TASK_FAILED, message);
    return response;
}

/**
 * @brief Human-readable accept-failure detail for a server name.
 *
 * @param[in] status      Accept outcome (not-ready / timeout / rejected).
 * @param[in] server_name Action server name fragment.
 * @return                Detail string for MakeAcceptFailure.
 */
inline std::string DescribeAcceptFailure(ActionAcceptStatus status,
                                         const char* server_name) {
    switch (status) {
        case ActionAcceptStatus::kServerNotReady:
            return std::string(server_name) + " action server is not ready";
        case ActionAcceptStatus::kTimeout:
            return std::string("timeout waiting for ") + server_name +
                   " goal acceptance";
        case ActionAcceptStatus::kRejected:
            return std::string(server_name) + " goal rejected";
        default:
            return std::string(server_name) + " goal failed";
    }
}

}  // namespace

/**
 * @brief Shared navigator Action CRTP layer (ids / reject / result / accept-fail).
 *
 * @tparam Derived NavigateToPose / NavigateThroughPoses leaf.
 *
 * @note kEmitAfterAccept is true: after accept, MakeAfterAccept emits RUNNING.
 */
template <typename Derived>
struct NavigatorActionBase : ActionBackgroundInterface<Derived> {
    AUTONOMY_SMART_PTR_DEFINITIONS(NavigatorActionBase<Derived>)

    /**
     * @brief Navigate RPC request type.
     */
    using Request = nav_rpc::NavigateRequest;

    /**
     * @brief Navigate RPC / stream response type.
     */
    using Response = nav_rpc::NavigateResponse;

    static constexpr bool kEmitAfterAccept = true;
    static constexpr auto kAcceptTimeout = std::chrono::seconds(30);

    /**
     * @brief Extract command id from the navigate request.
     *
     * @param[in] request Navigate RPC request.
     * @return            request.goal_id().
     */
    std::string CmdId(const Request& request) const {
        return request.goal_id();
    }

    /**
     * @brief Client id for session bookkeeping (unused → empty).
     *
     * @return Empty string.
     */
    std::string ClientId(const Request&) const { return {}; }

    /**
     * @brief Build a FAILED reject frame.
     *
     * @param[in] request Source request.
     * @param[in] message Detail text.
     * @return            NavigateResponse with FAILED state.
     */
    Response MakeReject(const Request& request,
                             const std::string& message) const {
        return MakeFrame(request, nav_rpc::NAVIGATION_STATE_FAILED, false,
                         message);
    }

    /**
     * @brief Map wrapped action result to a terminal NavigateResponse.
     *
     * @tparam WrappedResult Action GoalHandle::WrappedResult (deduced; avoids
     * naming incomplete Derived::Client while the CRTP base is instantiated).
     * @param[in] request Source request.
     * @param[in] wrapped Action WrappedResult.
     * @return            Terminal response with arrived / cancelled / failed state.
     */
    template <typename WrappedResult>
    Response MakeResult(const Request& request,
                        const WrappedResult& wrapped) const {
        const auto state = ResolveNavigationState(wrapped.code);
        const bool ok =
            wrapped.code == autolink::action::ResultCode::SUCCEEDED;
        std::string message;
        if (wrapped.result && !wrapped.result->error_msg().empty()) {
            message = wrapped.result->error_msg();
        }
        return MakeFrame(request, state, ok, message);
    }

    /**
     * @brief Build a reject frame for accept timeout / rejection.
     *
     * @param[in] request Source request.
     * @param[in] status  Accept failure status.
     * @return            NavigateResponse describing the accept failure.
     */
    Response MakeAcceptFailure(const Request& request,
                                    ActionAcceptStatus status) const {
        return MakeFrame(
            request, nav_rpc::NAVIGATION_STATE_FAILED, false,
            DescribeAcceptFailure(status, this->impl().ServerName()));
    }
};

/**
 * @brief NavigateToPose action policy (CRTP leaf).
 *
 * Converts the first waypoint into a single-pose action goal and maps
 * distance_remaining feedback onto NavigateResponse.
 */
struct NavigateToPoseTraits : NavigatorActionBase<NavigateToPoseTraits> {
    AUTONOMY_SMART_PTR_DEFINITIONS(NavigateToPoseTraits)

    /**
     * @brief NodeClient for NavigateToPoseAction.
     */
    using Client = NodeClient<::automsgs::actions::NavigateToPoseAction>;

    /**
     * @brief Navigate RPC request type.
     */
    using Request = nav_rpc::NavigateRequest;

    /**
     * @brief Navigate RPC / stream response type.
     */
    using Response = nav_rpc::NavigateResponse;

    /**
     * @brief Action goal type from Client.
     */
    using Goal = typename Client::Goal;

    /**
     * @brief Action feedback type from Client.
     */
    using Feedback = typename Client::Feedback;

    /**
     * @brief Action server name for NavigateToPose.
     *
     * @return C-string server name used in accept-failure messages.
     */
    const char* ServerName() const { return "navigate_to_pose"; }

    /**
     * @brief Reject detail when the action server is not ready.
     *
     * @return Human-readable not-ready message.
     */
    const char* ServerNotReadyMessage() const {
        return "navigate_to_pose action server is not ready";
    }

    /**
     * @brief Convert navigate request to a NavigateToPose goal.
     *
     * @param[in] request Must contain at least one waypoint.
     * @return            Goal with pose = waypoints(0).
     */
    Goal ConvertToGoal(const Request& request) const {
        Goal goal;
        *goal.mutable_pose() = request.waypoints(0);
        return goal;
    }

    /**
     * @brief Immediate accept ACK (PLANNING, 1 waypoint).
     */
    Response MakeAccept(const Request& request) const {
        return MakeFrame(request, nav_rpc::NAVIGATION_STATE_PLANNING, true,
                         "accepted", 1);
    }

    /**
     * @brief Post-accept RUNNING frame (kEmitAfterAccept).
     */
    Response MakeAfterAccept(const Request& request) const {
        return MakeFrame(request, nav_rpc::NAVIGATION_STATE_RUNNING, true, "",
                         1);
    }

    /**
     * @brief Map action feedback to a RUNNING NavigateResponse.
     *
     * @param[in] request  Source request.
     * @param[in] feedback Action feedback (pose + distance).
     * @return             Stream feedback frame.
     */
    Response MakeFeedback(const Request& request,
                               const Feedback& feedback) const {
        auto response = MakeFrame(request, nav_rpc::NAVIGATION_STATE_RUNNING,
                                  true, "", 1);
        *response.mutable_current_pose() = feedback.current_pose();
        response.set_remaining_distance_meters(feedback.distance_remaining());
        response.set_waypoint_index(0);
        return response;
    }
};

/**
 * @brief NavigateThroughPoses action policy (CRTP leaf).
 *
 * Copies all request waypoints into the multi-pose action goal and derives
 * waypoint_index from number_of_poses_remaining.
 */
struct NavigateThroughPosesTraits
    : NavigatorActionBase<NavigateThroughPosesTraits> {
    AUTONOMY_SMART_PTR_DEFINITIONS(NavigateThroughPosesTraits)

    /**
     * @brief NodeClient for NavigateThroughPosesAction.
     */
    using Client = NodeClient<::automsgs::actions::NavigateThroughPosesAction>;

    /**
     * @brief Navigate RPC request type.
     */
    using Request = nav_rpc::NavigateRequest;

    /**
     * @brief Navigate RPC / stream response type.
     */
    using Response = nav_rpc::NavigateResponse;

    /**
     * @brief Action goal type from Client.
     */
    using Goal = typename Client::Goal;

    /**
     * @brief Action feedback type from Client.
     */
    using Feedback = typename Client::Feedback;

    /**
     * @brief Action server name for NavigateThroughPoses.
     *
     * @return C-string server name used in accept-failure messages.
     */
    const char* ServerName() const { return "navigate_through_poses"; }

    /**
     * @brief Reject detail when the action server is not ready.
     *
     * @return Human-readable not-ready message.
     */
    const char* ServerNotReadyMessage() const {
        return "navigate_through_poses action server is not ready";
    }

    /**
     * @brief Convert navigate request to a NavigateThroughPoses goal.
     *
     * @param[in] request Waypoint list copied into goal.poses.
     * @return            Multi-pose action goal.
     */
    Goal ConvertToGoal(const Request& request) const {
        Goal goal;
        goal.mutable_poses()->Reserve(request.waypoints_size());
        for (const auto& pose : request.waypoints()) {
            *goal.add_poses() = pose;
        }
        return goal;
    }

    /**
     * @brief Immediate accept ACK (PLANNING, N waypoints).
     */
    Response MakeAccept(const Request& request) const {
        return MakeFrame(request, nav_rpc::NAVIGATION_STATE_PLANNING, true,
                         "accepted", request.waypoints_size());
    }

    /**
     * @brief Post-accept RUNNING frame (kEmitAfterAccept).
     */
    Response MakeAfterAccept(const Request& request) const {
        return MakeFrame(request, nav_rpc::NAVIGATION_STATE_RUNNING, true, "",
                         request.waypoints_size());
    }

    /**
     * @brief Map multi-pose feedback to a RUNNING NavigateResponse.
     *
     * @param[in] request  Source request.
     * @param[in] feedback Action feedback (pose + remaining poses).
     * @return             Stream feedback frame with waypoint_index.
     */
    Response MakeFeedback(const Request& request,
                               const Feedback& feedback) const {
        const int total_waypoints = request.waypoints_size();
        auto response =
            MakeFrame(request, nav_rpc::NAVIGATION_STATE_RUNNING, true, "",
                      total_waypoints);
        *response.mutable_current_pose() = feedback.current_pose();
        response.set_remaining_distance_meters(feedback.distance_remaining());
        response.set_waypoint_index(total_waypoints -
                                    feedback.number_of_poses_remaining());
        return response;
    }
};

}  // namespace navigator

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
