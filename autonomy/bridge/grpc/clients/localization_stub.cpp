/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/clients/localization_stub.hpp"
#include "autonomy/bridge/grpc/rpc_status.hpp"
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {
namespace {

namespace localization_rpc = ::automsgs::rpcs::localization;
namespace task_proto = ::autonomy::task::proto;
using StatusCode = ::automsgs::msgs::status_msgs::StatusCode;

constexpr char kAmclPoseChannel[] = "/amcl_pose";

bool CheckLocalizationTerminalStatus(task_proto::LocalizationStatus status) {
    return status == task_proto::LOCALIZATION_STATUS_SUCCEEDED ||
           status == task_proto::LOCALIZATION_STATUS_FAILED ||
           status == task_proto::LOCALIZATION_STATUS_CANCELED ||
           status == task_proto::LOCALIZATION_STATUS_IDLE;
}

}  // namespace

LocalizationTraits::Goal LocalizationTraits::ConvertToGoal(
    const Request& request) {
    Goal goal;
    goal.set_command(task_proto::LOCALIZATION_CMD_SET_INITIAL_POSE);
    auto* initial_pose = goal.mutable_initial_pose();
    *initial_pose->mutable_pose() = request.pose();
    if (request.pose().has_pose() && request.pose().pose().has_header()) {
        *initial_pose->mutable_header() = request.pose().pose().header();
    }
    return goal;
}

LocalizationTraits::Response LocalizationTraits::ConvertFromFeedback(
    const Feedback& feedback, const Request& /*last*/) {
    Response status;
    *status.mutable_status() = OkStatus();
    switch (feedback.status()) {
        case task_proto::LOCALIZATION_STATUS_RUNNING:
            status.set_state(localization_rpc::LOCALIZATION_STATE_LOCALIZED);
            break;
        case task_proto::LOCALIZATION_STATUS_INITIALIZING:
            status.set_state(
                localization_rpc::LOCALIZATION_STATE_INITIALIZING);
            break;
        case task_proto::LOCALIZATION_STATUS_FAILED:
            status.set_state(localization_rpc::LOCALIZATION_STATE_LOST);
            break;
        default:
            status.set_state(localization_rpc::LOCALIZATION_STATE_UNKNOWN);
            break;
    }
    if (feedback.localization_quality() > 0.f) {
        status.set_confidence(feedback.localization_quality());
    }
    return status;
}

LocalizationTraits::Response LocalizationTraits::MakeResponse(
    const Request& /*request*/, bool success, bool /*final*/,
    const std::string& message) {
    Response status;
    *status.mutable_status() =
        success ? OkStatus(message)
                : ErrorStatus(StatusCode::INVALID_ARGUMENT, message);
    status.set_state(success
                         ? localization_rpc::LOCALIZATION_STATE_INITIALIZING
                         : localization_rpc::LOCALIZATION_STATE_UNKNOWN);
    return status;
}

bool LocalizationTraits::IsTerminal(const Feedback& feedback) {
    return CheckLocalizationTerminalStatus(feedback.status());
}

LocalizationStub::LocalizationStub(std::shared_ptr<autolink::Node> node,
                                   TaskMuxer::SharedPtr muxer)
    : GoalChannelCommandStub(node, std::move(muxer)), node_(std::move(node)) {
    channel_.SetFeedbackHook(
        [this](const LocalizationTraits::Feedback& feedback) {
            HandleFeedback(feedback);
        });
    if (!node_) {
        return;
    }
    pose_cache_.BindReader(node_, kAmclPoseChannel, [this](const PoseMsg&) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (state_ == localization_rpc::LOCALIZATION_STATE_UNKNOWN) {
            state_ = localization_rpc::LOCALIZATION_STATE_LOCALIZED;
        }
    });
}

void LocalizationStub::HandleFeedback(
    const LocalizationTraits::Feedback& feedback) {
    std::lock_guard<std::mutex> lock(mutex_);
    quality_ = feedback.localization_quality();
    switch (feedback.status()) {
        case task_proto::LOCALIZATION_STATUS_RUNNING:
            state_ = localization_rpc::LOCALIZATION_STATE_LOCALIZED;
            break;
        case task_proto::LOCALIZATION_STATUS_INITIALIZING:
            state_ = localization_rpc::LOCALIZATION_STATE_INITIALIZING;
            break;
        case task_proto::LOCALIZATION_STATUS_FAILED:
            state_ = localization_rpc::LOCALIZATION_STATE_LOST;
            break;
        default:
            state_ = localization_rpc::LOCALIZATION_STATE_UNKNOWN;
            break;
    }
}

localization_rpc::GetPoseResponse LocalizationStub::GetPose(
    const localization_rpc::GetPoseRequest& request) const {
    localization_rpc::GetPoseResponse response;
    const auto pose = pose_cache_.GetLatestMessage();
    if (!pose || !pose->has_pose()) {
        *response.mutable_status() = ErrorStatus(
            StatusCode::LOCALIZATION_UNAVAILABLE, "pose unavailable");
        return response;
    }
    *response.mutable_pose() = pose->pose();
    if (!request.map_frame_id().empty() && response.pose().has_pose()) {
        response.mutable_pose()->mutable_pose()->mutable_header()->set_frame_id(
            request.map_frame_id());
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (quality_ > 0.f) {
            response.set_confidence(quality_);
        }
    }
    *response.mutable_status() = OkStatus();
    return response;
}

localization_rpc::LocalizationStatus LocalizationStub::GetStatus() const {
    localization_rpc::LocalizationStatus status;
    std::lock_guard<std::mutex> lock(mutex_);
    *status.mutable_status() = OkStatus();
    status.set_state(state_);
    if (quality_ > 0.f) {
        status.set_confidence(quality_);
    }
    return status;
}

::automsgs::rpcs::common::Status LocalizationStub::SetInitialPose(
    const localization_rpc::SetInitialPoseRequest& request) {
    if (!request.has_pose()) {
        return ErrorStatus(StatusCode::INVALID_ARGUMENT, "pose required");
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        state_ = localization_rpc::LOCALIZATION_STATE_INITIALIZING;
    }
    const auto goal = LocalizationTraits::ConvertToGoal(request);
    if (!channel_.WriteGoal(goal)) {
        return ErrorStatus(StatusCode::INTERNAL, "failed to publish goal");
    }
    return OkStatus("initial pose set");
}

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
