/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/clients/localization_stub.hpp"

#include "autonomy/bridge/grpc/rpc_convert.hpp"
#include "autonomy/bridge/proto/external_command_service.pb.h"
#include "autonomy/common/logging.hpp"
#include "autonomy/task/common/names.hpp"
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {
namespace {

using StatusCode = ::automsgs::msgs::status_msgs::StatusCode;
namespace localization_rpc = ::automsgs::rpcs::localization;
constexpr char kAmclPoseChannel[] = "/amcl_pose";

}  // namespace

LocalizationStub::LocalizationStub(std::shared_ptr<autolink::Node> node,
                                   std::shared_ptr<MapStub> map_stub)
    : map_stub_(std::move(map_stub)) {
    if (!node) {
        return;
    }
    feedback_cache_.BindReader(
        node, ::autonomy::task::kLocalizationFeedback,
        [this](const FeedbackMsg& feedback) { HandleFeedback(feedback); });
    pose_cache_.BindReader(node, kAmclPoseChannel, [this](const PoseMsg&) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (state_ == localization_rpc::LOCALIZATION_STATE_UNKNOWN) {
            state_ = localization_rpc::LOCALIZATION_STATE_LOCALIZED;
        }
    });
}

void LocalizationStub::HandleFeedback(const FeedbackMsg& feedback) {
    std::lock_guard<std::mutex> lock(mutex_);
    quality_ = feedback.localization_quality();
    switch (feedback.status()) {
        case ::autonomy::task::proto::LOCALIZATION_STATUS_RUNNING:
            state_ = localization_rpc::LOCALIZATION_STATE_LOCALIZED;
            break;
        case ::autonomy::task::proto::LOCALIZATION_STATUS_INITIALIZING:
            state_ = localization_rpc::LOCALIZATION_STATE_INITIALIZING;
            break;
        case ::autonomy::task::proto::LOCALIZATION_STATUS_FAILED:
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
    const auto pose = pose_cache_.GetLatest();
    if (!pose || !pose->has_pose()) {
        *response.mutable_status() = MakeRpcStatus(
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
    *response.mutable_status() = MakeOkStatus();
    return response;
}

localization_rpc::LocalizationStatus LocalizationStub::GetStatus() const {
    localization_rpc::LocalizationStatus status;
    std::lock_guard<std::mutex> lock(mutex_);
    *status.mutable_status() = MakeOkStatus();
    status.set_state(state_);
    if (quality_ > 0.f) {
        status.set_confidence(quality_);
    }
    return status;
}

::automsgs::rpcs::common::Status LocalizationStub::SetInitialPose(
    const localization_rpc::SetInitialPoseRequest& request) {
    if (!map_stub_ || !request.has_pose()) {
        return MakeRpcStatus(StatusCode::INVALID_ARGUMENT, "pose required");
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        state_ = localization_rpc::LOCALIZATION_STATE_INITIALIZING;
    }
    proto::MapCommandRequest bridge;
    bridge.set_command(proto::MAP_CMD_SET_INITIAL_POSE);
    auto* initial_pose = bridge.mutable_initial_pose();
    *initial_pose->mutable_pose() = request.pose();
    if (request.pose().has_pose() && request.pose().pose().has_header()) {
        *initial_pose->mutable_header() = request.pose().pose().header();
    }
    map_stub_->HandleCommand(bridge, [](const auto&) {});
    return MakeOkStatus("initial pose set");
}

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
