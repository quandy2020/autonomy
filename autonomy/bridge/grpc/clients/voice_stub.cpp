/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/clients/voice_stub.hpp"

#include "autonomy/bridge/grpc/clients/stub_util.hpp"
#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

VoiceStub::VoiceStub(std::shared_ptr<TaskMuxer> muxer,
                     clients::NavigatorStub* navigator,
                     clients::FollowStub* follow, clients::DockStub* dock,
                     clients::ExplorationStub* exploration)
    : muxer_(std::move(muxer)),
      navigator_(navigator),
      follow_(follow),
      dock_(dock),
      exploration_(exploration) {}

proto::VoiceCommandResponse VoiceStub::MakeResponse(
    const proto::VoiceCommandRequest& request, const proto::VoiceStatus status,
    const bool success, const bool final, const std::string& message) const {
    proto::VoiceCommandResponse response;
    response.set_status(status);
    response.set_intent(request.intent());
    const proto::TaskStatus task_status =
        success ? (final ? proto::TASK_STATUS_SUCCEEDED
                         : proto::TASK_STATUS_RUNNING)
                : proto::TASK_STATUS_FAILED;
    FillCommandAck(response, proto::TASK_TYPE_NONE, request, success, final,
                   task_status, message);
    if (!message.empty()) {
        response.set_detail(message);
    }
    return response;
}

void VoiceStub::CancelActiveSession() {
    if (navigator_ && navigator_->CheckNavigating()) {
        proto::NavigationCommandRequest cancel;
        cancel.set_command(proto::NAV_CMD_CANCEL);
        navigator_->HandleCommand(cancel, [](const auto&) {});
    }
    if (follow_) {
        follow_->CancelActiveSession();
    }
    if (dock_) {
        dock_->CancelActiveSession();
    }
    if (exploration_) {
        exploration_->CancelActiveSession();
    }
    if (muxer_) {
        muxer_->Clear();
    }
}

bool VoiceStub::HandleCommand(const proto::VoiceCommandRequest& request,
                              StreamCallback stream_callback) {
    if (!stream_callback) {
        return false;
    }
    if (muxer_ && muxer_->CheckEstopActive()) {
        stream_callback(MakeResponse(request, proto::VOICE_STATUS_FAILED, false,
                                     true, "emergency stop active"));
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_request_ = request;
        stream_callback_ = stream_callback;
    }

    stream_callback(
        MakeResponse(request, proto::VOICE_STATUS_DISPATCHING, true, false));

    const auto intent = request.intent();
    bool accepted = false;
    std::string detail;

    switch (intent) {
        case proto::VOICE_INTENT_NAVIGATE: {
            if (!navigator_ || !request.has_navigate()) {
                detail = "navigate payload / navigator unavailable";
                break;
            }
            accepted = navigator_->HandleCommand(
                request.navigate(),
                [stream_callback, request](
                    const proto::NavigationCommandResponse& navigation_response) {
                    proto::VoiceCommandResponse voice;
                    voice.set_intent(request.intent());
                    voice.set_status(navigation_response.ack().final()
                                         ? (navigation_response.ack().success()
                                                ? proto::VOICE_STATUS_SUCCEEDED
                                                : proto::VOICE_STATUS_FAILED)
                                         : proto::VOICE_STATUS_RUNNING);
                    *voice.mutable_ack() = navigation_response.ack();
                    voice.set_detail(navigation_response.ack().message());
                    stream_callback(voice);
                });
            break;
        }
        case proto::VOICE_INTENT_FOLLOW: {
            if (!follow_ || !request.has_follow()) {
                detail = "follow payload / stub unavailable";
                break;
            }
            accepted = follow_->HandleCommand(
                request.follow(),
                [stream_callback, request](
                    const proto::FollowCommandResponse& follow_response) {
                    proto::VoiceCommandResponse voice;
                    voice.set_intent(request.intent());
                    voice.set_status(follow_response.ack().final()
                                         ? (follow_response.ack().success()
                                                ? proto::VOICE_STATUS_SUCCEEDED
                                                : proto::VOICE_STATUS_FAILED)
                                         : proto::VOICE_STATUS_RUNNING);
                    *voice.mutable_ack() = follow_response.ack();
                    voice.set_detail(follow_response.ack().message());
                    stream_callback(voice);
                });
            break;
        }
        case proto::VOICE_INTENT_DOCK:
        case proto::VOICE_INTENT_UNDOCK: {
            if (!dock_ || !request.has_dock()) {
                detail = "dock payload / stub unavailable";
                break;
            }
            accepted = dock_->HandleCommand(
                request.dock(),
                [stream_callback, request](
                    const proto::DockCommandResponse& dock_response) {
                    proto::VoiceCommandResponse voice;
                    voice.set_intent(request.intent());
                    voice.set_status(dock_response.ack().final()
                                         ? (dock_response.ack().success()
                                                ? proto::VOICE_STATUS_SUCCEEDED
                                                : proto::VOICE_STATUS_FAILED)
                                         : proto::VOICE_STATUS_RUNNING);
                    *voice.mutable_ack() = dock_response.ack();
                    voice.set_detail(dock_response.ack().message());
                    stream_callback(voice);
                });
            break;
        }
        case proto::VOICE_INTENT_EXPLORE: {
            if (!exploration_ || !request.has_explore()) {
                detail = "explore payload / stub unavailable";
                break;
            }
            accepted = exploration_->HandleCommand(
                request.explore(),
                [stream_callback, request](
                    const proto::ExplorationCommandResponse& exploration_response) {
                    proto::VoiceCommandResponse voice;
                    voice.set_intent(request.intent());
                    voice.set_status(exploration_response.ack().final()
                                         ? (exploration_response.ack().success()
                                                ? proto::VOICE_STATUS_SUCCEEDED
                                                : proto::VOICE_STATUS_FAILED)
                                         : proto::VOICE_STATUS_RUNNING);
                    *voice.mutable_ack() = exploration_response.ack();
                    voice.set_detail(exploration_response.ack().message());
                    stream_callback(voice);
                });
            break;
        }
        case proto::VOICE_INTENT_STOP:
        case proto::VOICE_INTENT_CANCEL_ALL:
            CancelActiveSession();
            stream_callback(MakeResponse(request, proto::VOICE_STATUS_SUCCEEDED,
                                         true, true, "cancelled all tasks"));
            return true;
        default:
            detail = "unsupported voice intent";
            break;
    }

    if (!accepted) {
        stream_callback(MakeResponse(request, proto::VOICE_STATUS_FAILED, false,
                                     true,
                                     detail.empty() ? "voice dispatch failed"
                                                    : detail));
        return false;
    }
    return true;
}

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
