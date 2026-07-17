/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autonomy/bridge/plugins/grpc/clients/teleop_stub.hpp"

#include "autonomy/bridge/plugins/grpc/teleop_goal_convert.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/task/teleop_goal_channel.hpp"

namespace autonomy {
namespace bridge {
namespace plugins {
namespace grpc {
namespace clients {

namespace tp = ::autonomy::task::proto;

namespace {

bool IsTerminalTaskStatus(tp::TeleopStatus status) {
    return status == tp::TELEOP_STATUS_TIMEOUT ||
           status == tp::TELEOP_STATUS_REJECTED;
}

proto::TaskStatus TaskStatusForTeleop(proto::TeleopStatus status, bool success) {
    switch (status) {
        case proto::TELEOP_STATUS_ACTIVE:
            return proto::TASK_STATUS_RUNNING;
        case proto::TELEOP_STATUS_IDLE:
            return proto::TASK_STATUS_IDLE;
        case proto::TELEOP_STATUS_TIMEOUT:
        case proto::TELEOP_STATUS_REJECTED:
            return success ? proto::TASK_STATUS_IDLE : proto::TASK_STATUS_FAILED;
        default:
            return proto::TASK_STATUS_UNKNOWN;
    }
}

}  // namespace

TeleopStub::TeleopStub(std::shared_ptr<autolink::Node> node)
    : node_(std::move(node)) {
    if (!node_) {
        return;
    }
    goal_writer_ = node_->CreateWriter<tp::TeleopGoal>(task::kTeleopGoalChannel);
    if (!goal_writer_) {
        AERROR << "TeleopStub: failed to create writer on "
               << task::kTeleopGoalChannel;
    }

    TeleopStub* self = this;
    feedback_reader_ = node_->CreateReader<tp::TeleopFeedback>(
        task::kTeleopFeedbackChannel,
        [self](const std::shared_ptr<tp::TeleopFeedback>& feedback) {
            self->OnFeedback(feedback);
        });
    if (!feedback_reader_) {
        AWARN << "TeleopStub: feedback reader unavailable on "
              << task::kTeleopFeedbackChannel;
    }
}

void TeleopStub::ResetSession() {
    PublishStopGoal();
    std::lock_guard<std::mutex> lock(mutex_);
    session_active_ = false;
    stream_callback_ = nullptr;
    last_request_.Clear();
}

void TeleopStub::PublishStopGoal() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!session_active_ || !goal_writer_) {
        return;
    }
    tp::TeleopGoal stop;
    stop.set_command(tp::TELEOP_CMD_STOP);
    goal_writer_->Write(stop);
}

bool TeleopStub::PublishGoal(const tp::TeleopGoal& goal) {
    if (!goal_writer_) {
        return false;
    }
    return goal_writer_->Write(goal);
}

proto::TeleopCommandResponse TeleopStub::MakeResponse(
    const proto::TeleopCommandRequest& request, const proto::TeleopStatus status,
    const bool success, const bool final,
    const std::string& message) const {
    proto::TeleopCommandResponse response;
    response.set_status(status);

    auto* ack = response.mutable_ack();
    ack->set_success(success);
    ack->set_final(final);
    ack->set_task_type(proto::TASK_TYPE_TELEOP);
    ack->set_task_status(TaskStatusForTeleop(status, success));
    if (request.has_header()) {
        ack->set_cmd_id(request.header().cmd_id());
    }
    if (!message.empty()) {
        ack->set_message(message);
    }
    return response;
}

proto::TeleopCommandResponse TeleopStub::MakeResponseFromFeedback(
    const tp::TeleopFeedback& feedback) const {
    const auto bridge_status =
        static_cast<proto::TeleopStatus>(feedback.status());
    const bool terminal = IsTerminalTaskStatus(feedback.status()) ||
                          feedback.status() == tp::TELEOP_STATUS_IDLE;
    const bool success =
        feedback.status() != tp::TELEOP_STATUS_REJECTED &&
        feedback.status() != tp::TELEOP_STATUS_TIMEOUT;

    proto::TeleopCommandResponse response;
    response.set_status(bridge_status);

    auto* ack = response.mutable_ack();
    ack->set_success(success);
    ack->set_final(terminal);
    ack->set_task_type(proto::TASK_TYPE_TELEOP);
    ack->set_task_status(TaskStatusForTeleop(bridge_status, success));
    if (last_request_.has_header()) {
        ack->set_cmd_id(last_request_.header().cmd_id());
    }
    if (feedback.status() == tp::TELEOP_STATUS_REJECTED) {
        ack->set_message("teleop goal rejected by task");
    } else if (feedback.status() == tp::TELEOP_STATUS_TIMEOUT) {
        ack->set_message("teleop watchdog timeout");
    }
    return response;
}

void TeleopStub::OnFeedback(
    const std::shared_ptr<tp::TeleopFeedback>& feedback) {
    if (!feedback) {
        return;
    }

    StreamCallback callback;
    proto::TeleopCommandResponse response;
    bool clear_session = false;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!stream_callback_) {
            return;
        }

        const auto status = feedback->status();
        if (status == tp::TELEOP_STATUS_ACTIVE) {
            return;
        }
        if (status == tp::TELEOP_STATUS_IDLE && !session_active_) {
            return;
        }

        response = MakeResponseFromFeedback(*feedback);
        callback = stream_callback_;
        if (IsTerminalTaskStatus(status) ||
            (status == tp::TELEOP_STATUS_IDLE && session_active_)) {
            clear_session = true;
            session_active_ = false;
        }
    }

    callback(response);
    if (clear_session) {
        std::lock_guard<std::mutex> lock(mutex_);
        stream_callback_ = nullptr;
    }
}

bool TeleopStub::HandleCommand(const proto::TeleopCommandRequest& request,
                               StreamCallback stream_callback) {
    if (!stream_callback) {
        AERROR << "TeleopStub: stream callback is null.";
        return false;
    }
    if (!goal_writer_) {
        stream_callback(MakeResponse(
            request, proto::TELEOP_STATUS_REJECTED, false, true,
            "teleop goal writer unavailable"));
        return false;
    }

    const auto command = request.command();
    if (command == proto::TELEOP_CMD_UNSPECIFIED) {
        stream_callback(MakeResponse(request, proto::TELEOP_STATUS_REJECTED, false,
                                     true, "unspecified teleop command"));
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        stream_callback_ = stream_callback;
        last_request_ = request;
    }

    if (command == proto::TELEOP_CMD_VELOCITY) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!session_active_) {
            stream_callback(MakeResponse(
                request, proto::TELEOP_STATUS_REJECTED, false, false,
                "teleop session not active; send START first"));
            return false;
        }
    }

    if (command == proto::TELEOP_CMD_START) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (session_active_) {
            stream_callback(MakeResponse(
                request, proto::TELEOP_STATUS_REJECTED, false, false,
                "teleop session already active"));
            return false;
        }
    }

    const tp::TeleopGoal goal = ToTaskTeleopGoal(request);
    if (!PublishGoal(goal)) {
        stream_callback(MakeResponse(
            request, proto::TELEOP_STATUS_REJECTED, false, true,
            "failed to publish teleop goal"));
        return false;
    }

    switch (command) {
    case proto::TELEOP_CMD_START: {
        std::lock_guard<std::mutex> lock(mutex_);
        session_active_ = true;
        stream_callback(MakeResponse(request, proto::TELEOP_STATUS_ACTIVE, true,
                                     false));
        return true;
    }
    case proto::TELEOP_CMD_VELOCITY:
        stream_callback(MakeResponse(request, proto::TELEOP_STATUS_ACTIVE, true,
                                     false));
        return true;
    case proto::TELEOP_CMD_STOP: {
        stream_callback(
            MakeResponse(request, proto::TELEOP_STATUS_IDLE, true, true));
        std::lock_guard<std::mutex> lock(mutex_);
        session_active_ = false;
        stream_callback_ = nullptr;
        return true;
    }
    default:
        stream_callback(MakeResponse(request, proto::TELEOP_STATUS_REJECTED,
                                       false, true, "unsupported command"));
        return false;
    }
}

}  // namespace clients
}  // namespace grpc
}  // namespace plugins
}  // namespace bridge
}  // namespace autonomy
