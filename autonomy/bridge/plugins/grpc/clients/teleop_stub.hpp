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

#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "autolink/node/node.hpp"
#include "autonomy/bridge/proto/external_command_service.pb.h"
#include "autonomy/common/macros.hpp"
#include "autonomy/task/proto/teleop.pb.h"

namespace autonomy {
namespace bridge {
namespace plugins {
namespace grpc {
namespace clients {

/**
 * @brief Publishes bridge teleop commands to the task process via Autolink.
 *
 * Subscribes to task TeleopFeedback for REJECTED / TIMEOUT and other status
 * transitions; immediate command acks remain optimistic for START/VELOCITY.
 */
class TeleopStub
{
public:
    using StreamCallback =
        std::function<void(const proto::TeleopCommandResponse& response)>;

    AUTONOMY_SMART_PTR_DEFINITIONS(TeleopStub)

    explicit TeleopStub(std::shared_ptr<autolink::Node> node);

    bool HandleCommand(const proto::TeleopCommandRequest& request,
                       StreamCallback stream_callback);

    void ResetSession();

private:
    void OnFeedback(
        const std::shared_ptr<::autonomy::task::proto::TeleopFeedback>& feedback);

    proto::TeleopCommandResponse MakeResponse(
        const proto::TeleopCommandRequest& request, proto::TeleopStatus status,
        bool success, bool final, const std::string& message = "") const;

    proto::TeleopCommandResponse MakeResponseFromFeedback(
        const ::autonomy::task::proto::TeleopFeedback& feedback) const;

    bool PublishGoal(const ::autonomy::task::proto::TeleopGoal& goal);

    void PublishStopGoal();

    std::shared_ptr<autolink::Node> node_;
    std::shared_ptr<autolink::Writer<::autonomy::task::proto::TeleopGoal>>
        goal_writer_;
    std::shared_ptr<autolink::Reader<::autonomy::task::proto::TeleopFeedback>>
        feedback_reader_;

    mutable std::mutex mutex_;
    bool session_active_{false};
    proto::TeleopCommandRequest last_request_;
    StreamCallback stream_callback_;
};

}  // namespace clients
}  // namespace grpc
}  // namespace plugins
}  // namespace bridge
}  // namespace autonomy
