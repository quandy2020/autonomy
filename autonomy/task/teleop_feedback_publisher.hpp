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

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "autolink/node/node.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/task/apps/teleop/teleop.hpp"
#include "autonomy/task/proto/teleop.pb.h"

namespace autonomy {
namespace task {

/** Publishes {@link TeleopFeedback} for bridge gRPC status streaming. */
class TeleopFeedbackPublisher
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(TeleopFeedbackPublisher)

    TeleopFeedbackPublisher() = default;
    ~TeleopFeedbackPublisher();

    bool Start(const std::shared_ptr<autolink::Node>& node,
               const TeleopTask::SharedPtr& teleop,
               std::chrono::milliseconds period);
    void Stop();

    void Publish(const ::autonomy::task::proto::TeleopFeedback& feedback);

private:
    void PollLoop();

    TeleopTask::WeakPtr teleop_;
    std::shared_ptr<autolink::Node> node_;
    std::shared_ptr<autolink::Writer<::autonomy::task::proto::TeleopFeedback>>
        writer_;
    std::thread poll_thread_;
    std::chrono::milliseconds period_{100};
    std::atomic<bool> running_{false};
    ::autonomy::task::proto::TeleopStatus last_published_{
        ::autonomy::task::proto::TELEOP_STATUS_UNKNOWN};
};

}  // namespace task
}  // namespace autonomy
