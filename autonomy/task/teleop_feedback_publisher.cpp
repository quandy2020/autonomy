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

#include "autonomy/task/teleop_feedback_publisher.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/task/teleop_goal_channel.hpp"

namespace autonomy {
namespace task {

namespace tp = ::autonomy::task::proto;

TeleopFeedbackPublisher::~TeleopFeedbackPublisher() { Stop(); }

bool TeleopFeedbackPublisher::Start(
    const std::shared_ptr<autolink::Node>& node,
    const TeleopTask::SharedPtr& teleop,
    const std::chrono::milliseconds period) {
    Stop();
    if (!node || !teleop) {
        return false;
    }

    writer_ = node->CreateWriter<tp::TeleopFeedback>(kTeleopFeedbackChannel);
    if (!writer_) {
        AERROR << "TeleopFeedbackPublisher: failed to create writer on "
               << kTeleopFeedbackChannel;
        return false;
    }

    node_ = node;
    teleop_ = teleop;
    period_ = period.count() > 0 ? period : std::chrono::milliseconds(100);
    last_published_ = tp::TELEOP_STATUS_UNKNOWN;
    running_.store(true);
    poll_thread_ = std::thread([this]() { PollLoop(); });

    AINFO << "TeleopFeedbackPublisher on " << kTeleopFeedbackChannel
          << " period_ms=" << period_.count();
    return true;
}

void TeleopFeedbackPublisher::Stop() {
    running_.store(false);
    if (poll_thread_.joinable()) {
        poll_thread_.join();
    }
    writer_.reset();
    node_.reset();
    teleop_.reset();
    last_published_ = tp::TELEOP_STATUS_UNKNOWN;
}

void TeleopFeedbackPublisher::Publish(const tp::TeleopFeedback& feedback) {
    if (!writer_) {
        return;
    }
    writer_->Write(feedback);
    last_published_ = feedback.status();
}

void TeleopFeedbackPublisher::PollLoop() {
    while (running_.load()) {
        const auto teleop = teleop_.lock();
        if (teleop && writer_) {
            tp::TeleopFeedback feedback;
            if (teleop->GetFeedback(&feedback)) {
                const auto status = feedback.status();
                const bool terminal =
                    status == tp::TELEOP_STATUS_TIMEOUT ||
                    status == tp::TELEOP_STATUS_REJECTED;
                const bool active = teleop->IsActive();
                if (status != last_published_ &&
                    (active || terminal || status == tp::TELEOP_STATUS_IDLE)) {
                    Publish(feedback);
                }
            }
        }
        std::this_thread::sleep_for(period_);
    }
}

}  // namespace task
}  // namespace autonomy
