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

#include "autonomy/task/teleop_goal_ingress.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/task/teleop_goal_channel.hpp"

namespace autonomy {
namespace task {

namespace tp = ::autonomy::task::proto;

TeleopGoalIngress::~TeleopGoalIngress() { Stop(); }

bool TeleopGoalIngress::Start(const std::shared_ptr<autolink::Node>& node,
                              SubmitFn submit,
                              std::function<void(const tp::TeleopGoal&)>
                                  on_rejected) {
    Stop();
    if (!node || !submit) {
        return false;
    }

    node_ = node;
    submit_ = std::move(submit);
    on_rejected_ = std::move(on_rejected);

    reader_ = node_->CreateReader<::autonomy::task::proto::TeleopGoal>(
        kTeleopGoalChannel,
        [this](const std::shared_ptr<::autonomy::task::proto::TeleopGoal>&
                   goal) { OnGoal(goal); });
    if (!reader_) {
        AERROR << "TeleopGoalIngress: failed to create reader on "
               << kTeleopGoalChannel;
        node_.reset();
        submit_ = nullptr;
        on_rejected_ = nullptr;
        return false;
    }

    AINFO << "TeleopGoalIngress listening on " << kTeleopGoalChannel;
    return true;
}

void TeleopGoalIngress::Stop() {
    reader_.reset();
    node_.reset();
    submit_ = nullptr;
    on_rejected_ = nullptr;
}

void TeleopGoalIngress::OnGoal(
    const std::shared_ptr<::autonomy::task::proto::TeleopGoal>& goal) {
    if (!goal || !submit_) {
        return;
    }
    if (!submit_(*goal)) {
        AWARN << "TeleopGoalIngress: submit rejected command=" << goal->command();
        if (on_rejected_) {
            on_rejected_(*goal);
        }
    }
}

}  // namespace task
}  // namespace autonomy
