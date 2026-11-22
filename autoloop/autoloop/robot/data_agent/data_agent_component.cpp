/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

/**
 * @file data_agent_component.cpp
 * @brief Bypass capture loop. Never publishes /cmd_vel.
 */
#include "robot/data_agent/data_agent_component.hpp"
#include "common/base/path_util.hpp"

namespace autoloop {
namespace robot {

DataAgentComponent::DataAgentComponent(std::string record_root, std::string robot_id)
    : robot_id_(std::move(robot_id)),
      ring_(2'000'000'000),
      writer_({record_root, robot_id_}),
      uploader_(common::JoinPath(record_root, "staging")) {
    tel_.SetRobotId(robot_id_);
    triggers_.AddRule(TriggerRule(TriggerKind::kTakeover, Priority::kP0, kSampleTeleop));
    triggers_.AddRule(TriggerRule(TriggerKind::kTaskFail, Priority::kP0, kSampleFeedback));
    triggers_.AddRule(TriggerRule(TriggerKind::kEstop, Priority::kP0, 0));
}

common::Status DataAgentComponent::OnSample(Sample sample) {
    const Sample view = sample;
    ring_.Push(std::move(sample));
    tel_.SetRingSamples(ring_.size());
    for (const auto& ev : triggers_.Observe(view)) {
        auto st = Flush(ev);
        if (!st.ok()) {
            return st;
        }
    }
    return common::Status::OK();
}

common::Status DataAgentComponent::ManualSnapshot(const std::string& reason) {
    return Flush(triggers_.Manual(reason));
}

common::Status DataAgentComponent::PumpUpload() { return uploader_.PumpOnce(); }

common::Status DataAgentComponent::Flush(const TriggerEvent& ev) {
    std::string dir;
    auto st = writer_.Write(ring_.Snapshot(), ev, version_.check(), &dir);
    if (!st.ok()) {
        tel_.SetError(st.message());
        return st;
    }
    tel_.SetSnapshots(writer_.snapshots_written());
    uploader_.Enqueue({dir, ev.priority, 0});
    tel_.SetUploadPending(uploader_.pending());
    return common::Status::OK();
}

}  // namespace robot
}  // namespace autoloop
