/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#include <algorithm>

#include "autonomy/map/strata/robot/core/robot_phase.hpp"
#include "autonomy/map/strata/robot/robot_engine.hpp"
#include "autonomy/map/strata/robot/tasks/composites.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace robot {
namespace tasks {

Parallel::Parallel(std::vector<Task::SharedPtr> children, bool failFast)
    : children_(std::move(children)), failFast_(failFast) {}

TaskStatus Parallel::Tick(double deltaTimeMs) {
    if (status_ == TaskStatus::kCancelled) {
        return status_;
    }
    status_ = TaskStatus::kRunning;
    bool allCompleted = true;
    for (auto& child : children_) {
        const auto childStatus = child->status();
        if (childStatus == TaskStatus::kFailure || childStatus == TaskStatus::kCancelled) {
            if (failFast_) {
                Cancel();
                status_ = TaskStatus::kFailure;
                return status_;
            }
            continue;
        }
        if (childStatus != TaskStatus::kSuccess) {
            const auto result = child->Tick(deltaTimeMs);
            if (result == TaskStatus::kFailure || result == TaskStatus::kCancelled) {
                if (failFast_) {
                    Cancel();
                    status_ = TaskStatus::kFailure;
                    return status_;
                }
            }
            if (result != TaskStatus::kSuccess) {
                allCompleted = false;
            }
        }
    }
    if (allCompleted) {
        status_ = TaskStatus::kSuccess;
    }
    return status_;
}

void Parallel::Cancel() {
    Task::Cancel();
    for (auto& child : children_) {
        child->Cancel();
    }
}

void Parallel::Reset() {
    Task::Reset();
    for (auto& child : children_) {
        child->Reset();
    }
}

LoopTask::LoopTask(Task::SharedPtr child, int times) : child_(std::move(child)), times_(times) {}

TaskStatus LoopTask::Tick(double deltaTimeMs) {
    if (status_ == TaskStatus::kCancelled) {
        return status_;
    }
    if (times_ == 0) {
        status_ = TaskStatus::kSuccess;
        return status_;
    }
    status_ = TaskStatus::kRunning;
    const auto result = child_->Tick(deltaTimeMs);
    if (result == TaskStatus::kFailure || result == TaskStatus::kCancelled) {
        status_ = TaskStatus::kFailure;
        return status_;
    }
    if (result == TaskStatus::kSuccess) {
        ++iteration_;
        if (times_ != -1 && iteration_ >= times_) {
            status_ = TaskStatus::kSuccess;
            return status_;
        }
        child_->Reset();
    }
    return TaskStatus::kRunning;
}

void LoopTask::Cancel() {
    Task::Cancel();
    child_->Cancel();
}

void LoopTask::Reset() {
    Task::Reset();
    iteration_ = 0;
    child_->Reset();
}

ConditionalTask::ConditionalTask(std::function<bool()> condition, Task::SharedPtr thenTask,
                                 Task::SharedPtr elseTask)
    : condition_(std::move(condition)),
      thenTask_(std::move(thenTask)),
      elseTask_(std::move(elseTask)) {}

TaskStatus ConditionalTask::Tick(double deltaTimeMs) {
    if (status_ == TaskStatus::kCancelled) {
        return status_;
    }
    if (!started_) {
        activeTask_ = condition_() ? thenTask_ : elseTask_;
        started_ = true;
        if (!activeTask_) {
            status_ = TaskStatus::kSuccess;
            return status_;
        }
    }
    status_ = TaskStatus::kRunning;
    const auto result = activeTask_->Tick(deltaTimeMs);
    if (result == TaskStatus::kSuccess) {
        status_ = TaskStatus::kSuccess;
    } else if (result == TaskStatus::kFailure || result == TaskStatus::kCancelled) {
        status_ = TaskStatus::kFailure;
    }
    return status_;
}

void ConditionalTask::Cancel() {
    Task::Cancel();
    if (activeTask_) {
        activeTask_->Cancel();
    }
}

void ConditionalTask::Reset() {
    Task::Reset();
    started_ = false;
    activeTask_.reset();
    if (thenTask_) {
        thenTask_->Reset();
    }
    if (elseTask_) {
        elseTask_->Reset();
    }
}

RetryTask::RetryTask(Task::SharedPtr child, int maxRetries)
    : child_(std::move(child)), maxRetries_(maxRetries) {}

TaskStatus RetryTask::Tick(double deltaTimeMs) {
    if (status_ == TaskStatus::kCancelled) {
        return status_;
    }
    status_ = TaskStatus::kRunning;
    const auto result = child_->Tick(deltaTimeMs);
    if (result == TaskStatus::kSuccess) {
        status_ = TaskStatus::kSuccess;
        return status_;
    }
    if (result == TaskStatus::kFailure || result == TaskStatus::kCancelled) {
        ++attempts_;
        if (attempts_ > maxRetries_) {
            status_ = TaskStatus::kFailure;
            return status_;
        }
        child_->Reset();
    }
    return TaskStatus::kRunning;
}

void RetryTask::Cancel() {
    Task::Cancel();
    child_->Cancel();
}

void RetryTask::Reset() {
    Task::Reset();
    attempts_ = 0;
    child_->Reset();
}

RaceTask::RaceTask(std::vector<Task::SharedPtr> children) : children_(std::move(children)) {}

TaskStatus RaceTask::Tick(double deltaTimeMs) {
    if (status_ == TaskStatus::kCancelled) {
        return status_;
    }
    status_ = TaskStatus::kRunning;
    for (auto& child : children_) {
        if (child->status() == TaskStatus::kSuccess) {
            for (auto& other : children_) {
                if (other != child) {
                    other->Cancel();
                }
            }
            status_ = TaskStatus::kSuccess;
            return status_;
        }
    }
    for (auto& child : children_) {
        const auto result = child->Tick(deltaTimeMs);
        if (result == TaskStatus::kSuccess) {
            for (auto& other : children_) {
                if (other != child) {
                    other->Cancel();
                }
            }
            status_ = TaskStatus::kSuccess;
            return status_;
        }
    }
    const bool allFailed = std::all_of(children_.begin(), children_.end(), [](const Task::SharedPtr& t) {
        return t->status() == TaskStatus::kFailure || t->status() == TaskStatus::kCancelled;
    });
    if (allFailed) {
        status_ = TaskStatus::kFailure;
    }
    return status_;
}

void RaceTask::Cancel() {
    Task::Cancel();
    for (auto& child : children_) {
        child->Cancel();
    }
}

void RaceTask::Reset() {
    Task::Reset();
    for (auto& child : children_) {
        child->Reset();
    }
}

AnnounceTask::AnnounceTask(std::function<void(const std::string&, const std::string&)> emit,
                           std::string channel, std::string message, std::string meta_json)
    : emit_(std::move(emit)),
      channel_(std::move(channel)),
      message_(std::move(message)),
      meta_json_(std::move(meta_json)) {}

TaskStatus AnnounceTask::Tick(double deltaTimeMs) {
    (void)deltaTimeMs;
    if (status_ == TaskStatus::kCancelled) {
        return status_;
    }
    if (emit_) {
        std::string payload = message_;
        if (!meta_json_.empty()) {
            payload += "|" + meta_json_;
        }
        emit_(channel_, payload);
    }
    status_ = TaskStatus::kSuccess;
    return status_;
}

ChargeTask::ChargeTask(RobotState* state, ChargeTaskConfig config)
    : state_(state), config_(std::move(config)) {}

TaskStatus ChargeTask::Tick(double deltaTimeMs) {
    if (status_ == TaskStatus::kCancelled || state_ == nullptr) {
        return status_;
    }
    status_ = TaskStatus::kRunning;
    state_->phase = RobotPhase::kCharging;

    if (config_.chargeRate <= 0.) {
        status_ = TaskStatus::kSuccess;
        return status_;
    }

    const float capacity = static_cast<float>(config_.minCharge);
    state_->battery = std::min(
        capacity, state_->battery + static_cast<float>(config_.chargeRate * deltaTimeMs));
    if (state_->battery >= capacity - 1e-4f) {
        state_->battery = capacity;
        status_ = TaskStatus::kSuccess;
    }
    return status_;
}

void ChargeTask::Reset() {
    Task::Reset();
}

DockTask::DockTask(RobotState* state, std::string station_id, double duration_ms,
                   std::function<bool()> dock_action)
    : state_(state),
      station_id_(std::move(station_id)),
      duration_ms_(duration_ms),
      dock_action_(std::move(dock_action)) {}

TaskStatus DockTask::Tick(double deltaTimeMs) {
    if (status_ == TaskStatus::kCancelled || state_ == nullptr) {
        return status_;
    }
    status_ = TaskStatus::kRunning;
    state_->phase = RobotPhase::kDocking;

    if (elapsed_ms_ == 0. && dock_action_ && !dock_action_()) {
        status_ = TaskStatus::kFailure;
        return status_;
    }

    elapsed_ms_ += deltaTimeMs;
    if (elapsed_ms_ >= duration_ms_) {
        status_ = TaskStatus::kSuccess;
    }
    return status_;
}

void DockTask::Reset() {
    Task::Reset();
    elapsed_ms_ = 0.;
}

SignalTask::SignalTask(std::function<void(const std::string&, const std::string&)> emit,
                       std::string signal, std::string payload)
    : emit_(std::move(emit)), signal_(std::move(signal)), payload_(std::move(payload)) {}

TaskStatus SignalTask::Tick(double deltaTimeMs) {
    (void)deltaTimeMs;
    if (status_ == TaskStatus::kCancelled) {
        return status_;
    }
    if (emit_) {
        emit_(signal_, payload_);
    }
    status_ = TaskStatus::kSuccess;
    return status_;
}

}  // namespace tasks
}  // namespace robot
}  // namespace strata
}  // namespace map
}  // namespace autonomy
