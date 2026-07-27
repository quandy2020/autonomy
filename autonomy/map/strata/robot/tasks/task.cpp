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

#include <cmath>

#include "autonomy/map/strata/robot/core/robot_phase.hpp"
#include "autonomy/map/strata/robot/robot_engine.hpp"
#include "autonomy/map/strata/robot/tasks/task.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace robot {
namespace tasks {

namespace {

double NormalizeHeading(double heading_deg) {
    double normalized = std::fmod(heading_deg, 360.0);
    if (normalized < 0.) {
        normalized += 360.0;
    }
    return normalized;
}

double ShortestHeadingDelta(double from_deg, double to_deg) {
    return std::fmod(to_deg - from_deg + 540.0, 360.0) - 180.0;
}

}  // namespace

Sequence::Sequence(std::vector<Task::SharedPtr> children) : children_(std::move(children)) {}

TaskStatus Sequence::Tick(double delta_time_ms) {
    if (status_ == TaskStatus::kCancelled) {
        return status_;
    }
    status_ = TaskStatus::kRunning;
    while (index_ < children_.size()) {
        const auto child_status = children_[index_]->Tick(delta_time_ms);
        if (child_status == TaskStatus::kFailure || child_status == TaskStatus::kCancelled) {
            status_ = child_status;
            return status_;
        }
        if (child_status != TaskStatus::kSuccess) {
            return TaskStatus::kRunning;
        }
        ++index_;
    }
    status_ = TaskStatus::kSuccess;
    return status_;
}

void Sequence::Cancel() {
    Task::Cancel();
    for (auto& child : children_) {
        child->Cancel();
    }
}

void Sequence::Reset() {
    Task::Reset();
    index_ = 0;
    for (auto& child : children_) {
        child->Reset();
    }
}

MoveTask::MoveTask(LngLat target, RobotState* state, MoveTaskConfig config)
    : target_(target), state_(state), config_(std::move(config)) {}

TaskStatus MoveTask::Tick(double delta_time_ms) {
    if (status_ == TaskStatus::kCancelled || state_ == nullptr) {
        return status_;
    }
    if (status_ == TaskStatus::kSuccess || status_ == TaskStatus::kFailure) {
        return status_;
    }

    status_ = TaskStatus::kRunning;
    started_ = true;
    const double delta_seconds = delta_time_ms / 1000.0;

    const double delta_x = target_.x - state_->position.x;
    const double delta_y = target_.y - state_->position.y;
    const double distance = std::hypot(delta_x, delta_y);

    if (distance < config_.tolerance) {
        state_->position = target_;
        status_ = TaskStatus::kSuccess;
        state_->phase = RobotPhase::kArrived;
        if (config_.on_arrive) {
            config_.on_arrive(*state_);
        }
        return status_;
    }

    const double target_heading =
        NormalizeHeading(std::atan2(delta_x, delta_y) * 180.0 / M_PI);
    const double max_rotation = config_.kinematics.rotationSpeed * delta_seconds;
    const double heading_diff = ShortestHeadingDelta(state_->headingDeg, target_heading);
    const double abs_diff = std::abs(heading_diff);

    if (abs_diff > 1.0) {
        state_->phase = RobotPhase::kRotating;
        if (abs_diff <= max_rotation) {
            state_->headingDeg = target_heading;
        } else {
            state_->headingDeg =
                NormalizeHeading(state_->headingDeg + std::copysign(max_rotation, heading_diff));
        }
        state_->battery = std::max(
            0.f, state_->battery - static_cast<float>(config_.battery.drainRotate * delta_time_ms));
        return TaskStatus::kRunning;
    }

    state_->phase = RobotPhase::kMoving;
    state_->headingDeg = target_heading;
    const double step = std::min(config_.kinematics.maxSpeed * delta_seconds, distance);
    const double heading_rad = state_->headingDeg * M_PI / 180.0;
    state_->position.x += std::sin(heading_rad) * step;
    state_->position.y += std::cos(heading_rad) * step;
    state_->battery = std::max(
        0.f, state_->battery - static_cast<float>(config_.battery.drainMove * delta_time_ms));
    return TaskStatus::kRunning;
}

void MoveTask::Cancel() {
    started_ = false;
    Task::Cancel();
}

void MoveTask::Reset() {
    Task::Reset();
    started_ = false;
}

Task::SharedPtr MoveTask::AlongPath(const std::vector<LngLat>& waypoints, RobotState* state,
                                    MoveTaskConfig config) {
    if (waypoints.empty() || state == nullptr) {
        return nullptr;
    }
    if (waypoints.size() == 1) {
        return MoveTask::make_shared(waypoints.front(), state, config);
    }
    std::vector<Task::SharedPtr> moves;
    moves.reserve(waypoints.size());
    for (const auto& waypoint : waypoints) {
        moves.push_back(MoveTask::make_shared(waypoint, state, config));
    }
    return std::make_shared<Sequence>(std::move(moves));
}

WaitTask::WaitTask(double duration_ms) : duration_ms_(duration_ms) {}

TaskStatus WaitTask::Tick(double delta_time_ms) {
    if (status_ == TaskStatus::kCancelled) {
        return status_;
    }
    status_ = TaskStatus::kRunning;
    elapsed_ += delta_time_ms;
    if (elapsed_ >= duration_ms_) {
        status_ = TaskStatus::kSuccess;
    }
    return status_;
}

void WaitTask::Reset() {
    Task::Reset();
    elapsed_ = 0.;
}

ActionTask::ActionTask(std::function<bool()> action) : action_(std::move(action)) {}

TaskStatus ActionTask::Tick(double delta_time_ms) {
    (void)delta_time_ms;
    if (status_ == TaskStatus::kCancelled) {
        return status_;
    }
    status_ = action_ && action_() ? TaskStatus::kSuccess : TaskStatus::kFailure;
    return status_;
}

void ActionTask::Reset() {
    Task::Reset();
}

SensorTask::SensorTask(std::string sensor, RobotState* state,
                       std::function<bool(RobotState&)> action)
    : sensor_(std::move(sensor)), state_(state), action_(std::move(action)) {}

TaskStatus SensorTask::Tick(double delta_time_ms) {
    (void)delta_time_ms;
    if (status_ == TaskStatus::kCancelled || state_ == nullptr) {
        return status_;
    }
    if (status_ == TaskStatus::kSuccess || status_ == TaskStatus::kFailure) {
        return status_;
    }

    status_ = TaskStatus::kRunning;
    if (!launched_) {
        launched_ = true;
        bool ok = false;
        if (action_) {
            try {
                ok = action_(*state_);
            } catch (...) {
                ok = false;
            }
        }
        status_ = ok ? TaskStatus::kSuccess : TaskStatus::kFailure;
    }
    return status_;
}

void SensorTask::Reset() {
    Task::Reset();
    launched_ = false;
}

}  // namespace tasks
}  // namespace robot
}  // namespace strata
}  // namespace map
}  // namespace autonomy
