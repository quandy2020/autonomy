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

#include "autonomy/map/strata/robot/tasks/wait_condition.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace robot {
namespace tasks {

namespace {

bool CompareNumeric(double current, double target, SensorCompareOp op) {
    switch (op) {
        case SensorCompareOp::kEq:
            return current == target;
        case SensorCompareOp::kNeq:
            return current != target;
        case SensorCompareOp::kGt:
            return current > target;
        case SensorCompareOp::kGte:
            return current >= target;
        case SensorCompareOp::kLt:
            return current < target;
        case SensorCompareOp::kLte:
            return current <= target;
    }
    return false;
}

class DurationCondition : public WaitConditionBase
{
public:
    explicit DurationCondition(double duration_ms) : duration_ms_(duration_ms) {}

    WaitEvaluation Evaluate(const WaitContext& context) override {
        if (!started_) {
            start_elapsed_ms_ = 0.;
            started_ = true;
        }
        const double elapsed = context.elapsed_ms - start_elapsed_ms_;
        const double remaining = duration_ms_ - elapsed;
        met_ = remaining <= 0.;
        return {met_, std::max(0., remaining)};
    }

    void Reset() override {
        WaitConditionBase::Reset();
        started_ = false;
    }

private:
    double duration_ms_;
    double start_elapsed_ms_{0.};
    bool started_{false};
};

class SignalCondition : public WaitConditionBase
{
public:
    SignalCondition(std::string signal_name, double timeout_ms)
        : signal_name_(std::move(signal_name)), timeout_ms_(timeout_ms) {}

    WaitEvaluation Evaluate(const WaitContext& context) override {
        if (timeout_ms_ > 0. && context.elapsed_ms >= timeout_ms_) {
            met_ = false;
            return {false, 0.};
        }
        const auto it = context.signals.find(signal_name_);
        met_ = it != context.signals.end();
        return {met_, met_ ? 0. : 1.};
    }

private:
    std::string signal_name_;
    double timeout_ms_;
};

class DistanceCondition : public WaitConditionBase
{
public:
    DistanceCondition(std::string robot_id, double min_distance)
        : robot_id_(std::move(robot_id)), min_distance_(min_distance) {}

    WaitEvaluation Evaluate(const WaitContext& context) override {
        const auto it = context.robot_positions.find(robot_id_);
        if (it == context.robot_positions.end()) {
            return {false, min_distance_};
        }
        const double dx = context.current_position.x - it->second.x;
        const double dy = context.current_position.y - it->second.y;
        const double distance = std::hypot(dx, dy);
        met_ = distance >= min_distance_;
        return {met_, std::max(0., min_distance_ - distance)};
    }

private:
    std::string robot_id_;
    double min_distance_;
};

class ResourceCondition : public WaitConditionBase
{
public:
    ResourceCondition(infra::ResourceType resource_type, std::string resource_id,
                      bool shared)
        : resource_type_(resource_type),
          resource_id_(std::move(resource_id)),
          shared_(shared) {}

    WaitEvaluation Evaluate(const WaitContext& context) override {
        if (context.resource_manager == nullptr) {
            return {false, 1.};
        }
        const auto status =
            context.resource_manager->GetStatus(resource_type_, resource_id_);
        met_ = status == infra::ResourceStatus::kFree ||
               (shared_ && status == infra::ResourceStatus::kShared);
        return {met_, met_ ? 0. : 1.};
    }

private:
    infra::ResourceType resource_type_;
    std::string resource_id_;
    bool shared_;
};

class AllCondition : public WaitConditionBase
{
public:
    explicit AllCondition(std::vector<WaitConditionBase::SharedPtr> conditions)
        : conditions_(std::move(conditions)) {}

    WaitEvaluation Evaluate(const WaitContext& context) override {
        for (const auto& condition : conditions_) {
            if (!condition->Evaluate(context).met) {
                met_ = false;
                return {false, 1.};
            }
        }
        met_ = true;
        return {true, 0.};
    }

    void Reset() override {
        WaitConditionBase::Reset();
        for (auto& condition : conditions_) {
            condition->Reset();
        }
    }

private:
    std::vector<WaitConditionBase::SharedPtr> conditions_;
};

class RaceCondition : public WaitConditionBase
{
public:
    explicit RaceCondition(std::vector<WaitConditionBase::SharedPtr> conditions)
        : conditions_(std::move(conditions)) {}

    WaitEvaluation Evaluate(const WaitContext& context) override {
        for (const auto& condition : conditions_) {
            if (condition->Evaluate(context).met) {
                met_ = true;
                return {true, 0.};
            }
        }
        met_ = false;
        return {false, 1.};
    }

    void Reset() override {
        WaitConditionBase::Reset();
        for (auto& condition : conditions_) {
            condition->Reset();
        }
    }

private:
    std::vector<WaitConditionBase::SharedPtr> conditions_;
};

class NotCondition : public WaitConditionBase
{
public:
    explicit NotCondition(WaitConditionBase::SharedPtr condition)
        : condition_(std::move(condition)) {}

    WaitEvaluation Evaluate(const WaitContext& context) override {
        met_ = !condition_->Evaluate(context).met;
        return {met_, met_ ? 0. : 1.};
    }

    void Reset() override {
        WaitConditionBase::Reset();
        condition_->Reset();
    }

private:
    WaitConditionBase::SharedPtr condition_;
};

class SensorCondition : public WaitConditionBase
{
public:
    SensorCondition(std::string type, double target_value, SensorCompareOp op)
        : type_(std::move(type)), target_value_(target_value), op_(op) {}

    WaitEvaluation Evaluate(const WaitContext& context) override {
        const auto it = context.sensors.find(type_);
        if (it == context.sensors.end()) {
            return {false, 1.};
        }
        met_ = CompareNumeric(it->second, target_value_, op_);
        return {met_, met_ ? 0. : 1.};
    }

private:
    std::string type_;
    double target_value_;
    SensorCompareOp op_;
};

class SensorStringCondition : public WaitConditionBase
{
public:
    SensorStringCondition(std::string type, std::string target_value, SensorCompareOp op)
        : type_(std::move(type)), target_value_(std::move(target_value)), op_(op) {}

    WaitEvaluation Evaluate(const WaitContext& context) override {
        const auto it = context.sensor_strings.find(type_);
        if (it == context.sensor_strings.end()) {
            return {false, 1.};
        }
        met_ = (op_ == SensorCompareOp::kEq) ? it->second == target_value_
                                             : it->second != target_value_;
        return {met_, met_ ? 0. : 1.};
    }

private:
    std::string type_;
    std::string target_value_;
    SensorCompareOp op_;
};

class OccupancyCondition : public WaitConditionBase
{
public:
    OccupancyCondition(std::string zone_id, std::string state)
        : zone_id_(std::move(zone_id)), state_(std::move(state)) {}

    WaitEvaluation Evaluate(const WaitContext& context) override {
        const auto it = context.zones.find(zone_id_);
        met_ = it != context.zones.end() && it->second == state_;
        return {met_, met_ ? 0. : 1.};
    }

private:
    std::string zone_id_;
    std::string state_;
};

class ElevatorCondition : public WaitConditionBase
{
public:
    ElevatorCondition(std::string floor, std::string elevator_id, std::string direction,
                      bool require_doors_open)
        : floor_(std::move(floor)),
          elevator_id_(std::move(elevator_id)),
          direction_(std::move(direction)),
          require_doors_open_(require_doors_open) {}

    WaitEvaluation Evaluate(const WaitContext& context) override {
        if (context.elevators.empty()) {
            return {false, 1.};
        }
        const auto matches = [&](const ElevatorState& state) {
            if (state.floor != floor_) {
                return false;
            }
            if (!direction_.empty() && state.direction != direction_) {
                return false;
            }
            if (require_doors_open_ && !state.doors_open) {
                return false;
            }
            return true;
        };

        if (!elevator_id_.empty()) {
            const auto it = context.elevators.find(elevator_id_);
            met_ = it != context.elevators.end() && matches(it->second);
            return {met_, met_ ? 0. : 1.};
        }

        for (const auto& [_, state] : context.elevators) {
            if (matches(state)) {
                met_ = true;
                return {true, 0.};
            }
        }
        met_ = false;
        return {false, 1.};
    }

private:
    std::string floor_;
    std::string elevator_id_;
    std::string direction_;
    bool require_doors_open_;
};

class TrafficLightCondition : public WaitConditionBase
{
public:
    TrafficLightCondition(std::string color, std::string light_id)
        : color_(std::move(color)), light_id_(std::move(light_id)) {}

    WaitEvaluation Evaluate(const WaitContext& context) override {
        if (context.traffic_lights.empty()) {
            return {false, 1.};
        }
        if (!light_id_.empty()) {
            const auto it = context.traffic_lights.find(light_id_);
            met_ = it != context.traffic_lights.end() && it->second == color_;
            return {met_, met_ ? 0. : 1.};
        }
        for (const auto& [_, value] : context.traffic_lights) {
            if (value == color_) {
                met_ = true;
                return {true, 0.};
            }
        }
        met_ = false;
        return {false, 1.};
    }

private:
    std::string color_;
    std::string light_id_;
};

}  // namespace

WaitConditionBase::SharedPtr WaitCondition::Duration(double duration_ms) {
    return std::make_shared<DurationCondition>(duration_ms);
}

WaitConditionBase::SharedPtr WaitCondition::Signal(const std::string& signal_name,
                                                   double timeout_ms) {
    return std::make_shared<SignalCondition>(signal_name, timeout_ms);
}

WaitConditionBase::SharedPtr WaitCondition::Distance(const std::string& robot_id,
                                                     double min_distance) {
    return std::make_shared<DistanceCondition>(robot_id, min_distance);
}

WaitConditionBase::SharedPtr WaitCondition::Resource(infra::ResourceType resource_type,
                                                     const std::string& resource_id,
                                                     bool shared) {
    return std::make_shared<ResourceCondition>(resource_type, resource_id, shared);
}

WaitConditionBase::SharedPtr WaitCondition::All(
    const std::vector<WaitConditionBase::SharedPtr>& conditions) {
    return std::make_shared<AllCondition>(conditions);
}

WaitConditionBase::SharedPtr WaitCondition::Race(
    const std::vector<WaitConditionBase::SharedPtr>& conditions) {
    return std::make_shared<RaceCondition>(conditions);
}

WaitConditionBase::SharedPtr WaitCondition::Not(WaitConditionBase::SharedPtr condition) {
    return std::make_shared<NotCondition>(std::move(condition));
}

WaitConditionBase::SharedPtr WaitCondition::Sensor(const std::string& type,
                                                     double target_value,
                                                     SensorCompareOp op) {
    return std::make_shared<SensorCondition>(type, target_value, op);
}

WaitConditionBase::SharedPtr WaitCondition::SensorString(const std::string& type,
                                                         const std::string& target_value,
                                                         SensorCompareOp op) {
    return std::make_shared<SensorStringCondition>(type, target_value, op);
}

WaitConditionBase::SharedPtr WaitCondition::Occupancy(const std::string& zone_id,
                                                      const std::string& state) {
    return std::make_shared<OccupancyCondition>(zone_id, state);
}

WaitConditionBase::SharedPtr WaitCondition::Elevator(const std::string& floor,
                                                     const std::string& elevator_id,
                                                     const std::string& direction,
                                                     bool require_doors_open) {
    return std::make_shared<ElevatorCondition>(floor, elevator_id, direction,
                                             require_doors_open);
}

WaitConditionBase::SharedPtr WaitCondition::TrafficLight(const std::string& color,
                                                         const std::string& light_id) {
    return std::make_shared<TrafficLightCondition>(color, light_id);
}

WaitConditionTask::WaitConditionTask(WaitConditionBase::SharedPtr condition, WaitContext context,
                                     ContextUpdater context_updater, double timeout_ms)
    : condition_(std::move(condition)),
      context_(std::move(context)),
      contextUpdater_(std::move(context_updater)),
      timeout_ms_(timeout_ms) {}

TaskStatus WaitConditionTask::Tick(double deltaTimeMs) {
    if (status_ == TaskStatus::kCancelled) {
        return status_;
    }
    status_ = TaskStatus::kRunning;
    context_.elapsed_ms += deltaTimeMs;
    if (timeout_ms_ > 0. && context_.elapsed_ms >= timeout_ms_) {
        status_ = TaskStatus::kFailure;
        return status_;
    }
    if (contextUpdater_) {
        contextUpdater_(context_);
    }
    const auto evaluation = condition_->Evaluate(context_);
    if (evaluation.met) {
        status_ = TaskStatus::kSuccess;
    }
    return status_;
}

void WaitConditionTask::Reset() {
    Task::Reset();
    if (condition_) {
        condition_->Reset();
    }
    context_.elapsed_ms = 0.;
}

}  // namespace tasks
}  // namespace robot
}  // namespace strata
}  // namespace map
}  // namespace autonomy
