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

#pragma once

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/map/strata/robot/infra/event_bus.hpp"
#include "autonomy/map/strata/robot/robot_environment.hpp"
#include "autonomy/map/strata/robot/tasks/task.hpp"
#include "autonomy/map/strata/types.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace robot {
namespace tasks {

struct WaitContext {
    double elapsed_ms{0.};
    std::unordered_map<std::string, std::string> signals;
    std::unordered_map<std::string, LngLat> robot_positions;
    LngLat current_position;
    infra::ResourceManager* resource_manager{nullptr};

    std::unordered_map<std::string, double> sensors;
    std::unordered_map<std::string, std::string> sensor_strings;
    std::unordered_map<std::string, std::string> zones;
    std::unordered_map<std::string, ElevatorState> elevators;
    std::unordered_map<std::string, std::string> traffic_lights;
};

struct WaitEvaluation {
    bool met{false};
    double remaining{0.};
};

enum class SensorCompareOp { kEq, kNeq, kGt, kGte, kLt, kLte };

class WaitConditionBase
{
public:
    using SharedPtr = std::shared_ptr<WaitConditionBase>;
    virtual ~WaitConditionBase() = default;

    virtual WaitEvaluation Evaluate(const WaitContext& context) = 0;
    virtual void Reset() { met_ = false; }
    bool IsMet() const { return met_; }

protected:
    bool met_{false};
};

class WaitCondition
{
public:
    static WaitConditionBase::SharedPtr Duration(double duration_ms);
    static WaitConditionBase::SharedPtr Signal(const std::string& signal_name,
                                               double timeout_ms = 0.);
    static WaitConditionBase::SharedPtr Distance(const std::string& robot_id,
                                                 double min_distance);
    static WaitConditionBase::SharedPtr Resource(infra::ResourceType resource_type,
                                                 const std::string& resource_id,
                                                 bool shared = false);

    static WaitConditionBase::SharedPtr All(
        const std::vector<WaitConditionBase::SharedPtr>& conditions);
    static WaitConditionBase::SharedPtr Race(
        const std::vector<WaitConditionBase::SharedPtr>& conditions);
    static WaitConditionBase::SharedPtr Not(WaitConditionBase::SharedPtr condition);

    static WaitConditionBase::SharedPtr Sensor(const std::string& type,
                                                 double target_value,
                                                 SensorCompareOp op = SensorCompareOp::kEq);
    static WaitConditionBase::SharedPtr SensorString(const std::string& type,
                                                     const std::string& target_value,
                                                     SensorCompareOp op = SensorCompareOp::kEq);
    static WaitConditionBase::SharedPtr Occupancy(const std::string& zone_id,
                                                  const std::string& state = "free");
    static WaitConditionBase::SharedPtr Elevator(
        const std::string& floor, const std::string& elevator_id = {},
        const std::string& direction = {}, bool require_doors_open = true);
    static WaitConditionBase::SharedPtr TrafficLight(const std::string& color,
                                                     const std::string& light_id = {});
};

class WaitConditionTask : public Task
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(WaitConditionTask)

    using ContextUpdater = std::function<void(WaitContext&)>;

    WaitConditionTask(WaitConditionBase::SharedPtr condition, WaitContext context,
                      ContextUpdater context_updater = nullptr, double timeout_ms = 0.);

    TaskStatus Tick(double deltaTimeMs) override;
    void Reset() override;

private:
    WaitConditionBase::SharedPtr condition_;
    WaitContext context_;
    ContextUpdater contextUpdater_;
    double timeout_ms_{0.};
};

}  // namespace tasks
}  // namespace robot
}  // namespace strata
}  // namespace map
}  // namespace autonomy
