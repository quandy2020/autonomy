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

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/map/strata/constants.hpp"
#include "autonomy/map/strata/robot/core/robot_profile.hpp"
#include "autonomy/map/strata/types.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace robot {

struct RobotState;

namespace tasks {

class Task
{
public:
    using SharedPtr = std::shared_ptr<Task>;

    virtual ~Task() = default;
    virtual TaskStatus Tick(double delta_time_ms) = 0;
    virtual void Cancel() { status_ = TaskStatus::kCancelled; }
    virtual void Reset() { status_ = TaskStatus::kIdle; }
    TaskStatus status() const { return status_; }

protected:
    TaskStatus status_{TaskStatus::kIdle};
};

class Sequence : public Task
{
public:
    explicit Sequence(std::vector<Task::SharedPtr> children);
    TaskStatus Tick(double delta_time_ms) override;
    void Cancel() override;
    void Reset() override;

private:
    std::vector<Task::SharedPtr> children_;
    size_t index_{0};
};

struct MoveTaskConfig {
    core::RobotKinematics kinematics{};
    core::RobotBatteryProfile battery{};
    double tolerance{0.5};
    std::function<void(RobotState&)> on_arrive;
};

/** BICMap MoveTask — 旋转对齐后按 maxSpeed 移动，消耗电量。 */
class MoveTask : public Task
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(MoveTask)

    MoveTask(LngLat target, RobotState* state, MoveTaskConfig config = {});
    TaskStatus Tick(double delta_time_ms) override;
    void Cancel() override;
    void Reset() override;

    /** 按 Pathfinder 路点顺序移动；空路点等价于直达 target。 */
    static Task::SharedPtr AlongPath(const std::vector<LngLat>& waypoints, RobotState* state,
                                     MoveTaskConfig config = {});

private:
    LngLat target_;
    RobotState* state_;
    MoveTaskConfig config_;
    bool started_{false};
};

class WaitTask : public Task
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(WaitTask)

    explicit WaitTask(double duration_ms);
    TaskStatus Tick(double delta_time_ms) override;
    void Reset() override;

private:
    double duration_ms_;
    double elapsed_{0.};
};

class ActionTask : public Task
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(ActionTask)

    explicit ActionTask(std::function<bool()> action);
    TaskStatus Tick(double delta_time_ms) override;
    void Reset() override;

private:
    std::function<bool()> action_;
};

/** BICMap SensorTask — 执行传感器动作后完成。 */
class SensorTask : public Task
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(SensorTask)

    SensorTask(std::string sensor, RobotState* state, std::function<bool(RobotState&)> action);
    TaskStatus Tick(double delta_time_ms) override;
    void Reset() override;

private:
    std::string sensor_;
    RobotState* state_;
    std::function<bool(RobotState&)> action_;
    bool launched_{false};
};

}  // namespace tasks
}  // namespace robot
}  // namespace strata
}  // namespace map
}  // namespace autonomy
