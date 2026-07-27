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
#include <vector>

#include "autonomy/map/strata/robot/tasks/task.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace robot {
struct RobotState;
}  // namespace robot
}  // namespace strata
}  // namespace map
}  // namespace autonomy

namespace autonomy {
namespace map {
namespace strata {
namespace robot {
namespace tasks {

class Parallel : public Task
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(Parallel)

    Parallel(std::vector<Task::SharedPtr> children, bool failFast = true);
    TaskStatus Tick(double deltaTimeMs) override;
    void Cancel() override;
    void Reset() override;

private:
    std::vector<Task::SharedPtr> children_;
    bool failFast_{true};
};

class LoopTask : public Task
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(LoopTask)

    LoopTask(Task::SharedPtr child, int times = -1);
    TaskStatus Tick(double deltaTimeMs) override;
    void Cancel() override;
    void Reset() override;
    int iteration() const { return iteration_; }

private:
    Task::SharedPtr child_;
    int times_{-1};
    int iteration_{0};
};

class ConditionalTask : public Task
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(ConditionalTask)

    ConditionalTask(std::function<bool()> condition, Task::SharedPtr thenTask,
                    Task::SharedPtr elseTask = nullptr);
    TaskStatus Tick(double deltaTimeMs) override;
    void Cancel() override;
    void Reset() override;

private:
    std::function<bool()> condition_;
    Task::SharedPtr thenTask_;
    Task::SharedPtr elseTask_;
    Task::SharedPtr activeTask_;
    bool started_{false};
};

class RetryTask : public Task
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(RetryTask)

    RetryTask(Task::SharedPtr child, int maxRetries = 3);
    TaskStatus Tick(double deltaTimeMs) override;
    void Cancel() override;
    void Reset() override;
    int attempts() const { return attempts_; }

private:
    Task::SharedPtr child_;
    int maxRetries_{3};
    int attempts_{0};
};

class RaceTask : public Task
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(RaceTask)

    explicit RaceTask(std::vector<Task::SharedPtr> children);
    TaskStatus Tick(double deltaTimeMs) override;
    void Cancel() override;
    void Reset() override;

private:
    std::vector<Task::SharedPtr> children_;
};

class AnnounceTask : public Task
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(AnnounceTask)

    AnnounceTask(std::function<void(const std::string&, const std::string&)> emit,
                 std::string channel, std::string message, std::string meta_json = {});
    TaskStatus Tick(double deltaTimeMs) override;

private:
    std::function<void(const std::string&, const std::string&)> emit_;
    std::string channel_;
    std::string message_;
    std::string meta_json_;
};

struct ChargeTaskConfig {
    double minCharge{100.};
    double chargeRate{0.02};
    std::string stationId;
};

/** BICMap ChargeTask — 切换 CHARGING 阶段，按帧增加电量至 minCharge。 */
class ChargeTask : public Task
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(ChargeTask)

    ChargeTask(RobotState* state, ChargeTaskConfig config = {});
    TaskStatus Tick(double deltaTimeMs) override;
    void Reset() override;

private:
    RobotState* state_;
    ChargeTaskConfig config_;
};

class DockTask : public Task
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(DockTask)

    DockTask(RobotState* state, std::string station_id = {}, double duration_ms = 3000.,
             std::function<bool()> dock_action = nullptr);
    TaskStatus Tick(double deltaTimeMs) override;
    void Reset() override;

private:
    RobotState* state_;
    std::string station_id_;
    double duration_ms_;
    std::function<bool()> dock_action_;
    double elapsed_ms_{0.};
};

class SignalTask : public Task
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(SignalTask)

    SignalTask(std::function<void(const std::string&, const std::string&)> emit,
               std::string signal, std::string payload = {});
    TaskStatus Tick(double deltaTimeMs) override;

private:
    std::function<void(const std::string&, const std::string&)> emit_;
    std::string signal_;
    std::string payload_;
};

}  // namespace tasks
}  // namespace robot
}  // namespace strata
}  // namespace map
}  // namespace autonomy
