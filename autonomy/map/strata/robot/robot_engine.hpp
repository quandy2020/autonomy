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
#include "autonomy/map/strata/navigation/pathfinder.hpp"
#include "autonomy/map/strata/robot/core/robot_profile.hpp"
#include "autonomy/map/strata/robot/core/waypoint_types.hpp"
#include "autonomy/map/strata/robot/infra/display_manager.hpp"
#include "autonomy/map/strata/robot/robot_environment.hpp"
#include "autonomy/map/strata/robot/tasks/wait_condition.hpp"
#include "autonomy/map/strata/robot/infra/event_bus.hpp"
#include "autonomy/map/strata/robot/tasks/task.hpp"
#include "autonomy/map/strata/robot/visual/robot_visual.hpp"
#include "autonomy/map/strata/render/scene_state.hpp"
#include "autonomy/map/strata/types.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace robot {

struct RobotState {
    std::string id;
    RobotPhase phase{RobotPhase::kIdle};
    LngLat position;
    double headingDeg{0.};
    float battery{100.f};
    tasks::Task::SharedPtr currentTask;
};

struct RobotInitialState {
    RobotPhase phase{RobotPhase::kIdle};
    LngLat position{};
    double headingDeg{0.};
    float battery{100.f};
};

struct RobotTickSnapshot {
    LngLat position;
    double headingDeg{0.};
    RobotPhase phase{RobotPhase::kIdle};
    float battery{100.f};
    TaskStatus taskStatus{TaskStatus::kIdle};
};

RobotMarkerPatch RobotStateToMarkerPatch(const RobotState& state);

class RobotController
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(RobotController)

    explicit RobotController(std::string robot_id,
                             core::RobotProfile profile = {});

    void SetProfile(const core::RobotProfile& profile);
    const core::RobotProfile& profile() const { return profile_; }

    void SetEventBus(infra::EventBus::SharedPtr event_bus);
    void SetResourceManager(infra::ResourceManager::SharedPtr resource_manager);

    void MoveTo(const LngLat& target);
    void MoveAlongPath(const std::vector<LngLat>& waypoints, double tolerance = 0.5);
    void WaitFor(double durationMs);
    void WaitFor(tasks::WaitConditionBase::SharedPtr condition, double timeout_ms = 30000.);
    void SetWaitContextEnricher(std::function<void(tasks::WaitContext&)> enricher);
    void Charge(double min_charge = 100., double charge_rate = -1.);
    void Dock(const std::string& station_id = {}, double duration_ms = 3000.);
    void PerformAction(std::function<bool()> action);
    void Execute(tasks::Task::SharedPtr task);
    void Tick(double deltaTimeMs);
    void CancelCurrentTask();
    void EmergencyStop();
    void Resume();
    void SetPosition(const LngLat& position);
    void SetHeading(double headingDeg);
    void SetPhase(RobotPhase phase);

    const RobotState& state() const { return state_; }
    RobotState& mutableState() { return state_; }
    TaskStatus GetTaskStatus() const;

private:
    RobotState state_;
    core::RobotProfile profile_;
    infra::EventBus::SharedPtr eventBus_;
    infra::ResourceManager::SharedPtr resourceManager_;
    std::function<void(tasks::WaitContext&)> waitContextEnricher_;
};

struct RobotEngineConfig {
    double maxDeltaTimeMs = 100.;
    bool autoSyncDisplay = true;
};

class RobotEngine
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(RobotEngine)

    RobotEngine();
    explicit RobotEngine(RobotEngineConfig config);

    void Start();
    void Stop();
    void Pause();
    void Resume();
    bool IsRunning() const { return running_; }
    bool IsPaused() const { return paused_; }

    void Tick(double deltaTimeMs);
    void AddRobot(RobotController::SharedPtr robot);
    RobotController::SharedPtr AddRobot(const std::string& robot_id,
                                        const core::RobotProfile& profile,
                                        const RobotInitialState& initial = {});
    void RemoveRobot(const std::string& robotId);
    RobotController::SharedPtr GetRobot(const std::string& robotId);
    std::vector<RobotController::SharedPtr> GetRobots() const;
    std::vector<std::string> GetRobotIds() const;

    std::unordered_map<std::string, RobotTickSnapshot> CollectTickSnapshot() const;
    void SyncDisplays();

    void SetScene(render::SceneState::SharedPtr scene);
    void CreateFovForRobot(const std::string& robot_id,
                           const RobotFovOptions& options = {});
    void UpdateFov(const std::string& robot_id, const LngLat& position,
                   double heading_deg);
    void RemoveFov(const std::string& robot_id);

    void AssignMission(const std::string& robotId, tasks::Task::SharedPtr mission);
    void CancelMission(const std::string& robotId);
    TaskStatus GetMissionStatus(const std::string& robotId) const;

    void SetPathfinder(navigation::Pathfinder::SharedPtr pathfinder);
    navigation::Pathfinder::SharedPtr GetPathfinder() const { return pathfinder_; }
    void SetPathfinderObstacles(const std::vector<ObstaclePolygon>& obstacles);
    bool PlanPath(const LngLat& from, const LngLat& to, std::vector<LngLat>& waypoints) const;
    bool MoveRobotTo(const std::string& robotId, const LngLat& target, double tolerance = 0.5);
    void WaitRobotFor(const std::string& robotId, tasks::WaitConditionBase::SharedPtr condition,
                      double timeout_ms = 30000.);

    const RobotEnvironment& environment() const { return environment_; }
    RobotEnvironment& mutableEnvironment() { return environment_; }
    void SetSensor(const std::string& type, double value);
    void SetSensorString(const std::string& type, const std::string& value);
    void SetZoneOccupancy(const std::string& zone_id, const std::string& state);
    void SetElevator(const std::string& id, const ElevatorState& state);
    void SetTrafficLight(const std::string& id, const std::string& color);
    void RegisterResource(infra::ResourceType resource_type, const std::string& resource_id,
                          bool shared = false);

    using ConditionFactory = std::function<tasks::WaitConditionBase::SharedPtr()>;
    using ActionHandler = std::function<bool(RobotState&)>;
    void RegisterConditionType(const std::string& type, ConditionFactory factory);
    tasks::WaitConditionBase::SharedPtr CreateCondition(const std::string& type) const;
    void RegisterActionType(const std::string& type, ActionHandler handler);
    ActionHandler GetActionHandler(const std::string& type) const;
    void PerformTypedAction(const std::string& robotId, const std::string& type);
    void RegisterWaypointType(core::WaypointType type, std::function<void(core::Waypoint&)> handler);

    void Emit(const std::string& signal, const std::string& payload = {});
    void Announce(const std::string& channel, const std::string& message,
                  const std::string& meta_json = {});
    void Signal(const std::string& signal, const std::string& payload = {});
    void DockRobot(const std::string& robotId, const std::string& station_id = {},
                   double duration_ms = 3000.);
    infra::EventBus::SharedPtr GetEventBus() { return eventBus_; }
    infra::ResourceManager::SharedPtr GetResourceManager() { return resourceManager_; }
    infra::DisplayManager::SharedPtr GetDisplayManager() { return displayManager_; }
    robot::core::WaypointTypeRegistry::SharedPtr GetWaypointRegistry() {
        return waypointRegistry_;
    }

private:
    RobotEngineConfig config_;
    bool running_{false};
    bool paused_{false};
    std::unordered_map<std::string, RobotController::SharedPtr> robots_;
    infra::EventBus::SharedPtr eventBus_;
    infra::ResourceManager::SharedPtr resourceManager_;
    infra::DisplayManager::SharedPtr displayManager_;
    robot::core::WaypointTypeRegistry::SharedPtr waypointRegistry_;
    render::SceneState::SharedPtr scene_;
    std::unordered_map<std::string, visual::RobotFovController::SharedPtr> fovControllers_;
    navigation::Pathfinder::SharedPtr pathfinder_;
    std::unordered_map<std::string, ConditionFactory> conditionFactories_;
    std::unordered_map<std::string, ActionHandler> actionHandlers_;
    RobotEnvironment environment_;

    tasks::WaitContext BuildWaitContext(const RobotController::SharedPtr& robot) const;
    void EnrichWaitContext(tasks::WaitContext& context) const;
};

}  // namespace robot
}  // namespace strata
}  // namespace map
}  // namespace autonomy
