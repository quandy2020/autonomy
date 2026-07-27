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
#include <stdexcept>

#include "autonomy/common/logging.hpp"
#include "autonomy/map/strata/constants.hpp"
#include "autonomy/map/strata/robot/core/robot_phase.hpp"
#include "autonomy/map/strata/robot/core/waypoint_types.hpp"
#include "autonomy/map/strata/robot/robot_engine.hpp"
#include "autonomy/map/strata/robot/tasks/composites.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace robot {

RobotMarkerPatch RobotStateToMarkerPatch(const RobotState& state) {
    RobotMarkerPatch patch;
    patch.lngLat = state.position;
    patch.rotationDeg = state.headingDeg;
    patch.battery = state.battery;
    patch.robotStatus = core::PhaseToRobotStatus(state.phase);
    return patch;
}

RobotController::RobotController(std::string robot_id, core::RobotProfile profile)
    : profile_(std::move(profile)) {
    state_.id = std::move(robot_id);
    state_.battery = static_cast<float>(profile_.battery.capacity);
}

void RobotController::SetProfile(const core::RobotProfile& profile) { profile_ = profile; }

void RobotController::SetEventBus(infra::EventBus::SharedPtr event_bus) {
    eventBus_ = std::move(event_bus);
}

void RobotController::SetResourceManager(infra::ResourceManager::SharedPtr resource_manager) {
    resourceManager_ = std::move(resource_manager);
}

void RobotController::SetWaitContextEnricher(
    std::function<void(tasks::WaitContext&)> enricher) {
    waitContextEnricher_ = std::move(enricher);
}

void RobotController::MoveTo(const LngLat& target) {
    tasks::MoveTaskConfig config;
    config.kinematics = profile_.kinematics;
    config.battery = profile_.battery;
    Execute(tasks::MoveTask::make_shared(target, &state_, config));
}

void RobotController::MoveAlongPath(const std::vector<LngLat>& waypoints, double tolerance) {
    tasks::MoveTaskConfig config;
    config.kinematics = profile_.kinematics;
    config.battery = profile_.battery;
    config.tolerance = tolerance;
    auto task = tasks::MoveTask::AlongPath(waypoints, &state_, config);
    if (task) {
        Execute(std::move(task));
    }
}

void RobotController::WaitFor(double durationMs) {
    Execute(tasks::WaitTask::make_shared(durationMs));
    SetPhase(RobotPhase::kWaiting);
}

void RobotController::WaitFor(tasks::WaitConditionBase::SharedPtr condition,
                              double timeout_ms) {
    if (!condition) {
        return;
    }
    tasks::WaitContext context;
    context.current_position = state_.position;
    context.resource_manager = resourceManager_.get();

    tasks::WaitConditionTask::ContextUpdater updater = [this](tasks::WaitContext& ctx) {
        ctx.current_position = state_.position;
        if (eventBus_) {
            ctx.signals = eventBus_->ActiveSignals();
        }
        if (waitContextEnricher_) {
            waitContextEnricher_(ctx);
        }
    };

    Execute(tasks::WaitConditionTask::make_shared(std::move(condition), std::move(context),
                                                  std::move(updater), timeout_ms));
    SetPhase(RobotPhase::kWaiting);
}

void RobotController::Charge(double min_charge, double charge_rate) {
    tasks::ChargeTaskConfig config;
    config.minCharge = min_charge;
    config.chargeRate =
        charge_rate >= 0. ? charge_rate : profile_.battery.chargeRate;
    Execute(tasks::ChargeTask::make_shared(&state_, config));
    SetPhase(RobotPhase::kCharging);
}

void RobotController::Dock(const std::string& station_id, double duration_ms) {
    Execute(tasks::DockTask::make_shared(&state_, station_id, duration_ms));
    SetPhase(RobotPhase::kDocking);
}

void RobotController::PerformAction(std::function<bool()> action) {
    Execute(tasks::ActionTask::make_shared(std::move(action)));
}

void RobotController::Execute(tasks::Task::SharedPtr task) {
    CancelCurrentTask();
    state_.currentTask = std::move(task);
}

void RobotController::Tick(double deltaTimeMs) {
    if (!state_.currentTask) {
        return;
    }
    const auto status = state_.currentTask->Tick(deltaTimeMs);
    if (status == TaskStatus::kSuccess) {
        state_.currentTask.reset();
        if (state_.phase != RobotPhase::kArrived) {
            state_.phase = RobotPhase::kIdle;
        }
    } else if (status == TaskStatus::kFailure) {
        SetPhase(RobotPhase::kError);
    }
}

TaskStatus RobotController::GetTaskStatus() const {
    return state_.currentTask ? state_.currentTask->status() : TaskStatus::kIdle;
}

void RobotController::CancelCurrentTask() {
    if (state_.currentTask) {
        state_.currentTask->Cancel();
        state_.currentTask.reset();
    }
}

void RobotController::EmergencyStop() { SetPhase(RobotPhase::kManual); }

void RobotController::Resume() { SetPhase(RobotPhase::kIdle); }

void RobotController::SetPosition(const LngLat& position) { state_.position = position; }

void RobotController::SetHeading(double headingDeg) { state_.headingDeg = headingDeg; }

void RobotController::SetPhase(RobotPhase phase) {
    if (core::CanTransition(state_.phase, phase) || state_.phase == phase) {
        state_.phase = phase;
    } else {
        AWARN << "Invalid robot phase transition from " << ToString(state_.phase) << " to "
              << ToString(phase);
    }
}

RobotEngine::RobotEngine() : RobotEngine(RobotEngineConfig{}) {}

RobotEngine::RobotEngine(RobotEngineConfig config)
    : config_(config),
      eventBus_(infra::EventBus::make_shared()),
      resourceManager_(infra::ResourceManager::make_shared()),
      displayManager_(infra::DisplayManager::make_shared()),
      waypointRegistry_(robot::core::WaypointTypeRegistry::make_shared()) {}

void RobotEngine::Start() {
    running_ = true;
    paused_ = false;
}

void RobotEngine::Stop() {
    running_ = false;
    paused_ = false;
}

void RobotEngine::Pause() { paused_ = true; }

void RobotEngine::Resume() {
    if (!running_) {
        return;
    }
    paused_ = false;
}

void RobotEngine::Tick(double deltaTimeMs) {
    if (!running_ || paused_) {
        return;
    }
    const double dt = std::min(deltaTimeMs, config_.maxDeltaTimeMs);
    for (auto& [_, robot] : robots_) {
        robot->Tick(dt);
    }
    if (config_.autoSyncDisplay) {
        SyncDisplays();
    }
}

void RobotEngine::AddRobot(RobotController::SharedPtr robot) {
    robot->SetEventBus(eventBus_);
    robot->SetResourceManager(resourceManager_);
    robot->SetWaitContextEnricher([this](tasks::WaitContext& ctx) { EnrichWaitContext(ctx); });
    robots_[robot->state().id] = std::move(robot);
}

RobotController::SharedPtr RobotEngine::AddRobot(const std::string& robot_id,
                                                 const core::RobotProfile& profile,
                                                 const RobotInitialState& initial) {
    if (robots_.count(robot_id)) {
        throw std::invalid_argument("Robot \"" + robot_id + "\" already exists");
    }
    auto controller = RobotController::make_shared(robot_id, profile);
    controller->SetPhase(initial.phase);
    controller->SetPosition(initial.position);
    controller->SetHeading(initial.headingDeg);
    controller->mutableState().battery = initial.battery;
    AddRobot(controller);
    return controller;
}

void RobotEngine::RemoveRobot(const std::string& robotId) {
    const auto it = robots_.find(robotId);
    if (it != robots_.end()) {
        it->second->CancelCurrentTask();
        robots_.erase(it);
    }
    RemoveFov(robotId);
    displayManager_->RemoveRobot(robotId);
}

RobotController::SharedPtr RobotEngine::GetRobot(const std::string& robotId) {
    const auto it = robots_.find(robotId);
    return it == robots_.end() ? nullptr : it->second;
}

std::vector<RobotController::SharedPtr> RobotEngine::GetRobots() const {
    std::vector<RobotController::SharedPtr> result;
    result.reserve(robots_.size());
    for (const auto& [_, robot] : robots_) {
        result.push_back(robot);
    }
    return result;
}

std::vector<std::string> RobotEngine::GetRobotIds() const {
    std::vector<std::string> ids;
    ids.reserve(robots_.size());
    for (const auto& [id, _] : robots_) {
        ids.push_back(id);
    }
    return ids;
}

std::unordered_map<std::string, RobotTickSnapshot> RobotEngine::CollectTickSnapshot() const {
    std::unordered_map<std::string, RobotTickSnapshot> snapshot;
    for (const auto& [robot_id, robot] : robots_) {
        const auto& state = robot->state();
        snapshot[robot_id] = RobotTickSnapshot{state.position, state.headingDeg, state.phase,
                                               state.battery, robot->GetTaskStatus()};
    }
    return snapshot;
}

void RobotEngine::SyncDisplays() {
    for (const auto& [robot_id, robot] : robots_) {
        displayManager_->UpdateRobot(robot_id, RobotStateToMarkerPatch(robot->state()));
        UpdateFov(robot_id, robot->state().position, robot->state().headingDeg);
    }
}

void RobotEngine::SetScene(render::SceneState::SharedPtr scene) {
    scene_ = std::move(scene);
}

void RobotEngine::CreateFovForRobot(const std::string& robot_id,
                                    const RobotFovOptions& options) {
    if (!scene_ || fovControllers_.count(robot_id)) {
        return;
    }
    RobotFovOptions fov_options = options;
    fov_options.id = robot_id;
    auto controller = visual::RobotFovController::make_shared(scene_, fov_options);
    fovControllers_[robot_id] = controller;
    if (auto robot = GetRobot(robot_id)) {
        controller->Update(robot->state().position, robot->state().headingDeg);
    }
}

void RobotEngine::UpdateFov(const std::string& robot_id, const LngLat& position,
                            double heading_deg) {
    const auto it = fovControllers_.find(robot_id);
    if (it == fovControllers_.end()) {
        return;
    }
    it->second->Update(position, heading_deg);
}

void RobotEngine::RemoveFov(const std::string& robot_id) {
    const auto it = fovControllers_.find(robot_id);
    if (it == fovControllers_.end()) {
        return;
    }
    it->second->Remove();
    fovControllers_.erase(it);
}

void RobotEngine::AssignMission(const std::string& robotId, tasks::Task::SharedPtr mission) {
    if (auto robot = GetRobot(robotId)) {
        robot->Execute(std::move(mission));
    }
}

void RobotEngine::CancelMission(const std::string& robotId) {
    if (auto robot = GetRobot(robotId)) {
        robot->CancelCurrentTask();
    }
}

TaskStatus RobotEngine::GetMissionStatus(const std::string& robotId) const {
    const auto it = robots_.find(robotId);
    return it == robots_.end() ? TaskStatus::kIdle : it->second->GetTaskStatus();
}

void RobotEngine::Emit(const std::string& signal, const std::string& payload) {
    eventBus_->Emit(signal, payload);
}

void RobotEngine::Announce(const std::string& channel, const std::string& message,
                           const std::string& meta_json) {
    Emit(channel, meta_json.empty() ? message : message + "|" + meta_json);
}

void RobotEngine::Signal(const std::string& signal, const std::string& payload) {
    Emit(signal, payload);
}

void RobotEngine::DockRobot(const std::string& robotId, const std::string& station_id,
                            double duration_ms) {
    if (auto robot = GetRobot(robotId)) {
        robot->Dock(station_id, duration_ms);
    }
}

void RobotEngine::SetPathfinder(navigation::Pathfinder::SharedPtr pathfinder) {
    pathfinder_ = std::move(pathfinder);
}

void RobotEngine::SetPathfinderObstacles(const std::vector<ObstaclePolygon>& obstacles) {
    if (pathfinder_) {
        pathfinder_->SetObstacles(obstacles);
    }
}

bool RobotEngine::PlanPath(const LngLat& from, const LngLat& to,
                           std::vector<LngLat>& waypoints) const {
    waypoints.clear();
    if (!pathfinder_) {
        return false;
    }
    const double width = pathfinder_->widthMeters();
    const double height = pathfinder_->heightMeters();
    if (width <= 0. || height <= 0.) {
        return false;
    }

    const auto start_frac = std::make_pair(from.x / width, from.y / height);
    const auto end_frac = std::make_pair(to.x / width, to.y / height);
    const auto path = pathfinder_->FindPath(start_frac, end_frac);
    if (!path || path->empty()) {
        return false;
    }

    waypoints.reserve(path->size());
    for (const auto& [x_frac, y_frac] : *path) {
        LngLat waypoint;
        waypoint.x = x_frac * width;
        waypoint.y = y_frac * height;
        waypoint.z = to.z;
        waypoints.push_back(waypoint);
    }
    if (waypoints.back().x != to.x || waypoints.back().y != to.y) {
        waypoints.push_back(to);
    }
    return waypoints.size() >= 2;
}

bool RobotEngine::MoveRobotTo(const std::string& robotId, const LngLat& target, double tolerance) {
    auto robot = GetRobot(robotId);
    if (!robot) {
        return false;
    }

    std::vector<LngLat> waypoints;
    if (PlanPath(robot->state().position, target, waypoints)) {
        robot->MoveAlongPath(waypoints, tolerance);
        return true;
    }

    robot->MoveTo(target);
    return true;
}

void RobotEngine::WaitRobotFor(const std::string& robotId,
                               tasks::WaitConditionBase::SharedPtr condition,
                               double timeout_ms) {
    auto robot = GetRobot(robotId);
    if (!robot || !condition) {
        return;
    }
    robot->WaitFor(std::move(condition), timeout_ms);
}

tasks::WaitContext RobotEngine::BuildWaitContext(
    const RobotController::SharedPtr& robot) const {
    tasks::WaitContext context;
    if (robot) {
        context.current_position = robot->state().position;
    }
    context.resource_manager = resourceManager_.get();
    EnrichWaitContext(context);
    return context;
}

void RobotEngine::EnrichWaitContext(tasks::WaitContext& context) const {
    context.sensors = environment_.sensors;
    context.sensor_strings = environment_.sensor_strings;
    context.zones = environment_.zones;
    context.elevators = environment_.elevators;
    context.traffic_lights = environment_.traffic_lights;
    context.robot_positions.clear();
    for (const auto& [id, other] : robots_) {
        context.robot_positions[id] = other->state().position;
    }
    if (eventBus_) {
        context.signals = eventBus_->ActiveSignals();
    }
}

void RobotEngine::SetSensor(const std::string& type, double value) {
    environment_.sensors[type] = value;
}

void RobotEngine::SetSensorString(const std::string& type, const std::string& value) {
    environment_.sensor_strings[type] = value;
}

void RobotEngine::SetZoneOccupancy(const std::string& zone_id, const std::string& state) {
    environment_.zones[zone_id] = state;
}

void RobotEngine::SetElevator(const std::string& id, const ElevatorState& state) {
    environment_.elevators[id] = state;
}

void RobotEngine::SetTrafficLight(const std::string& id, const std::string& color) {
    environment_.traffic_lights[id] = color;
}

void RobotEngine::RegisterResource(infra::ResourceType resource_type,
                                   const std::string& resource_id, bool shared) {
    resourceManager_->RegisterResource(resource_type, resource_id, shared);
}

void RobotEngine::RegisterConditionType(const std::string& type, ConditionFactory factory) {
    conditionFactories_[type] = std::move(factory);
}

tasks::WaitConditionBase::SharedPtr RobotEngine::CreateCondition(const std::string& type) const {
    const auto it = conditionFactories_.find(type);
    return it == conditionFactories_.end() ? nullptr : it->second();
}

void RobotEngine::RegisterActionType(const std::string& type, ActionHandler handler) {
    actionHandlers_[type] = std::move(handler);
}

RobotEngine::ActionHandler RobotEngine::GetActionHandler(const std::string& type) const {
    const auto it = actionHandlers_.find(type);
    return it == actionHandlers_.end() ? ActionHandler{} : it->second;
}

void RobotEngine::PerformTypedAction(const std::string& robotId, const std::string& type) {
    auto handler = GetActionHandler(type);
    if (!handler) {
        return;
    }
    if (auto robot = GetRobot(robotId)) {
        robot->PerformAction([&handler, state = &robot->mutableState()]() { return handler(*state); });
    }
}

void RobotEngine::RegisterWaypointType(core::WaypointType type,
                                       std::function<void(core::Waypoint&)> handler) {
    waypointRegistry_->Register(type, std::move(handler));
}

}  // namespace robot
}  // namespace strata
}  // namespace map
}  // namespace autonomy
