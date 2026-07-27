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

#include <string>
#include <unordered_map>
#include <vector>

#include "autonomy/map/strata/types.hpp"

namespace autonomy {
namespace map {
namespace strata {

// Layer IDs — aligned with BICMap layerConfig.js
struct LayerIds {
    static constexpr const char* kCanvasMap = "canvas-map-layer";
    static constexpr const char* kRectangle = "rectangle-layer";
    static constexpr const char* kRectangleOutline = "rectangle-outline-layer";
    static constexpr const char* kPolygon = "polygon-layer";
    static constexpr const char* kPolygonOutline = "polygon-outline-layer";
    static constexpr const char* kBatchPoi = "batch-poi-layer";
    static constexpr const char* kRobotMarkers = "robot-markers-layer";
    static constexpr const char* kRectangleDrawFill = "rectangle-draw-fill-layer";
    static constexpr const char* kRectangleDrawLine = "rectangle-draw-line-layer";
    static constexpr const char* kCircleDrawFill = "circle-draw-fill-layer";
    static constexpr const char* kCircleDrawLine = "circle-draw-line-layer";
    static constexpr const char* kPolygonDrawFill = "polygon-draw-fill-layer";
    static constexpr const char* kPolygonDrawLine = "polygon-draw-line-layer";
    static constexpr const char* kPolygonDrawPoints = "polygon-draw-points-layer";
    static constexpr const char* kPolylineDrawFill = "polyline-draw-fill-layer";
    static constexpr const char* kPolylineDrawLine = "polyline-draw-line-layer";
    static constexpr const char* kPolylineDrawPoints = "polyline-draw-points-layer";
    static constexpr const char* kPointCloud = "point-cloud-layer";
    static constexpr const char* kCircle = "circle-layer";
    static constexpr const char* kPolyline = "polyline-layer";
    static constexpr const char* kWideLine = "wideline-layer";
    static constexpr const char* kSemanticZone = "semantic-zone-layer";
    static constexpr const char* kBuilding3D = "building-3d-layer";
    static constexpr const char* kRobotFov = "robot-fov-layer";
    static constexpr const char* kRobot3D = "robot-3d-layer";
};

inline const std::vector<std::string>& LayerOrderArray() {
    static const std::vector<std::string> order = {
        LayerIds::kCanvasMap,
        LayerIds::kRectangle,
        LayerIds::kRectangleOutline,
        LayerIds::kPolygon,
        LayerIds::kPolygonOutline,
        LayerIds::kSemanticZone,
        LayerIds::kBuilding3D,
        LayerIds::kBatchPoi,
        LayerIds::kRobotMarkers,
        LayerIds::kRobotFov,
        LayerIds::kRectangleDrawFill,
        LayerIds::kRectangleDrawLine,
        LayerIds::kCircleDrawFill,
        LayerIds::kCircleDrawLine,
        LayerIds::kPolygonDrawFill,
        LayerIds::kPolygonDrawLine,
        LayerIds::kPolygonDrawPoints,
        LayerIds::kPolylineDrawFill,
        LayerIds::kPolylineDrawLine,
        LayerIds::kPolylineDrawPoints,
        LayerIds::kCircle,
        LayerIds::kPolyline,
        LayerIds::kWideLine,
        LayerIds::kPointCloud,
        LayerIds::kRobot3D,
    };
    return order;
}

enum class ZoneType {
    kForbidden,
    kSpeedLimit,
    kServiceArea,
    kCharging,
    kElevator,
    kWaiting,
    kActivityArea,
};

inline std::string ToString(ZoneType type) {
    switch (type) {
        case ZoneType::kForbidden: return "forbidden";
        case ZoneType::kSpeedLimit: return "speed_limit";
        case ZoneType::kServiceArea: return "service_area";
        case ZoneType::kCharging: return "charging";
        case ZoneType::kElevator: return "elevator";
        case ZoneType::kWaiting: return "waiting";
        case ZoneType::kActivityArea: return "activity_area";
    }
    return "forbidden";
}

struct ZoneStylePreset {
    ColorRgba fillColor;
    float fillOpacity;
    ColorRgba outlineColor;
    float outlineWidth;
    std::vector<float> outlineDash;
    std::string label;
};

inline const std::unordered_map<ZoneType, ZoneStylePreset>& ZoneStylePresets() {
    static const std::unordered_map<ZoneType, ZoneStylePreset> presets = {
        {ZoneType::kForbidden,
         {{0.898f, 0.224f, 0.208f, 1.f}, 0.18f, {0.898f, 0.224f, 0.208f, 1.f}, 2.f, {4, 3}, "禁行区"}},
        {ZoneType::kSpeedLimit,
         {{0.984f, 0.549f, 0.f, 1.f}, 0.15f, {0.984f, 0.549f, 0.f, 1.f}, 2.f, {6, 3}, "限速区"}},
        {ZoneType::kServiceArea,
         {{0.f, 0.537f, 0.482f, 1.f}, 0.12f, {0.f, 0.537f, 0.482f, 1.f}, 1.5f, {}, "服务范围"}},
        {ZoneType::kCharging,
         {{0.263f, 0.627f, 0.278f, 1.f}, 0.2f, {0.263f, 0.627f, 0.278f, 1.f}, 2.f, {}, "充电区"}},
        {ZoneType::kElevator,
         {{0.118f, 0.533f, 0.898f, 1.f}, 0.15f, {0.118f, 0.533f, 0.898f, 1.f}, 2.f, {3, 3}, "电梯区"}},
        {ZoneType::kWaiting,
         {{0.471f, 0.565f, 0.612f, 1.f}, 0.12f, {0.471f, 0.565f, 0.612f, 1.f}, 1.5f, {5, 3}, "等候区"}},
        {ZoneType::kActivityArea,
         {{1.f, 1.f, 1.f, 1.f}, 0.15f, {0.263f, 0.627f, 0.278f, 1.f}, 2.f, {8, 4}, "活动区域"}},
    };
    return presets;
}

enum class IotEventType {
    kElevatorCall,
    kElevatorArrived,
    kElevatorOpen,
    kElevatorClose,
    kDoorOpen,
    kDoorClose,
    kDeliveryArrived,
    kCustom,
    kWarning,
    kError,
};

inline std::string ToString(IotEventType type) {
    switch (type) {
        case IotEventType::kElevatorCall: return "elevator_call";
        case IotEventType::kElevatorArrived: return "elevator_arrived";
        case IotEventType::kElevatorOpen: return "elevator_open";
        case IotEventType::kElevatorClose: return "elevator_close";
        case IotEventType::kDoorOpen: return "door_open";
        case IotEventType::kDoorClose: return "door_close";
        case IotEventType::kDeliveryArrived: return "delivery_arrived";
        case IotEventType::kCustom: return "custom";
        case IotEventType::kWarning: return "warning";
        case IotEventType::kError: return "error";
    }
    return "custom";
}

enum class RobotPhase {
    kIdle,
    kMoving,
    kRotating,
    kArrived,
    kDwelling,
    kWaiting,
    kCharging,
    kDocking,
    kError,
    kRecovering,
    kReturning,
    kManual,
    kPaused,
};

inline std::string ToString(RobotPhase phase) {
    switch (phase) {
        case RobotPhase::kIdle: return "idle";
        case RobotPhase::kMoving: return "moving";
        case RobotPhase::kRotating: return "rotating";
        case RobotPhase::kArrived: return "arrived";
        case RobotPhase::kDwelling: return "dwelling";
        case RobotPhase::kWaiting: return "waiting";
        case RobotPhase::kCharging: return "charging";
        case RobotPhase::kDocking: return "docking";
        case RobotPhase::kError: return "error";
        case RobotPhase::kRecovering: return "recovering";
        case RobotPhase::kReturning: return "returning";
        case RobotPhase::kManual: return "manual";
        case RobotPhase::kPaused: return "paused";
    }
    return "idle";
}

enum class TaskStatus { kIdle, kRunning, kSuccess, kFailure, kCancelled };

inline std::string ToString(RobotStatus status) {
    switch (status) {
        case RobotStatus::kIdle: return "idle";
        case RobotStatus::kRunning: return "running";
        case RobotStatus::kCharging: return "charging";
        case RobotStatus::kError: return "error";
        case RobotStatus::kReturning: return "returning";
    }
    return "idle";
}

struct RobotStatusStyle {
    ColorRgba color;
    std::string label;
};

inline const std::unordered_map<RobotStatus, RobotStatusStyle>& RobotStatusStyles() {
    static const std::unordered_map<RobotStatus, RobotStatusStyle> styles = {
        {RobotStatus::kIdle, {{0.392f, 0.455f, 0.545f, 1.f}, "待机"}},
        {RobotStatus::kRunning, {{0.f, 0.4f, 1.f, 1.f}, "执行中"}},
        {RobotStatus::kCharging, {{0.969f, 0.659f, 0.f, 1.f}, "充电"}},
        {RobotStatus::kError, {{1.f, 0.231f, 0.188f, 1.f}, "故障"}},
        {RobotStatus::kReturning, {{0.024f, 0.714f, 0.831f, 1.f}, "召回中"}},
    };
    return styles;
}

}  // namespace strata
}  // namespace map
}  // namespace autonomy
