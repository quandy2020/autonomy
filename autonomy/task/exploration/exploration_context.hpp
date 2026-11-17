/* Copyright 2026 The Openbot Authors. */
#pragma once
#include <functional>
#include <optional>
#include "autonomy/perception/data/observation_buffer.hpp"
#include "autonomy/task/exploration/integration/goal_pose_adapter.hpp"
#include "autonomy/task/exploration/coverage/completion_evaluator.hpp"
namespace autonomy::task::exploration {
enum class ExplorationState {kIdle,kInitializing,kExploring,kObserving,kReplanning,kRecovering,
    kWaitingForSensor,kPaused,kVerifyingCompletion,kReturning,kSaving,kFinishing,kFinished};
enum class ExplorationDriver { kAutomatic, kBehaviorTree };
enum class ExplorationPhase { kSelect, kAdapt, kObserve, kReturn, kSave };
enum class PhaseStatus { kPending, kSucceeded, kRetryable, kInvalidated, kFailed, kCanceled };
enum class SessionCommand {kPause,kResume,kCancel};
struct RequestVersion {
    std::uint64_t session_id=0,request_id=0,goal_version=0,map_epoch=0;
    std::uint64_t geometry_revision=0,sensor_epoch=0,coverage_revision=0;
    double valid_until=0;
    bool SameRequest(const RequestVersion& b) const{return session_id==b.session_id && request_id==b.request_id &&
        goal_version==b.goal_version && map_epoch==b.map_epoch && sensor_epoch==b.sensor_epoch;}
};
struct PhaseTicket {
    RequestVersion version;
    ExplorationPhase phase = ExplorationPhase::kSelect;
    bool SameRequest(const PhaseTicket& other) const {
        return phase == other.phase && version.SameRequest(other.version);
    }
};
struct PhaseResult {
    PhaseTicket ticket;
    PhaseStatus status = PhaseStatus::kInvalidated;
    std::string reason;
};
struct NavigationReply {RequestVersion version;bool success=false;};
struct MappingReply {
    RequestVersion version;
    bool success=false;
    bool fusion_confirmed=false;
    perception::ObservationCut fused_cut{};
};
struct ExplorationServices {
    std::function<bool(const Pose2d&,const RequestVersion&)> submit_navigation;
    std::function<std::optional<NavigationReply>()> poll_navigation;
    std::function<void(const RequestVersion&)> cancel_navigation;
    std::function<bool(const RequestVersion&)> request_map_refresh;
    std::function<std::optional<MappingReply>()> poll_map_refresh;
    std::function<bool(const std::string&,const RequestVersion&)> save_map;
    std::function<std::optional<MappingReply>()> poll_map_save;
    std::function<void(const RequestVersion&)> cancel_mapping;
    std::function<void(const MapReadView&)> update_route_map;
    std::function<bool(const AdaptedObservationGoal&,const Pose2d&,const MapReadView&)> navigation_still_valid;
    std::function<std::optional<Pose2d>(const Pose2d&,std::uint64_t,std::uint64_t)> reproject_pose;
};
struct ExplorationSessionOptions {
    ExplorationDriver driver = ExplorationDriver::kAutomatic;
    perception::SensorMode mode=perception::SensorMode::kRgbdOnly;
    std::vector<perception::SensorMask> validated_controller_modes;
    bool require_return=true,require_save=true,save_on_cancel=false;
    std::string map_name="exploration";
    double time_budget=600,distance_budget=1000,return_margin=10;
    double navigation_timeout=60,observation_timeout=5,minimum_observation_time=.5;
    double position_tolerance=.2,yaw_tolerance=.1,map_max_age=1,pose_max_age=.5;
    unsigned maximum_recoveries=3;
    perception::SensorHealthOptions health;
    unsigned completion_rounds=3;
    double maximum_sensor_wait=10,planning_timeout=10,map_operation_timeout=10;
    double nominal_speed=.4,blocked_retry_delay=2;
    double nominal_angular_speed=.5;
    double frontier_observation_height=.5;
    std::optional<Bounds2d> allowed_region;
    std::size_t maximum_events_per_tick = 32, maximum_observations_per_tick = 8;
};
struct ExplorationSnapshot {
    ExplorationState state=ExplorationState::kIdle;
    ExplorationResult result;
    std::optional<AdaptedObservationGoal> goal;
    std::optional<ObservationGoal> selection;
    std::optional<PhaseTicket> active_phase;
    perception::SensorReadiness sensors;
    bool motion_ready = false;
    std::uint64_t session_id=0;
    std::size_t pending_units=0;
    std::size_t live_frontiers=0;
    double traveled_distance=0;
    bool completion_confirmed=false;
    bool finishing=false;
    std::uint64_t last_observed_goal_id=0,last_observed_goal_version=0;
    std::size_t observed_view_count=0;
    std::optional<RequestVersion> active_request;
    std::string reason;
};
}  // namespace autonomy::task::exploration
