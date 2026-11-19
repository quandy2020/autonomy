/* Copyright 2026 The Openbot Authors. */
/** @file exploration_context.hpp
 * @brief Native exploration identities, finite evidence cuts and integration callbacks.
 */
#pragma once
#include <functional>
#include <optional>
#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include "autonomy/perception/data/observation_buffer.hpp"
#include "autonomy/task/exploration/integration/goal_pose_adapter.hpp"
#include "autonomy/task/exploration/coverage/completion_evaluator.hpp"
namespace autonomy::task::exploration {
/** @brief Owner lifecycle; reaching a pose does not certify coverage or saved maps. */
enum class ExplorationState {kIdle,kInitializing,kStartupObserving,kExploring,kObserving,kReplanning,kRecovering,
    kWaitingForSensor,kPaused,kVerifyingCompletion,kReturning,kSaving,kFinishing,kFinished};
/** @brief Selects one owner driver without creating competing motion submitters. */
enum class ExplorationDriver { kAutomatic, kBehaviorTree };
/** @brief Independently tracked asynchronous exploration phases. */
enum class ExplorationPhase { kStartupObservation, kSelect, kAdapt, kObserve, kReturn, kSave };
/** @brief Phase outcome; retryable and invalidated phases have not completed their intent. */
enum class PhaseStatus { kPending, kSucceeded, kRetryable, kInvalidated, kFailed, kCanceled };
/** @brief Priority commands processed before normal mapping and observation events. */
enum class SessionCommand {kPause,kResume,kCancel};
/** @brief Startup policy; replacement coverage cannot silently satisfy raw-input retention. */
enum class ObservationRecoveryPolicy {
    kReobserveScope,  ///< New measured coverage and full-scope fusion may establish a replacement baseline.
    kRequireOriginalReplay  ///< Original missing inputs must be independently verified as replayed.
};
/** @brief Request identity plus immutable evidence requirements and deadline. */
struct RequestVersion {
    std::uint64_t session_id = 0;  ///< Process-wide session identity.
    std::uint64_t request_id = 0;  ///< Monotonic request identity in this owner.
    std::uint64_t goal_version = 0;  ///< Observation intent version at submission.
    std::uint64_t map_epoch = 0;  ///< Coordinate-alignment epoch.
    std::uint64_t geometry_revision = 0;  ///< Geometry used by this request.
    std::uint64_t sensor_epoch = 0;  ///< Sensing capability generation.
    std::uint64_t coverage_revision = 0;  ///< Diagnostic measured-evidence revision.
    double valid_until = 0;  ///< Absolute request deadline, in owner clock seconds.
    std::uint64_t requirements_revision = 0;  ///< Work-domain version, excluding quality gains.
    std::uint64_t input_event_sequence = 0;  ///< Finite ordinary event admission prefix.
    perception::ObservationCut observation_cut{};  ///< Finite source prefix for this request.
    perception::ObservationCut observation_baseline{};  ///< Excluded prefix; does not certify missing original data.
    std::uint64_t observation_recovery_generation = 0;  ///< Verified or requested replacement-baseline identity.
    std::optional<perception::ObservationLossSnapshot> recovery_loss;  ///< Present only for a scope recovery refresh.
    std::optional<Bounds2d> recovery_region;  ///< Explicit full task scope to be independently verified by mapper.
    /** @brief Matches request identity; evidence compatibility is checked separately. */
    bool SameRequest(const RequestVersion& b) const{return session_id==b.session_id && request_id==b.request_id &&
        goal_version==b.goal_version && map_epoch==b.map_epoch && sensor_epoch==b.sensor_epoch;}
};
/** @brief Versioned phase handle; only its matching owner request may update it. */
struct PhaseTicket {
    RequestVersion version;  ///< Request identity, evidence requirements and deadline.
    ExplorationPhase phase = ExplorationPhase::kSelect;  ///< Operation associated with this ticket.
    /** @brief Tests phase and request identity without claiming evidence compatibility.
     * @param other Ticket to compare.
     * @return True only for the same phase and versioned request identity.
     */
    bool SameRequest(const PhaseTicket& other) const {
        return phase == other.phase && version.SameRequest(other.version);
    }
};
/** @brief Published state of a specific phase, including its explanatory reason. */
struct PhaseResult {
    PhaseTicket ticket;  ///< Identity whose operation produced this result.
    PhaseStatus status = PhaseStatus::kInvalidated;  ///< Pending or terminal phase state.
    std::string reason;  ///< Human-readable failure, invalidation or completion detail.
};
/** @brief Navigation result; the owner additionally verifies actual arrival and views. */
struct NavigationReply {
    RequestVersion version;  ///< Navigation identity that produced this reply.
    bool success = false;  ///< Execution service reported success; not a coverage receipt.
};
/** @brief Explicit result of the bounded, stationary startup observation. */
struct StartupObservationReply {
    RequestVersion version;  ///< Request identity whose native action produced this result.
    bool success = false;  ///< The complete yaw sweep was measured without translating.
    std::string reason;  ///< Actual validation failure or completion explanation.
};
/** @brief Receipt from the actual mapper; queue draining cannot create this receipt. */
struct MappingReply {
    RequestVersion version;  ///< Identity of the requested refresh or save.
    bool success = false;  ///< The actual mapping operation succeeded.
    bool fusion_confirmed = false;  ///< Real fusion through fused_cut relative to version.observation_baseline.
    perception::ObservationCut fused_cut{};  ///< Explicitly fused source prefix.
    bool recovery_scope_confirmed = false;  ///< Mapper independently verified the entire requested recovery scope.
    bool original_inputs_replayed = false;  ///< Verified original-data replay for a retention-required task.
};
/** @brief Native integration callbacks; real service semantics must be verified independently.
 * Refresh acceptance does not prove fusion. Cancellation callbacks identify
 * requests; physical stopping additionally needs the execution integration.
 */
struct ExplorationServices {
    std::function<bool(const Pose2d&,const RequestVersion&)> submit_navigation;  ///< Attempts versioned pose dispatch.
    std::function<std::optional<NavigationReply>()> poll_navigation;  ///< Polls an available execution result.
    std::function<void(const RequestVersion&)> cancel_navigation;  ///< Requests cancellation of the specified navigation.
    std::function<bool(const RequestVersion&)> request_map_refresh;  ///< Submits finite-cut fusion or scope recovery.
    std::function<std::optional<MappingReply>()> poll_map_refresh;  ///< Polls the actual mapper's versioned receipt.
    std::function<bool(const std::string&,const RequestVersion&)> save_map;  ///< Submits a named versioned save operation.
    std::function<std::optional<MappingReply>()> poll_map_save;  ///< Polls an actual persistence result.
    std::function<void(const RequestVersion&)> cancel_mapping;  ///< Cancels exactly the supplied refresh/save identity.
    std::function<void(const MapReadView&)> update_route_map;  ///< Worker callback updating the owned Vista consumer.
    std::function<bool(const AdaptedObservationGoal&,const Pose2d&,const MapReadView&)> navigation_still_valid;  ///< Revalidates current intent against changed geometry.
    std::function<std::optional<Pose2d>(const Pose2d&,std::uint64_t,std::uint64_t)> reproject_pose;  ///< Transforms a pose between known alignment epochs.
    std::function<bool(const RequestVersion&,double,double)> request_startup_observation;  ///< Starts a zero-translation yaw sweep with a finite allowance.
    std::function<std::optional<StartupObservationReply>()> poll_startup_observation;  ///< Polls the native rotation action's validated result.
    std::function<void(const RequestVersion&)> cancel_startup_observation;  ///< Cancels exactly the matching native rotation action.
};
/** @brief Deployment-confirmed, bounded startup sweep policy.
 * The sweep is permitted only at the initial pose with zero commanded translation.
 * It creates neither map cells nor coverage evidence; observations still enter
 * the ordinary measured mapper and coverage evaluators. Disabled by default.
 */
struct StartupObservationOptions {
    bool enabled = false;  ///< Require one native in-place observation sweep before selection.
    double target_yaw = 3.14159265358979323846;  ///< Nonzero sweep angle; SpinAction accepts at most pi.
    double time_allowance = 8;  ///< Native action deadline and Session request deadline basis, seconds.
    double deadline_margin = 2;  ///< Extra Session time before canceling an unresponsive native action.
    double maximum_position_drift = .03;  ///< Maximum measured translation from the startup pose, metres.
    double completion_tolerance = .03;  ///< Maximum missing yaw at successful completion, radians.
    double maximum_pose_age = 1;  ///< Maximum measured feedback age for the stationary startup sweep, seconds.
    double support_radius = .45;  ///< Self-occluded central disc closed only during initial departure, metres.
    double support_ring_radius = .9;  ///< Measured free ring used to derive floor and ceiling, metres.
    double support_timeout = 30;  ///< Maximum lifetime of the start-only support hint, seconds.
    std::size_t minimum_support_cells = 8;  ///< Minimum measured ring cells required before support closure.
};
/** @brief Fixed startup policy, bounded owner budgets and platform requirements.
 * Sensor dropout, candidate exhaustion and recovery do not weaken these
 * requirements. Timing values use the monotonic owner clock in seconds.
 */
struct ExplorationSessionOptions {
    ExplorationDriver driver = ExplorationDriver::kAutomatic;  ///< Automatic owner or native BT phase driver.
    perception::SensorMode mode=perception::SensorMode::kRgbdOnly;  ///< Startup source policy, independent of runtime health.
    std::vector<perception::SensorMask> validated_controller_modes;  ///< Exact independently validated execution masks.
    bool require_return=true;  ///< Successful task outcome requires return to the starting pose.
    bool require_save=true;  ///< Successful task outcome requires actual final persistence.
    bool save_on_cancel=false;  ///< Explicit policy to save partial output following user cancellation.
    std::string map_name="exploration";  ///< Persistence name supplied to the mapper.
    double time_budget=600;  ///< Active-time budget excluding pause intervals, in seconds.
    double distance_budget=1000;  ///< Total travel budget in meters.
    double return_margin=10;  ///< Nonnegative extra return-time reserve in seconds.
    double navigation_timeout=60;  ///< Deadline duration for one navigation request, in seconds.
    double observation_timeout=5;  ///< Maximum observation wait at a reached pose, in seconds.
    double minimum_observation_time=.5;  ///< Required measured view dwell in seconds.
    double position_tolerance=.2;  ///< Arrival and adaptation origin tolerance in meters.
    double yaw_tolerance=.1;  ///< Required view orientation tolerance in radians.
    double map_max_age=1;  ///< Maximum consumer map publication age in seconds.
    double pose_max_age=.5;  ///< Maximum localization pose age in seconds.
    unsigned maximum_recoveries=3;  ///< Retry cap, applied separately to ordinary and compatible loss recovery.
    perception::SensorHealthOptions health;  ///< Finite quality, freshness and hysteresis thresholds.
    unsigned completion_rounds=3;  ///< Required stable completion rounds with valid observation evidence.
    ObservationRecoveryPolicy observation_recovery_policy = ObservationRecoveryPolicy::kReobserveScope;  ///< Fixed loss recovery/retention requirement.
    double maximum_sensor_wait=10;  ///< Bounded sensing/localization outage wait in seconds.
    double planning_timeout=10;  ///< Deadline duration for a worker planning job, in seconds.
    double map_operation_timeout=10;  ///< Deadline duration for refresh, recovery or save, in seconds.
    double nominal_speed=.4;  ///< Translational speed used only for budget estimates, in meters per second.
    double blocked_retry_delay=2;  ///< Minimum compatible retry backoff in seconds.
    double nominal_angular_speed=.5;  ///< Angular speed used only for budget estimates, in radians per second.
    double frontier_observation_height=.5;  ///< Frontier requirement height above the map ground plane, in meters.
    std::optional<Bounds2d> allowed_region;  ///< Finite positive-area task scope required for full-scope recovery.
    StartupObservationOptions startup_observation;  ///< Optional explicit in-place bootstrap observation.
    std::size_t maximum_events_per_tick = 32;  ///< Maximum ordinary events examined per owner tick.
    std::size_t maximum_observations_per_tick = 8;  ///< Maximum admitted observations examined per owner tick.
};
/** @brief Immutable published owner snapshot; predictions never populate measured coverage. */
struct ExplorationSnapshot {
    std::shared_ptr<const ::automsgs::msgs::map_msgs::OccupancyGrid> measured_coverage_map;  ///< Actual target-ledger percentages; never an obstacle map or whole-room coverage claim.
    std::uint64_t measured_coverage_revision = 0;  ///< Actual ledger revision represented by measured_coverage_map.
    std::uint64_t measured_coverage_map_epoch = 0;  ///< Coordinate identity of the projected actual requirements.
    std::uint64_t measured_coverage_geometry_revision = 0;  ///< Actual map lattice revision associated with the projection.
    ExplorationState state=ExplorationState::kIdle;  ///< Current owner lifecycle state.
    ExplorationResult result;  ///< Independent exploration, return and persistence outcomes.
    std::optional<AdaptedObservationGoal> goal;  ///< Prepared or executing intent with preserved observation requirements.
    std::optional<ObservationGoal> selection;  ///< Selected intent awaiting adaptation or execution.
    std::optional<PhaseTicket> active_phase;  ///< Currently owned BT phase identity.
    perception::SensorReadiness sensors;  ///< Current usable sources and capability generation.
    bool motion_ready = false;  ///< Local readiness predicate; not a physical motion permit or stopping receipt.
    std::uint64_t session_id=0;  ///< Process-wide owner identity.
    std::size_t pending_units=0;  ///< Actual ledger requirements still awaiting measured evidence.
    std::size_t live_frontiers=0;  ///< Current unknown-boundary requirements detected by the consumer.
    double traveled_distance=0;  ///< Accumulated same-epoch pose distance in meters.
    bool completion_confirmed=false;  ///< Finite-cut completion has passed the configured consumer checks.
    bool startup_observation_required=false;  ///< Deployment policy requires a bounded in-place sweep.
    bool startup_observation_confirmed=false;  ///< The native sweep was measured and validated.
    std::string startup_observation_reason;  ///< Current startup admission or completion explanation.
    bool finishing=false;  ///< Owner has entered return/save or partial-result finalization.
    perception::ObservationLossSnapshot observation_losses;  ///< Historical first losses and currently unresolved sources.
    perception::ObservationCut accepted_observations{};  ///< Inputs admitted by the Session buffer; not coverage or fusion.
    perception::ObservationCut processed_observations{};  ///< Examined or discarded inputs, including expired packets.
    perception::ObservationCut valid_observations{};  ///< Actual valid observations evaluated against coverage requirements.
    perception::ObservationCut observation_baseline{};  ///< Verified replacement baseline, never synthesized fusion.
    std::uint64_t observation_recovery_generation = 0;  ///< Last owner-verified recovery identity.
    std::uint64_t last_observed_goal_id=0;  ///< Last intent whose required measured views were confirmed.
    std::uint64_t last_observed_goal_version=0;  ///< Version of that confirmed intent.
    std::size_t observed_view_count=0;  ///< Measured completed views of the current or last intent.
    std::optional<RequestVersion> active_request;  ///< Current navigation, worker, save or recovery identity.
    std::string reason;  ///< Current state, wait or failure explanation.
};
}  // namespace autonomy::task::exploration
