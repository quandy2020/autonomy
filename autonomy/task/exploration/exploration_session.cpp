/* Copyright 2026 The Openbot Authors. */
/** @file exploration_session.cpp
 * @brief Exploration owner and worker coordination with finite input prefixes.
 */
#include "autonomy/task/exploration/exploration_session.hpp"

#include <array>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <limits>
#include <map>
#include <mutex>
#include <set>
#include <stdexcept>
#include <thread>
#include <variant>
#include "autonomy/common/logging.hpp"
#include "autonomy/perception/data/observation_buffer.hpp"
#include "autonomy/task/exploration/coverage/coverage_evaluator.hpp"
#include "autonomy/task/exploration/coverage/measured_coverage_map.hpp"
#include "autonomy/task/exploration/coverage/surface_extractor.hpp"
#include "autonomy/task/exploration/frontier/frontier_extractor.hpp"
#include "autonomy/task/exploration/polygon_boundary.hpp"
#include "autonomy/task/exploration/routing/acquisition_keypose_ledger.hpp"

namespace autonomy::task::exploration {
namespace {
/// Process-wide session identities reject replies from reconstructed tasks.
std::atomic<std::uint64_t> next_session_identity{0};
/** @brief Allocate a nonzero identity without wrapping onto a prior session. */
std::uint64_t NewSessionIdentity() {
    auto current = next_session_identity.load();
    for (;;) {
        if (current == std::numeric_limits<std::uint64_t>::max())
            throw std::overflow_error("exploration session identity exhausted");
        if (next_session_identity.compare_exchange_weak(current, current + 1))
            return current + 1;
    }
}

/** @brief Compare exact finite sensor-prefix identities. */
bool SameObservationCut(const perception::ObservationCut& lhs,
                        const perception::ObservationCut& rhs) {
    return lhs.present == rhs.present && lhs.sequence == rhs.sequence;
}

/** @brief Require a Vista route certificate from the exact map snapshot. */
bool RouteEvidenceMatchesMap(const planning::planner::vista::VistaRoute& route,
                             const MapReadView& map) {
    return route.evidence && route.map_epoch == map.map_epoch &&
           route.geometry_revision == map.geometry_revision &&
           route.evidence->map_epoch == map.map_epoch &&
           route.evidence->geometry_revision == map.geometry_revision &&
           route.path.frame_id == map.frame_id &&
           route.evidence->frame_id == map.frame_id &&
           SameObservationCut(route.evidence->observation_cut,
                              map.observation_cut) &&
           SameObservationCut(route.evidence->observation_baseline,
                              map.observation_baseline) &&
           route.evidence->observation_recovery_generation ==
               map.observation_recovery_generation;
}
}  // namespace
class ExplorationSession::Impl
{
public:
    struct PoseEvent {
        Pose2d pose;   ///< Ground robot pose at the actual acquisition time.
        double stamp;  ///< Acquisition time in the Session clock domain.
        std::uint64_t epoch;  ///< Alignment identity of this pose evidence.
        perception::SensorMask
            localization;  ///< Sources attested by the localization owner.
    };
    /** @brief Compact, immutable acquisition poses for one admitted
     * observation. Retained independently from rolling pose_history_ until this
     * observation is examined or discarded. Validity is bounded by the admitted
     * observation, whose expiry was checked against every authoritative
     * acquisition proof.
     */
    struct PacketPoses {
        std::uint64_t
            epoch;  ///< Alignment shared by the packet and queued observation.
        std::uint64_t
            calibration_revision;  ///< Exact packet calibration identity.
        std::vector<PoseEvent>
            samples;  ///< Sorted exact-time samples, deduplicated by timestamp.
    };
    /** @brief Source-local identity; epochs are verified in the associated
     * value. */
    using ObservationIdentity =
        std::pair<perception::SensorKind, std::uint64_t>;
    struct CancelEvent {
        RequestVersion version;
    };
    struct PhaseCancelEvent {
        PhaseTicket ticket;
    };
    /** @brief One ordinary event correlates its source evidence and base poses.
     */
    struct PacketEvent {
        perception::ObservationPacket
            packet;          ///< Owned real acquisition evidence.
        double admitted_at;  ///< Original enqueue time, never used to renew
                             ///< expiry.
    };
    struct ReturnCache {
        RequestVersion version;
        double length = 0, traveled_distance = 0;
    };
    /** @brief One immutable Vista leg and its admitted execution waypoint. */
    struct ActiveObservationLeg {
        planning::planner::vista::VistaRoute route;
        RouteLookaheadResult waypoint;
        RequestVersion version;
    };
    using Event =
        std::variant<MapReadView, PoseEvent, CoverageUnit, SessionCommand,
                     CancelEvent, PhaseCancelEvent, PacketEvent>;
    struct Job {
        RequestVersion version;
        MapReadView map;
        CoverageState coverage;
        Pose2d pose, home;
        perception::SensorMask sensors;
        double now;
        double pose_stamp;
        std::uint64_t pose_sequence;
        MomentumSampleIdentity motion_identity;
        bool returning;
        bool estimate_only, require_return;
        double traveled_distance;
        std::shared_ptr<std::atomic<bool>> canceled;
        std::optional<ExplorationPhase> phase;
        std::optional<ObservationGoal> selected_goal;
        bool evidence_only =
            false;  ///< Update graph evidence without selection or motion.
        bool continuation = false;  ///< Replan only the admitted goal's route.
        std::optional<Pose2d>
            final_goal;  ///< Immutable adapted route endpoint.
    };
    struct Planned {
        RequestVersion version;
        StrategyDecision decision;
        std::optional<AdaptedObservationGoal> adapted;
        MapReadView map;
        bool returning = false, route_valid = false;
        bool estimate_only = false, return_route_valid = false;
        double return_length = 0, return_from_goal = 0, traveled_distance = 0;
        Pose2d origin;
        std::optional<RouteMapEvidence> graph_evidence;
        std::string error;
        std::optional<ExplorationPhase> phase;
        bool evidence_only = false;  ///< A consumer evidence refresh, not an
                                     ///< exploration decision.
        bool continuation = false;  ///< Result routes an already admitted goal.
        std::optional<planning::planner::vista::VistaRoute> observation_route;
        std::optional<RouteLookaheadResult> observation_waypoint;
    };

    Impl(std::shared_ptr<ExplorationStrategy> strategy,
         std::shared_ptr<const GoalPoseAdapter> adapter,
         std::shared_ptr<const PathProvider> paths,
         ExplorationServices services, ExplorationSessionOptions options)
        : strategy_(std::move(strategy)),
          adapter_(std::move(adapter)),
          paths_(std::move(paths)),
          services_(std::move(services)),
          options_(options),
          keyposes_({options.keypose_spacing_m, 4096}),
          health_(options.mode, options.health),
          completion_(options.completion_rounds) {
        if (!strategy_ || !adapter_ || !paths_ ||
            !services_.submit_navigation || !services_.poll_navigation ||
            !services_.cancel_navigation ||
            options.validated_controller_modes.empty())
            throw std::invalid_argument(
                "exploration requires planning and navigation services");
        if (options.use_line_of_sight_lookahead &&
            !services_.route_line_of_sight_view)
            throw std::invalid_argument(
                "line-of-sight lookahead requires measured geometry");
        if (!options.maximum_events_per_tick ||
            !options.maximum_observations_per_tick)
            throw std::invalid_argument(
                "exploration input processing budgets must be positive");
        if (options.observation_recovery_policy !=
                ObservationRecoveryPolicy::kReobserveScope &&
            options.observation_recovery_policy !=
                ObservationRecoveryPolicy::kRequireOriginalReplay)
            throw std::invalid_argument(
                "unsupported observation recovery policy");
        if (options.allowed_region) {
            const auto& bounds = *options.allowed_region;
            if (!std::isfinite(bounds.min_x) || !std::isfinite(bounds.min_y) ||
                !std::isfinite(bounds.max_x) || !std::isfinite(bounds.max_y) ||
                bounds.min_x >= bounds.max_x || bounds.min_y >= bounds.max_y)
                throw std::invalid_argument(
                    "exploration scope must be finite and have positive area");
        }
        if (options.allowed_polygon &&
            (!options.allowed_region ||
             !PolygonBoundaryWithinBounds(*options.allowed_polygon,
                                          *options.allowed_region)))
            throw std::invalid_argument(
                "exploration polygon must be finite, simple and nondegenerate");
        for (const auto& polygon : options.no_go_areas)
            if (!ValidatePolygonBoundary(polygon))
                throw std::invalid_argument(
                    "exploration no-go polygon must be finite, simple and "
                    "nondegenerate");
        for (double value :
             {options.time_budget, options.distance_budget,
              options.navigation_timeout, options.observation_timeout,
              options.minimum_observation_time, options.position_tolerance,
              options.return_position_tolerance, options.yaw_tolerance,
              options.map_max_age, options.pose_max_age,
              options.maximum_sensor_wait, options.planning_timeout,
              options.map_operation_timeout, options.nominal_speed,
              options.blocked_retry_delay, options.keypose_spacing_m,
              options.lookahead_distance_m})
            if (!std::isfinite(value) || value <= 0)
                throw std::invalid_argument(
                    "invalid exploration timing or platform configuration");
        if (!std::isfinite(options.nominal_angular_speed) ||
            options.nominal_angular_speed <= 0)
            throw std::invalid_argument("invalid platform angular speed");
        if (options.use_line_of_sight_lookahead &&
            !services_.route_line_of_sight_view)
            throw std::invalid_argument(
                "line-of-sight lookahead requires measured geometry");
        if ((options.require_save || options.save_on_cancel) &&
            !services_.cancel_mapping)
            throw std::invalid_argument(
                "mapping requests require versioned cancellation");
        const auto& startup = options.startup_observation;
        if (startup.enabled) {
            if (!services_.request_startup_observation ||
                !services_.poll_startup_observation ||
                !services_.cancel_startup_observation)
                throw std::invalid_argument(
                    "startup observation requires versioned native action "
                    "services");
            for (double value :
                 {startup.target_yaw, startup.time_allowance,
                  startup.deadline_margin, startup.maximum_position_drift,
                  startup.completion_tolerance, startup.maximum_pose_age})
                if (!std::isfinite(value) || value <= 0)
                    throw std::invalid_argument(
                        "startup observation values must be finite and "
                        "positive");
            if (!std::isfinite(startup.support_radius) ||
                startup.support_radius <= 0 ||
                !std::isfinite(startup.support_ring_radius) ||
                startup.support_ring_radius <= startup.support_radius ||
                !std::isfinite(startup.support_timeout) ||
                startup.support_timeout <= 0 || !startup.minimum_support_cells)
                throw std::invalid_argument(
                    "invalid startup support-closure policy");
            if (std::abs(startup.target_yaw) >
                2 * autonomy::navigation::kPi + 1e-9)
                throw std::invalid_argument(
                    "startup observation target exceeds the native spin "
                    "envelope");
        }
        if (!std::isfinite(options.return_margin) ||
            options.return_margin < 0 ||
            !std::isfinite(options.frontier_observation_height) ||
            !options.minimum_frontier_cluster_size ||
            options.minimum_frontier_cluster_size > 1000000)
            throw std::invalid_argument(
                "invalid exploration return or observation configuration");
        worker_ = std::thread([this] { Work(); });
    }

    ~Impl() {
        CancelPlanning();
        {
            std::lock_guard<std::mutex> lock(work_mutex_);
            stopping_ = true;
        }
        work_ready_.notify_all();
        if (worker_.joinable())
            worker_.join();
        CancelNavigation();
        CancelStartupObservation();
        CancelMapOperation();
        CancelRecoveryOperation();
    }

    bool Enqueue(Event event) {
        std::lock_guard<std::mutex> lock(events_mutex_);
        auto& queue = std::holds_alternative<SessionCommand>(event) ||
                              std::holds_alternative<CancelEvent>(event) ||
                              std::holds_alternative<PhaseCancelEvent>(event)
                          ? priority_events_
                          : events_;
        if (queue.size() >= 128)
            return false;
        if (&queue == &events_ && accepted_event_sequence_ ==
                                      std::numeric_limits<std::uint64_t>::max())
            return false;
        queue.push_back(std::move(event));
        if (&queue == &events_)
            ++accepted_event_sequence_;
        return true;
    }

    /** @brief Admit a whole source packet, recording capacity/validity loss. */
    bool PushPacket(perception::ObservationPacket packet, double now) {
        std::lock_guard<std::mutex> lock(events_mutex_);
        if (!(perception::Mask(options_.mode) &
              perception::Mask(packet.observation.sensor)) ||
            packet.observation.map_epoch != admission_epoch_)
            return false;
        if (!packet.Valid(now) || events_.size() >= 128 ||
            accepted_event_sequence_ ==
                std::numeric_limits<std::uint64_t>::max()) {
            observations_.Reject(packet.observation.sensor,
                                 packet.observation.sequence);
            return false;
        }
        events_.push_back(PacketEvent{std::move(packet), now});
        ++accepted_event_sequence_;
        return true;
    }

    /** @brief Share immutable raw depth with its original provenance and
     * deadline. */
    std::vector<std::shared_ptr<const perception::LocalizedDepthView>>
    DepthViews(double now) const {
        std::vector<std::shared_ptr<const perception::LocalizedDepthView>>
            result;
        std::lock_guard<std::mutex> lock(depth_mutex_);
        for (const auto& view : depth_history_)
            if (view->Valid(now))
                result.push_back(view);
        return result;
    }

    void Start(const Pose2d& home, std::uint64_t epoch, double now) {
        if (!autonomy::navigation::IsFinite(home) || !epoch ||
            !std::isfinite(now))
            throw std::invalid_argument("invalid exploration start");
        if (snapshot_.state != ExplorationState::kIdle &&
            snapshot_.state != ExplorationState::kFinished)
            throw std::logic_error("exploration session already active");
        CancelPlanning();
        CancelNavigation();
        CancelStartupObservation();
        CancelMapOperation();
        if (execution_fault_)
            throw std::runtime_error(
                "previous navigation cancellation was not confirmed");
        CancelRecoveryOperation();
        {
            std::lock_guard<std::mutex> lock(events_mutex_);
            events_.clear();
            priority_events_.clear();
            accepted_event_sequence_ = processed_event_sequence_ = 0;
            admission_epoch_ = epoch;
        }
        {
            std::lock_guard<std::mutex> lock(depth_mutex_);
            depth_history_.clear();
        }
        {
            std::lock_guard<std::mutex> lock(work_mutex_);
            planned_results_.clear();
        }
        snapshot_ = {};
        snapshot_.state = ExplorationState::kInitializing;
        snapshot_.session_id = NewSessionIdentity();
        snapshot_.startup_observation_required =
            options_.startup_observation.enabled;
        snapshot_.startup_observation_confirmed =
            !options_.startup_observation.enabled;
        snapshot_.startup_observation_reason =
            options_.startup_observation.enabled
                ? "bounded in-place startup observation is pending"
                : "startup observation is disabled";
        startup_observation_confirmed_ = !options_.startup_observation.enabled;
        startup_observation_failed_ = false;
        started_ = last_now_ = now;
        paused_time_ = 0;
        wait_started_.reset();
        paused_at_.reset();
        owner_.reset();
        map_.reset();
        map_operation_.reset();
        return_cache_.reset();
        pose_history_.clear();
        pose_history_.push_back({home, now, epoch, 0});
        view_evidence_.clear();
        view_started_.clear();
        current_view_ = 0;
        paused_state_.reset();
        pose_ = home_ = home;
        home_valid_ = true;
        pose_stamp_ = now;
        pose_sequence_ = 1;
        pose_epoch_ = home_epoch_ = epoch;
        localization_ = 0;
        coverage_.Reset(epoch);
        dormant_requirements_.clear();
        observations_.Reset(epoch);
        keyposes_.Reset(epoch);
        packet_poses_.clear();
        observed_cut_ = {};
        valid_observation_cut_ = {};
        recovery_loss_.reset();
        observation_baseline_ = {};
        observation_recovery_generation_ = 0;
        loss_recovery_attempts_ = 0;
        health_ =
            perception::SensorHealthMonitor(options_.mode, options_.health);
        completion_.Reset();
        completion_version_.reset();
        confirmation_round_ = 0;
        worker_reset_requested_.store(true);
        active_phase_.reset();
        observation_rotation_.reset();
        phase_history_.clear();
        selected_version_.reset();
        prepared_version_.reset();
        resume_phase_ = false;
        finishing_ = false;
        recovery_count_ = 0;
        selection_after_ = now;
        Publish();
    }

    void CancelPlanning() {
        if (planning_cancel_)
            planning_cancel_->store(true);
        planning_.reset();
        std::lock_guard<std::mutex> lock(work_mutex_);
        if (queued_job_)
            queued_job_->canceled->store(true);
        queued_job_.reset();
    }

    template <typename Callback>
    void CancelService(const Callback& callback, const RequestVersion& version,
                       const char* reason, bool motion) noexcept {
        try {
            if (callback)
                callback(version);
        } catch (...) {
            execution_fault_ = execution_fault_ || motion;
            try {
                snapshot_.reason = reason;
            } catch (...) {
            }
        }
    }

    void CancelNavigation() noexcept {
        if (navigation_) {
            const auto version = *navigation_;
            navigation_.reset();
            CancelService(services_.cancel_navigation, version,
                          "navigation cancellation callback failed", true);
        }
    }

    /** @brief Cancel only the matching native startup action and revoke its
     * movement admission. */
    void CancelStartupObservation() noexcept {
        if (!startup_observation_)
            return;
        const auto version = *startup_observation_;
        startup_observation_.reset();
        CancelService(services_.cancel_startup_observation, version,
                      "startup observation cancellation callback failed", true);
    }

    /** @brief Request one deployment-bounded, zero-translation yaw sweep. */
    bool BeginStartupObservation(double now) {
        if (!options_.startup_observation.enabled ||
            startup_observation_confirmed_)
            return true;
        if (startup_observation_ || startup_observation_failed_)
            return false;
        const auto& policy = options_.startup_observation;
        startup_observation_ =
            Version(now, policy.time_allowance + policy.deadline_margin);
        snapshot_.state = ExplorationState::kStartupObserving;
        snapshot_.startup_observation_reason =
            "executing bounded in-place startup observation";
        if (services_.request_startup_observation(*startup_observation_,
                                                  policy.target_yaw,
                                                  policy.time_allowance))
            return true;
        startup_observation_.reset();
        // The native controller/transport can still be initializing on the
        // first task tick. A rejected request is retryable until the phase
        // deadline; do not permanently poison the session on that race.
        snapshot_.startup_observation_reason =
            "native startup rotation action is not ready";
        snapshot_.reason = snapshot_.startup_observation_reason;
        return false;
    }

    /** @brief Validate a terminal native rotation only after measured
     * completion evidence. */
    void PollStartupObservation(double now) {
        if (!startup_observation_)
            return;
        if (now > startup_observation_->valid_until) {
            CancelStartupObservation();
            startup_observation_failed_ = true;
            snapshot_.startup_observation_reason =
                "startup observation deadline exceeded";
            snapshot_.reason = snapshot_.startup_observation_reason;
            FinishPhase(PhaseStatus::kFailed, snapshot_.reason);
            if (!BtDriven())
                BeginFinish(ExplorationOutcome::kIncompleteObservation, now);
            return;
        }
        const auto reply = services_.poll_startup_observation();
        if (!reply || !reply->version.SameRequest(*startup_observation_))
            return;
        startup_observation_.reset();
        // A bounded sensor/viewpoint cancellation is not a terminal action
        // failure. Keep the startup phase eligible for a later retry once the
        // input owner is healthy again instead of poisoning it forever.
        if (!reply->success &&
            reply->reason.find("canceled") != std::string::npos) {
            startup_observation_failed_ = false;
            snapshot_.startup_observation_reason = reply->reason;
            snapshot_.reason = reply->reason;
            return;
        }
        startup_observation_confirmed_ = reply->success;
        startup_observation_failed_ = !reply->success;
        snapshot_.startup_observation_confirmed = reply->success;
        snapshot_.startup_observation_reason =
            reply->reason.empty()
                ? (reply->success ? "startup observation confirmed"
                                  : "startup observation failed")
                : reply->reason;
        snapshot_.reason = snapshot_.startup_observation_reason;
        if (reply->success) {
            FinishPhase(PhaseStatus::kSucceeded, snapshot_.reason);
            selection_after_ = now;
            snapshot_.state = finishing_ ? ExplorationState::kFinishing
                                         : ExplorationState::kExploring;
        } else {
            FinishPhase(PhaseStatus::kFailed, snapshot_.reason);
            if (!BtDriven())
                BeginFinish(ExplorationOutcome::kIncompleteObservation, now);
        }
    }

    RequestVersion Version(double now, double duration) {
        RequestVersion version;
        version.session_id = snapshot_.session_id;
        version.request_id = ++request_counter_;
        version.goal_version =
            snapshot_.goal ? snapshot_.goal->intent.goal_version : 0;
        version.map_epoch = map_ ? map_->map_epoch : pose_epoch_;
        version.geometry_revision = map_ ? map_->geometry_revision : 0;
        version.sensor_epoch = snapshot_.sensors.sensor_epoch;
        version.coverage_revision = coverage_.revision();
        version.requirements_revision = coverage_.requirements_revision();
        {
            std::lock_guard<std::mutex> lock(events_mutex_);
            version.input_event_sequence = accepted_event_sequence_;
        }
        version.observation_cut = observations_.accepted_cut();
        version.observation_baseline = observation_baseline_;
        version.observation_recovery_generation =
            observation_recovery_generation_;
        version.valid_until = now + duration;
        return version;
    }

    bool BtDriven() const {
        return options_.driver == ExplorationDriver::kBehaviorTree;
    }

    /** @brief Whether a live observation phase still owns a safe current
     * target. Coverage loss can add work while the robot is moving toward an
     * already admitted viewpoint. Geometry changes are authoritative only when
     * the platform revalidator rejects that target; a new requirement revision
     * by itself must not stop safe motion that can recover the missing
     * evidence.
     */
    bool ActiveObservationTargetValid() const {
        if (!active_phase_ ||
            active_phase_->phase != ExplorationPhase::kObserve ||
            !snapshot_.goal || !prepared_version_ || !map_ || finishing_ ||
            prepared_version_->map_epoch != map_->map_epoch ||
            prepared_version_->sensor_epoch != snapshot_.sensors.sensor_epoch)
            return false;
        return prepared_version_->geometry_revision ==
                   map_->geometry_revision ||
               (services_.navigation_still_valid &&
                services_.navigation_still_valid(*snapshot_.goal, pose_,
                                                 *map_));
    }

    void FinishPhase(PhaseStatus status, const std::string& reason) {
        if (!active_phase_)
            return;
        AINFO << "EXPLORATION_PHASE result=" << static_cast<int>(status)
              << " phase=" << static_cast<int>(active_phase_->phase)
              << " reason=" << reason;
        for (auto& result : phase_history_) {
            if (result.ticket.SameRequest(*active_phase_)) {
                result.status = status;
                result.reason = reason;
                break;
            }
        }
        active_phase_.reset();
        snapshot_.active_phase.reset();
        resume_phase_ = false;
    }

    /** @brief Ends the active target and drops evidence contradicted while it
     * ran. */
    void ReleaseGoal() {
        snapshot_.goal.reset();
        for (const auto& id : dormant_requirements_)
            coverage_.Remove(id);
        dormant_requirements_.clear();
    }

    void CancelActivePhase(PhaseStatus status, const std::string& reason) {
        if (!active_phase_)
            return;
        const auto phase = active_phase_->phase;
        CancelObservationRotation();
        CancelPlanning();
        CancelNavigation();
        if (phase == ExplorationPhase::kStartupObservation)
            CancelStartupObservation();
        if (phase == ExplorationPhase::kSave)
            CancelMapOperation();
        if (phase == ExplorationPhase::kSelect ||
            phase == ExplorationPhase::kAdapt) {
            snapshot_.selection.reset();
            selected_version_.reset();
        }
        if (phase != ExplorationPhase::kSave &&
            phase != ExplorationPhase::kReturn) {
            ReleaseGoal();
            prepared_version_.reset();
        }
        FinishPhase(status, reason);
        if (snapshot_.state != ExplorationState::kPaused)
            snapshot_.state = finishing_ ? ExplorationState::kFinishing
                                         : ExplorationState::kExploring;
    }

    void AfterReturn(double now) {
        if (!BtDriven()) {
            BeginRefresh(now);
            return;
        }
        FinishPhase(snapshot_.result.returned == ReturnOutcome::kReturned
                        ? PhaseStatus::kSucceeded
                        : PhaseStatus::kFailed,
                    snapshot_.result.returned == ReturnOutcome::kReturned
                        ? "return arrival confirmed"
                        : "return failed");
        snapshot_.state = ExplorationState::kFinishing;
    }

    std::optional<PhaseTicket> RequestPhase(ExplorationPhase phase,
                                            double now) {
        if (!BtDriven() || active_phase_)
            return {};
        if (!std::isfinite(now) || now < last_now_)
            throw std::invalid_argument(
                "phase requests require a monotonic clock");
        if (!owner_)
            owner_ = std::this_thread::get_id();
        if (*owner_ != std::this_thread::get_id())
            throw std::logic_error(
                "phase requests require the task owner thread");
        double duration = options_.planning_timeout;
        if (phase == ExplorationPhase::kObserve)
            duration =
                (options_.navigation_timeout + options_.observation_timeout) *
                std::max<std::size_t>(
                    1, snapshot_.goal
                           ? snapshot_.goal->intent.required_views.size()
                           : 1);
        else if (phase == ExplorationPhase::kStartupObservation)
            duration = options_.startup_observation.time_allowance +
                       options_.startup_observation.deadline_margin;
        else if (phase == ExplorationPhase::kReturn)
            duration += options_.navigation_timeout;
        else if (phase == ExplorationPhase::kSave)
            duration = 2 * options_.map_operation_timeout;
        const PhaseTicket ticket{Version(now, duration), phase};
        active_phase_ = ticket;
        snapshot_.active_phase = ticket;
        phase_history_.push_back({ticket, PhaseStatus::kPending, {}});
        if (phase_history_.size() > 32)
            phase_history_.pop_front();
        auto finish = [&](PhaseStatus status, const char* reason) {
            FinishPhase(status, reason);
            Publish();
            return std::optional<PhaseTicket>{ticket};
        };
        if ((phase == ExplorationPhase::kReturn &&
             (!options_.require_return ||
              snapshot_.result.returned == ReturnOutcome::kReturned)) ||
            (phase == ExplorationPhase::kSave &&
             (!options_.require_save ||
              snapshot_.result.saved == MapSaveOutcome::kSaved))) {
            if (phase == ExplorationPhase::kSave && finishing_)
                snapshot_.state = ExplorationState::kFinished;
            return finish(PhaseStatus::kSucceeded,
                          "phase is disabled or already confirmed");
        }
        if (snapshot_.state == ExplorationState::kIdle ||
            snapshot_.state == ExplorationState::kFinished)
            return finish(PhaseStatus::kFailed, "session is not active");
        if (snapshot_.state == ExplorationState::kPaused)
            return finish(PhaseStatus::kRetryable, "session paused");
        if (recovery_operation_)
            return finish(
                PhaseStatus::kRetryable,
                "scope recovery still owns the mapper evidence request");
        if (phase != ExplorationPhase::kSave &&
            (!snapshot_.sensors.ready || !PoseReady(now) || execution_fault_)) {
            if (phase == ExplorationPhase::kReturn && finishing_) {
                snapshot_.result.returned = ReturnOutcome::kReturnFailed;
                return finish(
                    PhaseStatus::kFailed,
                    "return cannot execute without current motion evidence");
            }
            return finish(PhaseStatus::kRetryable,
                          "current sensing, pose or execution is unavailable");
        }
        try {
            switch (phase) {
                case ExplorationPhase::kStartupObservation:
                    if (!options_.startup_observation.enabled ||
                        startup_observation_confirmed_)
                        return finish(PhaseStatus::kSucceeded,
                                      "startup observation is not required");
                    if (startup_observation_failed_ ||
                        !BeginStartupObservation(now))
                        return finish(
                            PhaseStatus::kRetryable,
                            snapshot_.startup_observation_reason.c_str());
                    break;
                case ExplorationPhase::kSelect:
                    if (!startup_observation_confirmed_)
                        return finish(PhaseStatus::kRetryable,
                                      "startup observation is not confirmed");
                    if (finishing_ || now < selection_after_)
                        return finish(PhaseStatus::kRetryable,
                                      "finishing or awaiting a bounded retry");
                    snapshot_.selection.reset();
                    ReleaseGoal();
                    selected_version_.reset();
                    prepared_version_.reset();
                    QueuePlanning(now, false, false, phase);
                    break;
                case ExplorationPhase::kAdapt:
                    if (finishing_ || !snapshot_.selection ||
                        !selected_version_ ||
                        selected_version_->map_epoch != map_->map_epoch ||
                        selected_version_->sensor_epoch !=
                            snapshot_.sensors.sensor_epoch)
                        return finish(
                            PhaseStatus::kInvalidated,
                            "selected observation intent is no longer current");
                    // The selected intent is a spatial observation requirement,
                    // not a committed path. Adapt it against the latest
                    // same-epoch map; execution is separately revalidated
                    // before motion admission.
                    QueuePlanning(now, false, false, phase);
                    break;
                case ExplorationPhase::kObserve:
                    if (finishing_ || !snapshot_.goal || !prepared_version_ ||
                        prepared_version_->map_epoch != map_->map_epoch ||
                        prepared_version_->sensor_epoch !=
                            snapshot_.sensors.sensor_epoch ||
                        prepared_version_->requirements_revision !=
                            coverage_.requirements_revision() ||
                        (prepared_version_->geometry_revision !=
                             map_->geometry_revision &&
                         (!services_.navigation_still_valid ||
                          !services_.navigation_still_valid(*snapshot_.goal,
                                                            pose_, *map_))))
                        return finish(
                            PhaseStatus::kInvalidated,
                            "adapted observation pose is no longer valid");
                    CancelObservationRotation();
                    view_evidence_.assign(
                        snapshot_.goal->intent.required_views.size(), {});
                    view_started_.assign(view_evidence_.size(), {});
                    current_view_ = 0;
                    snapshot_.observed_view_count = 0;
                    snapshot_.state = ExplorationState::kExploring;
                    if (!observation_leg_ ||
                        observation_leg_->version.geometry_revision !=
                            map_->geometry_revision) {
                        observation_leg_.reset();
                        QueueObservationContinuation(
                            now, ExplorationPhase::kObserve);
                    } else {
                        SubmitNavigation(observation_leg_->waypoint.pose, now);
                    }
                    break;
                case ExplorationPhase::kReturn:
                    if (!finishing_)
                        return finish(
                            PhaseStatus::kFailed,
                            "exploration has not reached a finish outcome");
                    if (!home_valid_) {
                        snapshot_.result.returned =
                            ReturnOutcome::kReturnFailed;
                        return finish(PhaseStatus::kFailed,
                                      "home alignment is unavailable");
                    }
                    if (At(home_)) {
                        snapshot_.result.returned = ReturnOutcome::kReturned;
                        AfterReturn(now);
                    } else
                        QueuePlanning(now, true, false, phase);
                    break;
                case ExplorationPhase::kSave:
                    if (!finishing_ || (options_.require_return &&
                                        snapshot_.result.returned ==
                                            ReturnOutcome::kNotRequested))
                        return finish(
                            PhaseStatus::kFailed,
                            "return outcome must precede required saving");
                    BeginRefresh(now);
                    break;
            }
        } catch (...) {
            CancelActivePhase(PhaseStatus::kFailed,
                              "phase service raised an exception");
            snapshot_.reason = "phase service raised an exception";
        }
        Publish();
        return ticket;
    }

    PhaseResult PhaseState(const PhaseTicket& ticket) const {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        for (const auto& result : published_phases_)
            if (result.ticket.SameRequest(ticket))
                return result;
        return {ticket, PhaseStatus::kInvalidated,
                "phase ticket is unknown or belongs to an old session"};
    }

    void QueuePlanning(double now, bool returning = false,
                       bool estimate_only = false,
                       std::optional<ExplorationPhase> phase = {},
                       bool evidence_only = false, bool continuation = false) {
        if (!map_)
            return;
        planning_cancel_ = std::make_shared<std::atomic<bool>>(false);
        planning_ = Version(now, options_.planning_timeout);
        Job job{*planning_,
                *map_,
                coverage_,
                pose_,
                home_,
                snapshot_.sensors.active_sensors,
                now,
                pose_stamp_,
                pose_sequence_,
                {planning_->session_id, map_->map_epoch,
                 observation_recovery_generation_ + 1, map_->frame_id},
                returning,
                estimate_only,
                options_.require_return,
                snapshot_.traveled_distance,
                planning_cancel_,
                phase,
                snapshot_.selection,
                evidence_only,
                continuation,
                continuation && snapshot_.goal
                    ? std::optional<Pose2d>(snapshot_.goal->execution_pose)
                    : std::nullopt};
        {
            std::lock_guard<std::mutex> lock(work_mutex_);
            queued_job_ = std::move(job);
        }
        work_ready_.notify_one();
        if (!estimate_only && !evidence_only)
            snapshot_.state = returning ? ExplorationState::kReturning
                                        : ExplorationState::kReplanning;
    }

    /** @brief Replan the current goal from the latest measured pose and map. */
    void QueueObservationContinuation(
        double now, std::optional<ExplorationPhase> phase = {}) {
        if (!snapshot_.goal || planning_)
            return;
        QueuePlanning(now, false, false, phase, false, true);
    }

    void Work() {
        for (;;) {
            std::optional<Job> job;
            {
                std::unique_lock<std::mutex> lock(work_mutex_);
                work_ready_.wait(lock, [this] {
                    return stopping_ || queued_job_.has_value();
                });
                if (stopping_)
                    return;
                job = std::move(queued_job_);
                queued_job_.reset();
            }
            Planned result;
            result.version = job->version;
            result.map = job->map;
            result.returning = job->returning;
            result.estimate_only = job->estimate_only;
            result.phase = job->phase;
            result.evidence_only = job->evidence_only;
            result.origin = job->pose;
            result.traveled_distance = job->traveled_distance;
            auto canceled = [token = job->canceled] { return token->load(); };
            try {
                if (!canceled() &&
                    (worker_reset_requested_.exchange(false) ||
                     worker_session_ != job->version.session_id)) {
                    strategy_->Reset();
                    worker_session_ = job->version.session_id;
                    worker_motion_sequence_ = 0;
                }
                if (!canceled() && services_.update_route_map)
                    services_.update_route_map(job->map);
                if (!canceled() && job->require_return && !job->evidence_only) {
                    const auto route =
                        paths_->Route(job->pose, job->home, canceled);
                    result.route_valid =
                        route.success &&
                        route.map_epoch == job->map.map_epoch &&
                        route.geometry_revision == job->map.geometry_revision;
                    result.return_route_valid = result.route_valid;
                    result.return_length = route.length;
                }
                if (!canceled() && !job->returning && !job->estimate_only &&
                    !job->evidence_only) {
                    auto select_waypoint = [&](planning::planner::vista::
                                                   VistaRoute route,
                                               const Pose2d& goal) {
                        if (!RouteEvidenceMatchesMap(route, job->map))
                            throw std::runtime_error(
                                "observation route evidence is stale");
                        std::optional<RouteLineOfSightView> line_of_sight;
                        if (options_.use_line_of_sight_lookahead) {
                            line_of_sight = services_.route_line_of_sight_view(
                                job->map, canceled);
                            if (!line_of_sight)
                                throw std::runtime_error(
                                    "route line of sight is unavailable");
                        }
                        const auto waypoint = RouteLookaheadSelector{}.Select(
                            route, job->pose, goal,
                            options_.lookahead_distance_m, job->map.map_epoch,
                            job->map.geometry_revision, job->map.frame_id,
                            line_of_sight ? line_of_sight->field_generation : 0,
                            line_of_sight ? &*line_of_sight : nullptr,
                            canceled);
                        if (waypoint.status != RouteLookaheadStatus::kSelected)
                            throw std::runtime_error(
                                "observation route lookahead is unavailable");
                        result.observation_route = std::move(route);
                        result.observation_waypoint = waypoint;
                    };
                    if (job->continuation) {
                        result.continuation = true;
                        if (!job->final_goal)
                            throw std::runtime_error(
                                "observation continuation has no final goal");
                        select_waypoint(
                            paths_->Route(job->pose, *job->final_goal,
                                          canceled),
                            *job->final_goal);
                    } else {
                        if (job->phase == ExplorationPhase::kAdapt)
                            result.decision.goal = job->selected_goal;
                        else {
                            if (job->pose_sequence != worker_motion_sequence_) {
                                strategy_->UpdateMeasuredMotion(
                                    job->pose, job->pose_stamp,
                                    job->motion_identity);
                                worker_motion_sequence_ = job->pose_sequence;
                            }
                            result.decision = strategy_->Select(
                                job->pose, job->home, job->map, job->coverage,
                                job->sensors, job->now, canceled);
                        }
                        if (result.decision.goal && !canceled() &&
                            job->phase != ExplorationPhase::kSelect)
                            result.adapted = adapter_->Adapt(
                                *result.decision.goal, job->pose, job->map,
                                job->coverage, job->sensors, canceled);
                        if (result.adapted && !canceled())
                            select_waypoint(result.adapted->route,
                                            result.adapted->execution_pose);
                        if (result.adapted && job->require_return &&
                            !canceled()) {
                            const auto route =
                                paths_->Route(result.adapted->execution_pose,
                                              job->home, canceled);
                            if (!route.success ||
                                route.map_epoch != job->map.map_epoch ||
                                route.geometry_revision !=
                                    job->map.geometry_revision)
                                result.return_route_valid = false;
                            result.return_from_goal = route.length;
                        }
                    }
                }
                // Estimate-only work must never contend with or invalidate an
                // active observation phase merely to refresh graph evidence.
                // The next route-bearing planning job performs that proof.
                if (!job->estimate_only)
                    result.graph_evidence = paths_->Evidence();
            } catch (const std::exception& error) {
                result.error = error.what();
            } catch (...) {
                result.error =
                    "non-standard exception from exploration worker service";
            }
            if (!canceled()) {
                std::lock_guard<std::mutex> lock(work_mutex_);
                if (planned_results_.size() < 8)
                    planned_results_.push_back(std::move(result));
            }
        }
    }

    bool PoseReady(double now) const {
        return map_ && pose_epoch_ == map_->map_epoch && localization_ &&
               std::isfinite(pose_stamp_) && now >= pose_stamp_ &&
               now - pose_stamp_ <= options_.pose_max_age &&
               now >= map_->source_stamp &&
               now - map_->source_stamp <= options_.map_max_age;
    }

    void CancelMapOperation() noexcept {
        if (map_operation_) {
            const auto version = *map_operation_;
            map_operation_.reset();
            CancelService(services_.cancel_mapping, version,
                          "mapping cancellation callback failed", false);
        }
    }

    /** @brief Cancels only the recovery operation and retains unresolved loss
     * history. */
    void CancelRecoveryOperation() noexcept {
        if (recovery_operation_) {
            const auto version = *recovery_operation_;
            recovery_operation_.reset();
            CancelService(services_.cancel_mapping, version,
                          "observation recovery cancellation callback failed",
                          false);
        }
        recovery_reply_.reset();
    }

    /** @brief Checks that actual map/graph evidence describes the requested
     * suffix. */
    template <typename Evidence>
    bool BaselineMatches(const Evidence& evidence,
                         const RequestVersion& version) const {
        return evidence.observation_baseline == version.observation_baseline &&
               evidence.observation_recovery_generation ==
                   version.observation_recovery_generation;
    }

    /** @brief Rejects receipts that certify another baseline or only a shorter
     * suffix. */
    bool ReceiptCovers(const MappingReply& reply,
                       const RequestVersion& version) const {
        const auto& receipt = reply.version;
        if (!receipt.SameRequest(version) ||
            !BaselineMatches(receipt, version) ||
            receipt.geometry_revision != version.geometry_revision ||
            receipt.requirements_revision != version.requirements_revision ||
            receipt.input_event_sequence != version.input_event_sequence ||
            !(receipt.observation_cut == version.observation_cut) ||
            receipt.recovery_region.has_value() !=
                version.recovery_region.has_value())
            return false;
        if (version.recovery_region) {
            const auto& actual = *receipt.recovery_region;
            const auto& requested = *version.recovery_region;
            if (actual.min_x != requested.min_x ||
                actual.min_y != requested.min_y ||
                actual.max_x != requested.max_x ||
                actual.max_y != requested.max_y)
                return false;
        }
        return reply.fusion_confirmed &&
               reply.fused_cut.Covers(version.observation_cut);
    }

    /** @brief Excludes pre-reopening queued observations from replacement
     * coverage. */
    bool ReplacementEligible(
        const perception::SensorObservation& observation) const {
        if (!recovery_loss_ || !(recovery_loss_->pending_sources &
                                 perception::Mask(observation.sensor)))
            return true;
        const auto index =
            observation.sensor == perception::SensorKind::kLidar ? 0U : 1U;
        return observation.sequence >
               std::max(recovery_loss_->admitted_cut.sequence[index],
                        recovery_loss_->last_loss.sequence[index]);
    }

    /** @brief Reopens unknown-scope source evidence before admitting
     * replacements. Loss detected during Drain is handled again afterwards,
     * discarding any evidence from that earlier admission boundary.
     * Other-source ledger quality survives; full task-scope confirmation still
     * requires the mapping owner.
     */
    void SyncObservationLoss(double now) {
        const auto loss = observations_.LossSnapshot();
        if (!loss.pending_sources)
            return;
        const auto geometry = map_ ? map_->geometry_revision : 0;
        if (recovery_loss_ && recovery_loss_->generation == loss.generation &&
            recovery_loss_->map_epoch == loss.map_epoch &&
            recovery_sensor_epoch_ == snapshot_.sensors.sensor_epoch) {
            // One open gap keeps its identity while admitted observations keep
            // advancing the map and the work domain. Treating those revisions
            // as a fresh loss retired the in-flight recovery request and
            // restarted the plan on every frame, so one real gap was replayed
            // as hundreds of plans and no sweep could finish. Geometry drift is
            // bookkeeping here: the recovery poll still retires a request whose
            // certification target moved backwards, and a new generation,
            // alignment epoch or sensing epoch falls through to the full
            // invalidation below.
            recovery_geometry_ = geometry;
            const auto requirements = coverage_.requirements_revision();
            if (recovery_requirements_ != requirements) {
                // A new work item can be marked satisfied by measurements that
                // belong to the still-open gap. Reopen those source
                // requirements without abandoning the observation motion that
                // is already acquiring the replacement views.
                coverage_.InvalidateSources(loss.pending_sources);
                // Invalidation bumps the work revision itself; recording the
                // pre-invalidation value would re-enter this branch forever and
                // invalidate every in-flight plan.
                recovery_requirements_ = coverage_.requirements_revision();
                worker_reset_requested_.store(true);
                snapshot_.completion_confirmed = false;
            }
            return;
        }
        RevokeCompletion();
        if (finishing_)
            return;  // Partial/canceled results retain loss but do not restart
                     // exploration.
        CancelRecoveryOperation();
        coverage_.InvalidateSources(loss.pending_sources);
        recovery_loss_ = loss;
        recovery_geometry_ = geometry;
        recovery_requirements_ = coverage_.requirements_revision();
        recovery_sensor_epoch_ = snapshot_.sensors.sensor_epoch;
        loss_recovery_attempts_ = 0;
        loss_recovery_after_ = now;
        worker_reset_requested_.store(true);
        snapshot_.completion_confirmed = false;
        if (ActiveObservationTargetValid()) {
            // The admitted route remains safe and can acquire the replacement
            // views. Preserve its phase, navigation request and dwell evidence;
            // only refresh the work-domain version used by completion checks.
            prepared_version_->geometry_revision = map_->geometry_revision;
            prepared_version_->coverage_revision = coverage_.revision();
            prepared_version_->requirements_revision =
                coverage_.requirements_revision();
            snapshot_.reason =
                "observation loss reopened coverage; safe observation motion "
                "retained";
            AINFO << "EXPLORATION_LOSS action=retain_observation_goal mask="
                  << static_cast<unsigned>(loss.pending_sources)
                  << " goal=" << snapshot_.goal->intent.goal_id;
            return;
        }
        CancelActivePhase(PhaseStatus::kInvalidated,
                          "observation loss reopened task requirements");
        CancelPlanning();
        CancelNavigation();
        ReleaseGoal();
        snapshot_.selection.reset();
        selected_version_.reset();
        prepared_version_.reset();
        view_evidence_.clear();
        view_started_.clear();
        current_view_ = 0;
        snapshot_.state = ExplorationState::kExploring;
        snapshot_.reason =
            "observation loss requires new measured coverage and full scope "
            "confirmation";
        AINFO << "EXPLORATION_LOSS action=replan mask="
              << static_cast<unsigned>(loss.pending_sources);
        if (StartRecoveryRotation(now))
            snapshot_.reason = "observation loss recovery rotation started";
    }

    /** @brief Starts a bounded, explicitly scoped mapper recovery after a fresh
     * sweep. */
    void BeginLossRecovery(const RequestVersion& planned, double now) {
        if (!recovery_loss_ || recovery_operation_ ||
            now < loss_recovery_after_)
            return;
        if (!options_.allowed_region || !services_.request_map_refresh ||
            !services_.poll_map_refresh || !services_.cancel_mapping) {
            snapshot_.reason =
                "loss recovery requires an explicit task scope and a real "
                "mapper barrier";
            return;
        }
        if (loss_recovery_attempts_ >= options_.maximum_recoveries) {
            snapshot_.reason =
                "observation recovery exhausted its bounded mapper attempts";
            return;
        }
        if (!valid_observation_cut_.Covers(planned.observation_cut))
            return;
        for (unsigned i = 0; i < 2; ++i)
            if ((recovery_loss_->pending_sources & (1U << i)) &&
                (!(planned.observation_cut.present & (1U << i)) ||
                 planned.observation_cut.sequence[i] <=
                     std::max(recovery_loss_->admitted_cut.sequence[i],
                              recovery_loss_->last_loss.sequence[i])))
                return;
        auto version = Version(now, options_.map_operation_timeout);
        version.observation_cut = planned.observation_cut;
        version.input_event_sequence = planned.input_event_sequence;
        version.observation_baseline = recovery_loss_->admitted_cut;
        version.observation_recovery_generation = recovery_loss_->generation;
        version.recovery_loss = recovery_loss_;
        version.recovery_region = options_.allowed_region;
        recovery_operation_ = version;
        if (!services_.request_map_refresh(version)) {
            ++loss_recovery_attempts_;
            CancelRecoveryOperation();
            loss_recovery_after_ = now + options_.blocked_retry_delay;
            snapshot_.reason = "mapper rejected observation scope recovery";
        }
    }

    /** @brief Commits recovery only after a matched scope receipt and real
     * map/graph evidence. */
    void PollLossRecovery(double now) {
        if (!recovery_operation_ || !recovery_loss_)
            return;
        const auto version = *recovery_operation_;
        const auto loss = observations_.LossSnapshot();
        const bool broken =
            now > version.valid_until || !map_ ||
            version.map_epoch != map_->map_epoch ||
            loss.generation != recovery_loss_->generation ||
            loss.map_epoch != recovery_loss_->map_epoch ||
            !loss.pending_sources ||
            version.sensor_epoch != snapshot_.sensors.sensor_epoch ||
            !snapshot_.sensors.ready || !PoseReady(now) ||
            PendingInvalidation();
        // A live map retires the receipt it was requested against. That is a
        // re-based certification target, not a failed recovery, so it must not
        // consume the bounded failure budget while mapping stays active.
        const bool rebased =
            !broken && (map_->geometry_revision < version.geometry_revision ||
                        version.requirements_revision !=
                            coverage_.requirements_revision());
        if (broken || rebased) {
            if (!rebased)
                ++loss_recovery_attempts_;
            CancelRecoveryOperation();
            loss_recovery_after_ = now + options_.blocked_retry_delay;
            snapshot_.reason =
                "observation recovery dependency or deadline changed";
            return;
        }
        if (!recovery_reply_) {
            const auto reply = services_.poll_map_refresh();
            if (reply && reply->version.SameRequest(version))
                recovery_reply_ = reply;
        }
        if (!recovery_reply_)
            return;
        if (!recovery_reply_->success ||
            !ReceiptCovers(*recovery_reply_, version) ||
            !recovery_reply_->recovery_scope_confirmed ||
            (options_.observation_recovery_policy ==
                 ObservationRecoveryPolicy::kRequireOriginalReplay &&
             !recovery_reply_->original_inputs_replayed)) {
            ++loss_recovery_attempts_;
            CancelRecoveryOperation();
            loss_recovery_after_ = now + options_.blocked_retry_delay;
            snapshot_.reason =
                "mapper receipt did not certify the required loss recovery "
                "policy";
            return;
        }
        const auto graph = paths_->Evidence();
        if (map_->mapping_synchronized && BaselineMatches(*map_, version) &&
            (!graph || !BaselineMatches(*graph, version) ||
             !graph->observation_cut.Covers(version.observation_cut)) &&
            !planning_)
            QueuePlanning(now, false, false, {}, true);
        if (!InputPrefixProcessed(version) || coverage_.PendingCount() ||
            snapshot_.live_frontiers || !map_->mapping_synchronized ||
            !BaselineMatches(*map_, version) ||
            !map_->observation_cut.Covers(version.observation_cut) || !graph ||
            graph->map_epoch != map_->map_epoch ||
            graph->geometry_revision != map_->geometry_revision ||
            !BaselineMatches(*graph, version) ||
            !graph->observation_cut.Covers(version.observation_cut))
            return;
        if (!observations_.AcknowledgeRecovery(*recovery_loss_,
                                               version.observation_cut))
            return;
        observation_baseline_ = version.observation_baseline;
        observation_recovery_generation_ =
            version.observation_recovery_generation;
        recovery_operation_.reset();
        recovery_reply_.reset();
        recovery_loss_.reset();
        completion_.Reset();
        completion_version_.reset();
        worker_reset_requested_.store(true);
        snapshot_.state = ExplorationState::kExploring;
        snapshot_.reason =
            "replacement observations and mapper/graph scope evidence "
            "established a new baseline";
    }

    void RevokeCompletion() {
        CancelRecoveryOperation();
        completion_.Reset();
        completion_version_.reset();
        if (!snapshot_.completion_confirmed &&
            snapshot_.result.exploration != ExplorationOutcome::kCompleted)
            return;
        CancelActivePhase(PhaseStatus::kInvalidated,
                          "new work revoked the finish outcome");
        CancelPlanning();
        CancelNavigation();
        CancelMapOperation();
        snapshot_.completion_confirmed = false;
        snapshot_.result = {};
        finishing_ = false;
        ReleaseGoal();
        if (snapshot_.state == ExplorationState::kPaused)
            paused_state_ = ExplorationState::kExploring;
        else
            snapshot_.state = ExplorationState::kExploring;
    }

    void ApplyMap(MapReadView next) {
        if (!next.grid || !next.map_epoch || !std::isfinite(next.source_stamp))
            return;
        if (!next.startup_support && startup_observation_confirmed_ && map_ &&
            map_->map_epoch == next.map_epoch && std::isfinite(pose_.x) &&
            std::isfinite(pose_.y)) {
            const Pose2d center = home_valid_ ? home_ : pose_;
            if (autonomy::navigation::Distance(pose_, center) <=
                options_.startup_observation.support_radius + 1e-9) {
                // The platform's startup closure is measured independently.
                // Preserve it in the session snapshot even if the map event
                // carrying it had the same geometry revision as the last map.
                next.startup_support =
                    autonomy::navigation::StartupSupportRegion{
                        center.x, center.y,
                        options_.startup_observation.support_radius, 0.0,
                        std::max(1.0,
                                 options_.frontier_observation_height + 1.0)};
            }
        }
        if (map_ && next.map_epoch == map_->map_epoch &&
            next.geometry_revision == map_->geometry_revision &&
            next.startup_support && next.source_stamp < map_->source_stamp) {
            // Startup support is metadata derived from the already accepted
            // stationary sweep, not newer geometry. Admit it without renewing
            // the map acquisition stamp or fabricating a geometric update.
            map_->startup_support = next.startup_support;
            return;
        }
        if (map_ && (next.map_epoch < map_->map_epoch ||
                     (next.map_epoch == map_->map_epoch &&
                      (next.geometry_revision < map_->geometry_revision ||
                       next.source_stamp < map_->source_stamp))))
            return;
        const bool changed = !map_ || map_->map_epoch != next.map_epoch ||
                             map_->geometry_revision != next.geometry_revision;
        const bool volume_changed =
            !map_ ||
            static_cast<bool>(map_->observed_volume) !=
                static_cast<bool>(next.observed_volume) ||
            (map_->observed_volume && next.observed_volume &&
             (map_->observed_volume.get() != next.observed_volume.get() ||
              map_->observed_volume->source_stamp !=
                  next.observed_volume->source_stamp ||
              map_->observed_volume->mapping_synchronized !=
                  next.observed_volume->mapping_synchronized ||
              !SameObservationCut(map_->observed_volume->observation_cut,
                                  next.observed_volume->observation_cut)));
        const bool alignment = coverage_.epoch() != next.map_epoch;
        const bool invalidated_observation_leg =
            changed && observation_leg_.has_value();
        if (invalidated_observation_leg) {
            CancelPlanning();
            CancelObservationRotation();
            CancelNavigation();
            observation_leg_.reset();
            observation_view_target_.reset();
            final_adapted_arrival_confirmed_ = false;
        }
        if (changed) {
            RevokeCompletion();
            return_cache_.reset();
            // Geometry updates arrive while a full-map selection scan is still
            // running. The scan owns an immutable MapReadView and the Adapt
            // phase deliberately revalidates against the latest same-epoch map,
            // so cancelling here created a livelock: every fresh selection was
            // discarded by the next observation. Only an alignment-epoch change
            // invalidates the in-flight selection contract.
            if (alignment)
                CancelActivePhase(PhaseStatus::kInvalidated,
                                  "map changed during the phase");
            if (alignment) {
                snapshot_.selection.reset();
                selected_version_.reset();
            }
        }
        if (alignment) {
            CancelPlanning();
            CancelNavigation();
            coverage_.Reset(next.map_epoch);
            dormant_requirements_.clear();
            observations_.Reset(next.map_epoch);
            keyposes_.Reset(next.map_epoch);
            packet_poses_.clear();
            {
                std::lock_guard<std::mutex> lock(events_mutex_);
                admission_epoch_ = next.map_epoch;
            }
            {
                std::lock_guard<std::mutex> lock(depth_mutex_);
                depth_history_.clear();
            }
            recovery_loss_.reset();
            observation_baseline_ = {};
            observation_recovery_generation_ = 0;
            observed_cut_ = {};
            valid_observation_cut_ = {};
            pose_history_.clear();
            view_evidence_.clear();
            view_started_.clear();
            health_ =
                perception::SensorHealthMonitor(options_.mode, options_.health);
            completion_.Reset();
            if (services_.reproject_pose) {
                auto transformed = services_.reproject_pose(home_, home_epoch_,
                                                            next.map_epoch);
                home_valid_ =
                    transformed && autonomy::navigation::IsFinite(*transformed);
                if (home_valid_)
                    home_ = *transformed;
            } else
                home_valid_ = false;
            home_epoch_ = next.map_epoch;
            ReleaseGoal();
        }
        map_ = std::move(next);
        if (!changed && !volume_changed)
            return;
        // Keep an in-flight selection scan alive across geometry-only updates;
        // its result is versioned and revalidated by the Adapt phase.
        const auto frontiers = FrontierExtractor{}.Extract(
            *map_, options_.frontier_observation_height,
            options_.allowed_region, options_.minimum_frontier_cluster_size,
            options_.allowed_polygon, options_.no_go_areas);
        std::set<CoverageId> live_frontiers;
        for (const auto& unit : frontiers)
            live_frontiers.insert(unit.id);
        const auto protects_active_goal = [&](const CoverageId& id) {
            if (!snapshot_.goal)
                return false;
            const auto& required =
                snapshot_.goal->intent.required_coverage_units;
            return std::find(required.begin(), required.end(), id) !=
                   required.end();
        };
        std::vector<CoverageId> retired_frontiers;
        for (const auto& item : coverage_.units()) {
            if (item.first.kind != CoverageKind::kFrontier ||
                live_frontiers.count(item.first))
                continue;
            if (protects_active_goal(item.first))
                dormant_requirements_.insert(item.first);
            else
                retired_frontiers.push_back(item.first);
        }
        for (const auto& id : retired_frontiers) {
            coverage_.Remove(id);
            dormant_requirements_.erase(id);
        }
        for (const auto& item : coverage_.units()) {
            const auto& unit = item.second;
            // A changed planar cell invalidates non-frontier geometric work;
            // frontier lifecycle is governed by the exact live ID set above.
            if (unit.id.kind != CoverageKind::kFrontier &&
                std::any_of(map_->dirty_regions.begin(),
                            map_->dirty_regions.end(), [&](const auto& bounds) {
                                return bounds.Contains(
                                    {unit.center.x, unit.center.y, 0});
                            }))
                coverage_.Invalidate(item.first);
        }
        for (auto unit : frontiers) {
            if (dormant_requirements_.erase(unit.id))
                coverage_.Invalidate(unit.id);
            coverage_.Upsert(std::move(unit));
        }
        // Occupied faces become requirements only when an adjacent voxel is
        // explicitly measured free. Omitted sparse voxels remain unknown and
        // can neither create nor retire a surface obligation.
        auto surfaces = SurfaceExtractor{}.Extract(
            *map_, options_.allowed_region, options_.allowed_polygon,
            options_.no_go_areas);
        std::set<CoverageId> live_surfaces;
        for (const auto& unit : surfaces)
            live_surfaces.insert(unit.id);
        const auto& volume = map_->observed_volume;
        if (volume && volume->voxels && volume->mapping_synchronized &&
            volume->map_epoch == map_->map_epoch &&
            volume->geometry_revision == map_->geometry_revision &&
            volume->frame_id == map_->frame_id &&
            std::isfinite(volume->source_stamp) && volume->source_stamp >= 0 &&
            volume->origin.array().isFinite().all() &&
            (volume->dimensions.array() > 0).all() &&
            std::isfinite(volume->resolution) && volume->resolution > 0) {
            using VoxelIndex = std::array<int, 3>;
            std::map<VoxelIndex, perception::ObservedVoxelState> measured;
            bool canonical_volume = true;
            for (const auto& voxel : *volume->voxels) {
                if ((voxel.index.array() < 0).any() ||
                    (voxel.index.array() >= volume->dimensions.array()).any() ||
                    (voxel.state != perception::ObservedVoxelState::kFree &&
                     voxel.state !=
                         perception::ObservedVoxelState::kOccupied) ||
                    !measured
                         .emplace(VoxelIndex{voxel.index.x(), voxel.index.y(),
                                             voxel.index.z()},
                                  voxel.state)
                         .second) {
                    canonical_volume = false;
                    break;
                }
            }
            if (!canonical_volume)
                measured.clear();
            std::vector<CoverageId> retired_surfaces;
            for (const auto& item : coverage_.units()) {
                const auto& unit = item.second;
                if (item.first.kind != CoverageKind::kSurface ||
                    live_surfaces.count(item.first))
                    continue;
                const Eigen::Vector3i normal{
                    static_cast<int>(std::llround(unit.normal.x)),
                    static_cast<int>(std::llround(unit.normal.y)),
                    static_cast<int>(std::llround(unit.normal.z))};
                const Eigen::Vector3d center{unit.center.x, unit.center.y,
                                             unit.center.z};
                if (normal.squaredNorm() != 1 ||
                    (normal.cast<double>() - Eigen::Vector3d{unit.normal.x,
                                                             unit.normal.y,
                                                             unit.normal.z})
                            .norm() > 1e-9)
                    continue;
                const Eigen::Vector3d source_coordinates =
                    (center - volume->origin) / volume->resolution -
                    Eigen::Vector3d::Constant(.5) - normal.cast<double>() * .5;
                if (!source_coordinates.array().isFinite().all())
                    continue;
                const Eigen::Vector3d rounded_source =
                    source_coordinates.array().round();
                if ((source_coordinates - rounded_source)
                            .array()
                            .abs()
                            .maxCoeff() > 1e-9 ||
                    (rounded_source.array() < 0).any() ||
                    (rounded_source.array() >=
                     volume->dimensions.cast<double>().array())
                        .any())
                    continue;
                const Eigen::Vector3i source = rounded_source.cast<int>();
                const Eigen::Vector3i adjacent = source + normal;
                if ((adjacent.array() < 0).any() ||
                    (adjacent.array() >= volume->dimensions.array()).any())
                    continue;
                const Eigen::Vector3d expected_center =
                    volume->origin +
                    (source.cast<double>() + Eigen::Vector3d::Constant(.5) +
                     normal.cast<double>() * .5) *
                        volume->resolution;
                const Eigen::Vector3d world_coordinates =
                    (center / volume->resolution).array().floor();
                int face_bin = -1;
                if (normal.x() == 1)
                    face_bin = 0;
                else if (normal.x() == -1)
                    face_bin = 1;
                else if (normal.y() == 1)
                    face_bin = 2;
                else if (normal.y() == -1)
                    face_bin = 3;
                else if (normal.z() == 1)
                    face_bin = 4;
                else if (normal.z() == -1)
                    face_bin = 5;
                if ((center - expected_center).norm() > 1e-9 ||
                    !world_coordinates.array().isFinite().all() ||
                    (world_coordinates.array() <
                     std::numeric_limits<int>::min())
                        .any() ||
                    (world_coordinates.array() >
                     std::numeric_limits<int>::max())
                        .any() ||
                    item.first.x != static_cast<int>(world_coordinates.x()) ||
                    item.first.y != static_cast<int>(world_coordinates.y()) ||
                    item.first.z != static_cast<int>(world_coordinates.z()) ||
                    item.first.direction != face_bin)
                    continue;
                const auto source_state =
                    measured.find({source.x(), source.y(), source.z()});
                const auto adjacent_state =
                    measured.find({adjacent.x(), adjacent.y(), adjacent.z()});
                if ((source_state != measured.end() &&
                     source_state->second ==
                         perception::ObservedVoxelState::kFree) ||
                    (adjacent_state != measured.end() &&
                     adjacent_state->second ==
                         perception::ObservedVoxelState::kOccupied)) {
                    if (protects_active_goal(item.first))
                        dormant_requirements_.insert(item.first);
                    else
                        retired_surfaces.push_back(item.first);
                }
            }
            for (const auto& id : retired_surfaces) {
                coverage_.Remove(id);
                dormant_requirements_.erase(id);
            }
        }
        for (auto unit : surfaces) {
            if (dormant_requirements_.erase(unit.id))
                coverage_.Invalidate(unit.id);
            coverage_.Upsert(std::move(unit));
        }
        snapshot_.live_frontiers = frontiers.size();
        completion_.Reset();
        if (snapshot_.goal && (!services_.navigation_still_valid ||
                               !services_.navigation_still_valid(
                                   *snapshot_.goal, pose_, *map_))) {
            CancelNavigation();
            ReleaseGoal();
            if (active_phase_ &&
                active_phase_->phase == ExplorationPhase::kObserve)
                CancelActivePhase(PhaseStatus::kInvalidated,
                                  "observation target failed map revalidation");
        }
        if (invalidated_observation_leg && snapshot_.goal && !finishing_ &&
            snapshot_.state != ExplorationState::kPaused &&
            (options_.driver == ExplorationDriver::kAutomatic ||
             (active_phase_ &&
              active_phase_->phase == ExplorationPhase::kObserve))) {
            QueueObservationContinuation(
                last_now_, active_phase_ && active_phase_->phase ==
                                                ExplorationPhase::kObserve
                               ? std::optional<ExplorationPhase>(
                                     ExplorationPhase::kObserve)
                               : std::nullopt);
        }
        if (!finishing_ && snapshot_.state != ExplorationState::kPaused &&
            !snapshot_.goal)
            snapshot_.state = ExplorationState::kExploring;
    }

    void ApplyPose(const PoseEvent& event) {
        if (!autonomy::navigation::IsFinite(event.pose) ||
            !std::isfinite(event.stamp) || event.epoch < pose_epoch_ ||
            (map_ && event.epoch < map_->map_epoch) ||
            event.stamp < pose_stamp_)
            return;
        const bool new_acquisition =
            event.epoch != pose_epoch_ || event.stamp > pose_stamp_;
        if (event.epoch == pose_epoch_ && event.stamp > pose_stamp_)
            snapshot_.traveled_distance +=
                autonomy::navigation::Distance(pose_, event.pose);
        if (event.epoch != pose_epoch_)
            pose_history_.clear();
        if (new_acquisition &&
            pose_sequence_ < std::numeric_limits<std::uint64_t>::max())
            ++pose_sequence_;
        pose_ = event.pose;
        pose_epoch_ = event.epoch;
        pose_stamp_ = event.stamp;
        localization_ = event.localization & perception::Mask(options_.mode);
        if (!localization_) {
            AINFO_EVERY(20)
                << "EXPLORATION_POSE_MASK event="
                << static_cast<unsigned>(event.localization)
                << " mode=" << static_cast<int>(options_.mode) << " mode_mask="
                << static_cast<unsigned>(perception::Mask(options_.mode))
                << " stamp=" << event.stamp;
        }
        pose_history_.push_back(event);
        if (pose_history_.size() > 512)
            pose_history_.pop_front();
    }

    /** @brief Convert attested base rotation to the ground robot's actual yaw.
     */
    static PoseEvent PacketPose(const perception::AcquisitionLocalization& pose,
                                double stamp) {
        return {{pose.world_from_base.translation.x,
                 pose.world_from_base.translation.y,
                 std::atan2(pose.world_from_base.rotation[3],
                            pose.world_from_base.rotation[0])},
                stamp,
                pose.map_epoch,
                pose.support};
    }

    /** @brief Apply attested acquisition poses without moving current time
     * backward.
     * @details An exact-time sample refreshes the current pose and localization
     * support, including the untrusted Start seed. Older samples only update
     * historical evidence. Neither case contributes fictitious traveled
     * distance.
     */
    void RecordPacketPose(const PoseEvent& event) {
        if (event.epoch != pose_epoch_ || event.stamp > pose_stamp_) {
            ApplyPose(event);
            return;
        }
        if (event.stamp == pose_stamp_) {
            pose_ = event.pose;
            localization_ =
                event.localization & perception::Mask(options_.mode);
        }
        auto position = std::lower_bound(
            pose_history_.begin(), pose_history_.end(), event.stamp,
            [](const PoseEvent& pose, double stamp) {
                return pose.stamp < stamp;
            });
        if (position != pose_history_.end() && position->stamp == event.stamp) {
            // An exact acquisition sample replaces a prior seed/interpolation.
            *position = event;
        } else
            pose_history_.insert(position, event);
        if (pose_history_.size() > 512)
            pose_history_.pop_front();
    }

    /** @brief Apply correlated input only after actual-time observation
     * admission. */
    void ApplyPacket(PacketEvent event, double now) {
        auto& packet = event.packet;
        const auto stamp = packet.observation.stamp;
        const auto sensor = packet.observation.sensor;
        const auto sequence = packet.observation.sequence;
        now = std::max(now, event.admitted_at);
        if (!packet.Valid(now) || !map_ ||
            packet.observation.map_epoch != map_->map_epoch ||
            packet.observation.frame_id != map_->frame_id) {
            observations_.Reject(sensor, sequence);
            return;
        }
        const double valid_until = packet.observation.valid_until;
        PacketPoses poses{packet.observation.map_epoch,
                          packet.localization.calibration_revision,
                          {}};
        poses.samples.reserve(packet.acquisition_poses.size() + 1);
        for (const auto& pose : packet.acquisition_poses) {
            const auto sample = PacketPose(pose.localization, pose.stamp);
            if (poses.samples.empty() ||
                poses.samples.back().stamp != sample.stamp)
                poses.samples.push_back(sample);
        }
        const auto end = PacketPose(packet.localization, stamp);
        if (poses.samples.empty() || poses.samples.back().stamp != stamp)
            poses.samples.push_back(end);
        if (!observations_.Push(std::move(packet.observation), now))
            return;
        auto retained = packet_poses_.insert_or_assign(
            ObservationIdentity{sensor, sequence}, std::move(poses));
        // Packet::Valid already requires ordered exact-time proofs. Rolling
        // history remains bounded; observation qualification uses retained
        // poses.
        for (const auto& pose : retained.first->second.samples)
            RecordPacketPose(pose);
        if (packet.depth_view) {
            auto view = std::make_shared<const perception::LocalizedDepthView>(
                perception::LocalizedDepthView{std::move(*packet.depth_view),
                                               packet.localization,
                                               valid_until});
            std::lock_guard<std::mutex> lock(depth_mutex_);
            depth_history_.push_back(std::move(view));
            if (depth_history_.size() > 4)
                depth_history_.pop_front();
        }
    }

    void Command(SessionCommand command, double now) {
        if (snapshot_.state == ExplorationState::kFinished)
            return;
        if (command == SessionCommand::kCancel) {
            CancelRecoveryOperation();
            CancelStartupObservation();
            CancelActivePhase(PhaseStatus::kCanceled, "session canceled");
            CancelPlanning();
            CancelNavigation();
            CancelMapOperation();
            ReleaseGoal();
            snapshot_.result.exploration = ExplorationOutcome::kCanceled;
            finishing_ = true;
            if (options_.save_on_cancel)
                BeginRefresh(now);
            else
                snapshot_.state = ExplorationState::kFinished;
        } else if (command == SessionCommand::kPause) {
            if (snapshot_.state == ExplorationState::kPaused)
                return;
            CancelRecoveryOperation();
            recovery_loss_.reset();
            CancelStartupObservation();
            CancelPlanning();
            CancelNavigation();
            paused_state_ = snapshot_.state;
            paused_at_ = now;
            snapshot_.state = ExplorationState::kPaused;
        } else if (command == SessionCommand::kResume && paused_at_) {
            const double paused_duration = now - *paused_at_;
            paused_time_ += paused_duration;
            if (active_phase_) {
                active_phase_->version.valid_until += paused_duration;
                for (auto& result : phase_history_)
                    if (result.ticket.SameRequest(*active_phase_))
                        result.ticket = *active_phase_;
                resume_phase_ = true;
            }
            paused_at_.reset();
            wait_started_.reset();
            snapshot_.state =
                paused_state_.value_or(ExplorationState::kExploring);
            paused_state_.reset();
            if (snapshot_.state == ExplorationState::kObserving &&
                current_view_ < view_evidence_.size()) {
                // Paused time and queued rays acquired during the pause cannot
                // satisfy the unfinished view's observation requirement.
                arrived_at_ = now;
                view_started_[current_view_].reset();
                view_evidence_[current_view_].clear();
            }
            if (!active_phase_ &&
                snapshot_.state != ExplorationState::kSaving &&
                snapshot_.state != ExplorationState::kObserving) {
                ReleaseGoal();
                snapshot_.state = finishing_ ? ExplorationState::kReturning
                                             : ExplorationState::kExploring;
            }
        }
    }

    std::optional<PoseEvent> PoseAt(double stamp) const {
        if (pose_history_.empty())
            return {};
        const auto upper =
            std::upper_bound(pose_history_.begin(), pose_history_.end(), stamp,
                             [](double time, const PoseEvent& event) {
                                 return time < event.stamp;
                             });
        if (upper == pose_history_.begin())
            return {};
        auto result = *std::prev(upper);
        if (result.epoch != pose_epoch_ ||
            stamp - result.stamp > options_.pose_max_age)
            return {};
        if (upper != pose_history_.end() && upper->epoch == result.epoch &&
            upper->stamp > result.stamp &&
            upper->stamp - result.stamp <= options_.pose_max_age) {
            const double fraction =
                (stamp - result.stamp) / (upper->stamp - result.stamp);
            result.pose.x += fraction * (upper->pose.x - result.pose.x);
            result.pose.y += fraction * (upper->pose.y - result.pose.y);
            result.pose.yaw = autonomy::navigation::NormalizeAngle(
                result.pose.yaw +
                fraction * autonomy::navigation::NormalizeAngle(
                               upper->pose.yaw - result.pose.yaw));
            result.localization &= upper->localization;
        }
        result.stamp = stamp;
        return result;
    }

    /** @brief Resolve a ray only from its packet's actual pose when one exists.
     * @param observation Source/ordinal/epoch identity of the consumed input.
     * @param stamp Exact acquisition time used to transform this ray.
     * @return Matching authoritative sample, or the legacy pose-history lookup
     * for observations submitted without an associated native input packet.
     * A packet sample that is missing or belongs to another epoch never falls
     * back to interpolation or another observation's same-time localization.
     */
    std::optional<PoseEvent> ObservationPoseAt(
        const perception::SensorObservation& observation, double stamp) const {
        const auto packet =
            packet_poses_.find({observation.sensor, observation.sequence});
        if (packet == packet_poses_.end())
            return PoseAt(stamp);
        if (packet->second.epoch != observation.map_epoch)
            return {};
        const auto& samples = packet->second.samples;
        const auto found =
            std::lower_bound(samples.begin(), samples.end(), stamp,
                             [](const PoseEvent& pose, double time) {
                                 return pose.stamp < time;
                             });
        if (found == samples.end() || found->stamp != stamp ||
            found->epoch != observation.map_epoch)
            return {};
        return *found;
    }

    /** @brief Release packet poses only after the matching FIFO items are
     * examined. Expired observations also advance processed_cut(), so rejection
     * frees their evidence. Paused or unexamined suffixes retain proofs without
     * extending the observation's expiry. Start and alignment reset clear all
     * retained entries.
     */
    void PrunePacketPoses() {
        const auto examined = observations_.processed_cut();
        for (auto item = packet_poses_.begin(); item != packet_poses_.end();) {
            const auto sensor = item->first.first;
            const auto index =
                sensor == perception::SensorKind::kLidar ? 0U : 1U;
            if (item->second.epoch != coverage_.epoch() ||
                ((examined.present & perception::Mask(sensor)) &&
                 item->first.second <= examined.sequence[index]))
                item = packet_poses_.erase(item);
            else
                ++item;
        }
    }

    void RecordObservation(const perception::SensorObservation& observation,
                           double now) {
        if (snapshot_.state != ExplorationState::kObserving ||
            !snapshot_.goal ||
            observation.valid_fraction <
                options_.health.minimum_valid_fraction ||
            observation.scan_completeness <
                options_.health.minimum_completeness)
            return;
        for (std::size_t index = 0;
             index < snapshot_.goal->intent.required_views.size(); ++index) {
            auto filtered = observation;
            filtered.rays.clear();
            for (const auto& ray : observation.rays) {
                if (ray.stamp < arrived_at_)
                    continue;
                const auto acquisition =
                    ObservationPoseAt(observation, ray.stamp);
                if (!acquisition ||
                    !(acquisition->localization &
                      perception::Mask(observation.sensor)) ||
                    autonomy::navigation::Distance(
                        acquisition->pose, snapshot_.goal->execution_pose) >
                        options_.position_tolerance ||
                    std::abs(autonomy::navigation::NormalizeAngle(
                        acquisition->pose.yaw -
                        snapshot_.goal->intent.required_views[index])) >
                        options_.yaw_tolerance)
                    continue;
                filtered.rays.push_back(ray);
                if (!view_started_[index])
                    view_started_[index] = ray.stamp;
            }
            if (filtered.rays.empty())
                continue;
            auto scratch = coverage_;
            const auto measured =
                CoverageEvaluator{}.Apply(filtered, now, scratch);
            for (const auto& evidence : measured.evidence) {
                const auto sensor =
                    evidence.sensor == perception::SensorKind::kLidar ? 0U : 1U;
                auto& quality = view_evidence_[index][evidence.id][sensor];
                quality = std::max(quality, evidence.quality);
            }
        }
    }

    bool ViewObserved(std::size_t index, double now) const {
        if (index >= view_evidence_.size() || !view_started_[index] ||
            now - *view_started_[index] < options_.minimum_observation_time)
            return false;
        for (const auto& id : snapshot_.goal->intent.required_coverage_units) {
            const auto unit = coverage_.units().find(id);
            const auto measured = view_evidence_[index].find(id);
            if (unit == coverage_.units().end() ||
                measured == view_evidence_[index].end())
                return false;
            if (unit->second.required_sensors) {
                for (unsigned sensor = 0; sensor < 2; ++sensor)
                    if ((unit->second.required_sensors & (1U << sensor)) &&
                        measured->second[sensor] <
                            unit->second.required_quality)
                        return false;
            } else if (std::max(measured->second[0], measured->second[1]) <
                       unit->second.required_quality)
                return false;
        }
        return true;
    }

    bool GoalObserved(double now) const {
        if (!snapshot_.goal ||
            now - arrived_at_ < options_.minimum_observation_time)
            return false;
        if (snapshot_.goal->intent.required_coverage_units.empty() ||
            view_evidence_.empty())
            return false;
        for (std::size_t index = 0; index < view_evidence_.size(); ++index)
            if (!ViewObserved(index, now))
                return false;
        return true;
    }

    bool At(const Pose2d& goal) const {
        return autonomy::navigation::Distance(pose_, goal) <=
                   options_.position_tolerance &&
               std::abs(autonomy::navigation::NormalizeAngle(
                   pose_.yaw - goal.yaw)) <= options_.yaw_tolerance;
    }

    /** @brief Check TARE-compatible measured return arrival without yaw. */
    bool AtHome() const {
        return autonomy::navigation::Distance(pose_, home_) <=
               options_.return_position_tolerance;
    }

    /** @brief Position-only arrival used when viewpoint orientation is applied
     * separately. */
    bool AtPosition(const Pose2d& goal) const {
        return autonomy::navigation::Distance(pose_, goal) <=
               options_.position_tolerance;
    }

    /** @brief Verify the current required view from immutable goal intent. */
    bool AtCurrentRequiredView() const {
        if (!snapshot_.goal ||
            current_view_ >= snapshot_.goal->intent.required_views.size())
            return false;
        auto view_pose = snapshot_.goal->execution_pose;
        view_pose.yaw = snapshot_.goal->intent.required_views[current_view_];
        return At(view_pose);
    }

    /** @brief Whether the existing native `/spin` service can rotate at a
     * viewpoint. */
    bool HasObservationRotation() const {
        return options_.startup_observation.enabled &&
               services_.request_startup_observation &&
               services_.poll_startup_observation &&
               services_.cancel_startup_observation;
    }

    /** @brief Cancel only the viewpoint rotation, preserving the observation
     * phase. */
    void CancelObservationRotation() {
        if (observation_rotation_ && services_.cancel_startup_observation)
            services_.cancel_startup_observation(*observation_rotation_);
        observation_rotation_.reset();
        pending_navigation_pose_.reset();
        rotation_for_navigation_ = false;
    }

    /** @brief Cancel only the loss-reacquisition sweep; keep sensor waiting
     * alive. */
    void CancelRecoveryRotation() {
        if (recovery_rotation_ && services_.cancel_startup_observation)
            services_.cancel_startup_observation(*recovery_rotation_);
        recovery_rotation_.reset();
    }

    /** @brief Rotate in place with one explicit post-rotation purpose. */
    bool StartYawRotation(double target_yaw, double now, bool for_navigation) {
        if (!HasObservationRotation() || !snapshot_.goal)
            return false;
        CancelRecoveryRotation();
        CancelObservationRotation();
        const double delta =
            autonomy::navigation::NormalizeAngle(target_yaw - pose_.yaw);
        if (std::abs(delta) <= options_.yaw_tolerance) {
            if (!for_navigation) {
                arrived_at_ = now;
                snapshot_.state = ExplorationState::kObserving;
            }
            return true;
        }
        const double allowance = options_.startup_observation.time_allowance +
                                 options_.startup_observation.deadline_margin;
        auto version = Version(now, std::max(1.0, allowance));
        if (!services_.request_startup_observation(version, delta, allowance))
            return false;
        observation_rotation_ = version;
        rotation_for_navigation_ = for_navigation;
        snapshot_.state = for_navigation ? ExplorationState::kExploring
                                         : ExplorationState::kObserving;
        AINFO << "EXPLORATION_YAW_ROTATION submit yaw=" << target_yaw
              << " delta=" << delta
              << " goal=" << snapshot_.goal->intent.goal_id;
        return true;
    }

    /** @brief Rotate to the required camera view after positional arrival. */
    bool StartObservationRotation(double target_yaw, double now) {
        if (!snapshot_.goal || !final_adapted_arrival_confirmed_)
            return false;
        observation_view_target_ = snapshot_.goal->execution_pose;
        observation_view_target_->yaw = target_yaw;
        return StartYawRotation(target_yaw, now, false);
    }

    /** @brief Begin measured viewing only after exact final-pose arrival. */
    void BeginObservationAtFinal(double now) {
        if (!snapshot_.goal || !final_adapted_arrival_confirmed_ ||
            !At(snapshot_.goal->execution_pose)) {
            Retry(now, "final adapted pose arrival was not measured");
            return;
        }
        const auto& views = snapshot_.goal->intent.required_views;
        if (current_view_ >= views.size()) {
            Retry(now, "observation goal has no remaining required view");
            return;
        }
        observation_view_target_ = snapshot_.goal->execution_pose;
        observation_view_target_->yaw = views[current_view_];
        if (At(*observation_view_target_)) {
            arrived_at_ = now;
            snapshot_.state = ExplorationState::kObserving;
        } else if (HasObservationRotation()) {
            if (!StartObservationRotation(views[current_view_], now))
                Retry(now, "viewpoint rotation could not start");
        } else {
            SubmitNavigation(*observation_view_target_, now);
        }
    }

    /** @brief Reacquire measurements after a loss without requiring a coverage
     * goal. */
    bool StartRecoveryRotation(double now) {
        if (!HasObservationRotation() || recovery_rotation_)
            return false;
        CancelObservationRotation();
        const double allowance = options_.startup_observation.time_allowance +
                                 options_.startup_observation.deadline_margin;
        auto version = Version(now, std::max(1.0, allowance));
        if (!services_.request_startup_observation(
                version, 2 * autonomy::navigation::kPi, allowance))
            return false;
        recovery_rotation_ = version;
        snapshot_.state = ExplorationState::kObserving;
        AINFO << "EXPLORATION_RECOVERY_ROTATION submit allowance=" << allowance
              << " mask="
              << static_cast<unsigned>(
                     observations_.LossSnapshot().pending_sources);
        return true;
    }

    /** @brief Consume one measured in-place viewpoint rotation. */
    void PollObservationRotation(double now) {
        if (!observation_rotation_)
            return;
        const auto reply = services_.poll_startup_observation();
        if (reply && reply->version.SameRequest(*observation_rotation_)) {
            observation_rotation_.reset();
            if (!reply->success) {
                Retry(now, reply->reason.empty() ? "viewpoint rotation failed"
                                                 : reply->reason);
                return;
            }
            if (rotation_for_navigation_) {
                rotation_for_navigation_ = false;
                if (!pending_navigation_pose_) {
                    Retry(now, "navigation rotation has no pending target");
                    return;
                }
                const auto pose = *pending_navigation_pose_;
                pending_navigation_pose_.reset();
                DispatchNavigation(pose, now);
                return;
            }
            if (!snapshot_.goal || !final_adapted_arrival_confirmed_ ||
                !observation_view_target_ || !AtCurrentRequiredView()) {
                Retry(now, "required view orientation was not measured");
                return;
            }
            arrived_at_ = now;
            snapshot_.state = ExplorationState::kObserving;
            return;
        }
        if (now > observation_rotation_->valid_until) {
            services_.cancel_startup_observation(*observation_rotation_);
            observation_rotation_.reset();
            Retry(now, "viewpoint rotation deadline exceeded");
        }
    }

    /** @brief Consume the bounded loss-reacquisition sweep. */
    void PollRecoveryRotation(double now) {
        if (!recovery_rotation_)
            return;
        const auto reply = services_.poll_startup_observation();
        if (reply && reply->version.SameRequest(*recovery_rotation_)) {
            recovery_rotation_.reset();
            snapshot_.state = ExplorationState::kExploring;
            selection_after_ = now + options_.blocked_retry_delay;
            snapshot_.reason = reply->success
                                   ? "observation recovery rotation completed"
                                   : "observation recovery rotation failed";
            return;
        }
        if (now > recovery_rotation_->valid_until) {
            services_.cancel_startup_observation(*recovery_rotation_);
            recovery_rotation_.reset();
            snapshot_.state = ExplorationState::kExploring;
            selection_after_ = now + options_.blocked_retry_delay;
            snapshot_.reason =
                "observation recovery rotation deadline exceeded";
        }
    }

    void SubmitNavigation(const Pose2d& pose, double now) {
        CancelObservationRotation();
        Pose2d navigation_pose = pose;
        if (snapshot_.goal && HasObservationRotation()) {
            const double dx = pose.x - pose_.x;
            const double dy = pose.y - pose_.y;
            if (std::hypot(dx, dy) > options_.position_tolerance) {
                navigation_pose.yaw = std::atan2(dy, dx);
                const double heading_error =
                    autonomy::navigation::NormalizeAngle(navigation_pose.yaw -
                                                         pose_.yaw);
                if (std::abs(heading_error) > options_.yaw_tolerance) {
                    if (!StartYawRotation(navigation_pose.yaw, now, true))
                        Retry(now, "approach rotation could not start");
                    else
                        pending_navigation_pose_ = pose;
                    return;
                }
            }
        }
        DispatchNavigation(pose, now);
    }

    /** @brief Dispatch one already oriented navigation target. */
    void DispatchNavigation(const Pose2d& navigation_pose, double now) {
        navigation_ = Version(now, options_.navigation_timeout);
        AINFO << "EXPLORATION_NAVIGATION submit x=" << navigation_pose.x
              << " y=" << navigation_pose.y << " yaw=" << navigation_pose.yaw
              << " goal="
              << (snapshot_.goal ? snapshot_.goal->intent.goal_id : 0);
        if (!services_.submit_navigation(navigation_pose, *navigation_)) {
            navigation_.reset();
            if (finishing_) {
                snapshot_.result.returned = ReturnOutcome::kReturnFailed;
                AfterReturn(now);
            } else
                Retry(now, "navigation submission rejected");
        } else {
            // A fresh admitted route starts a new recovery episode. The global
            // time/distance budgets, not failures of superseded viewpoints,
            // bound exploration across changing maps.
            recovery_count_ = 0;
        }
    }

    void Retry(double now, const std::string& reason) {
        FinishPhase(PhaseStatus::kRetryable, reason);
        CancelObservationRotation();
        CancelNavigation();
        if (snapshot_.goal)
            strategy_->MarkFailed(snapshot_.goal->intent, now);
        ReleaseGoal();
        observation_leg_.reset();
        observation_view_target_.reset();
        final_adapted_arrival_confirmed_ = false;
        snapshot_.reason = reason;
        if (++recovery_count_ > options_.maximum_recoveries) {
            BeginFinish(ExplorationOutcome::kIncompleteObservation, now);
            return;
        }
        selection_after_ = now + options_.blocked_retry_delay;
        snapshot_.state = ExplorationState::kRecovering;
    }

    void BeginFinish(ExplorationOutcome outcome, double now) {
        CancelRecoveryRotation();
        AINFO << "EXPLORATION_FINISH outcome=" << static_cast<int>(outcome)
              << " elapsed=" << now - started_ - paused_time_
              << " budget=" << options_.time_budget
              << " traveled=" << snapshot_.traveled_distance;
        CancelRecoveryOperation();
        CancelActivePhase(PhaseStatus::kRetryable,
                          "exploration reached a finish outcome");
        CancelPlanning();
        CancelNavigation();
        snapshot_.result.exploration = outcome;
        finishing_ = true;
        ReleaseGoal();
        if (BtDriven()) {
            snapshot_.state = ExplorationState::kFinishing;
            return;
        }
        if (options_.require_return) {
            if (execution_fault_ || !home_valid_ || !PoseReady(now) ||
                !snapshot_.sensors.ready) {
                snapshot_.result.returned = ReturnOutcome::kReturnFailed;
                BeginRefresh(now);
            } else if (At(home_)) {
                snapshot_.result.returned = ReturnOutcome::kReturned;
                BeginRefresh(now);
            } else
                QueuePlanning(now, true);
        } else
            BeginRefresh(now);
    }

    void BeginRefresh(double now) {
        CancelRecoveryOperation();
        CancelPlanning();
        if (!options_.require_save && !options_.save_on_cancel) {
            snapshot_.state = ExplorationState::kFinished;
            return;
        }
        map_operation_ = Version(now, options_.map_operation_timeout);
        BindCompletionPrefix(*map_operation_);
        saving_map_ = false;
        snapshot_.state = ExplorationState::kSaving;
        if (!services_.request_map_refresh || !services_.poll_map_refresh ||
            !services_.request_map_refresh(*map_operation_)) {
            snapshot_.result.saved = MapSaveOutcome::kSaveFailed;
            snapshot_.state = ExplorationState::kFinished;
            map_operation_.reset();
            FinishPhase(PhaseStatus::kFailed, "map refresh submission failed");
        }
    }

    void PollMapOperation(double now) {
        if (!map_operation_)
            return;
        const auto reply = saving_map_ ? (services_.poll_map_save
                                              ? services_.poll_map_save()
                                              : std::optional<MappingReply>{})
                                       : services_.poll_map_refresh();
        if (reply && reply->version.SameRequest(*map_operation_) &&
            now <= map_operation_->valid_until) {
            if (!reply->success) {
                snapshot_.result.saved = MapSaveOutcome::kSaveFailed;
                snapshot_.state = ExplorationState::kFinished;
                map_operation_.reset();
                FinishPhase(PhaseStatus::kFailed, "mapping operation failed");
            } else if (saving_map_) {
                if (snapshot_.result.exploration ==
                        ExplorationOutcome::kCompleted &&
                    (!FinalCompletionInputsValid(now) ||
                     !ReceiptCovers(*reply, *map_operation_) ||
                     map_operation_->map_epoch != map_->map_epoch ||
                     map_operation_->geometry_revision !=
                         map_->geometry_revision ||
                     map_operation_->sensor_epoch !=
                         snapshot_.sensors.sensor_epoch)) {
                    snapshot_.result.exploration =
                        ExplorationOutcome::kIncompleteObservation;
                    snapshot_.completion_confirmed = false;
                    snapshot_.reason =
                        "saved map does not certify the latest healthy "
                        "exploration evidence";
                }
                snapshot_.result.saved = MapSaveOutcome::kSaved;
                snapshot_.state = ExplorationState::kFinished;
                map_operation_.reset();
                FinishPhase(PhaseStatus::kSucceeded, "map save acknowledged");
            } else if (snapshot_.result.exploration ==
                           ExplorationOutcome::kCompleted &&
                       (coverage_.PendingCount() || snapshot_.live_frontiers)) {
                map_operation_.reset();
                finishing_ = false;
                completion_.Reset();
                snapshot_.completion_confirmed = false;
                snapshot_.state = ExplorationState::kExploring;
                FinishPhase(PhaseStatus::kInvalidated,
                            "new work appeared during refresh");
            } else {
                if (snapshot_.result.exploration ==
                        ExplorationOutcome::kCompleted &&
                    (!FinalCompletionInputsValid(now) ||
                     !ReceiptCovers(*reply, *map_operation_))) {
                    snapshot_.result.exploration =
                        ExplorationOutcome::kIncompleteObservation;
                    snapshot_.completion_confirmed = false;
                    snapshot_.reason =
                        "final map refresh has no verified observation fusion "
                        "barrier";
                }
                map_operation_ = Version(now, options_.map_operation_timeout);
                BindCompletionPrefix(*map_operation_);
                saving_map_ = true;
                if (!services_.save_map || !services_.poll_map_save ||
                    !services_.save_map(options_.map_name, *map_operation_)) {
                    snapshot_.result.saved = MapSaveOutcome::kSaveFailed;
                    snapshot_.state = ExplorationState::kFinished;
                    map_operation_.reset();
                    FinishPhase(PhaseStatus::kFailed,
                                "map save submission failed");
                }
            }
        } else if (now > map_operation_->valid_until) {
            CancelMapOperation();
            snapshot_.result.saved = MapSaveOutcome::kSaveFailed;
            snapshot_.state = ExplorationState::kFinished;
            map_operation_.reset();
            FinishPhase(PhaseStatus::kFailed,
                        "mapping operation deadline exceeded");
        }
    }

    void ConsumePlan(double now) {
        if (planning_ &&
            (!InputPrefixProcessed(*planning_) || PendingInvalidation()))
            return;
        std::deque<Planned> results;
        {
            std::lock_guard<std::mutex> lock(work_mutex_);
            results.swap(planned_results_);
        }
        for (auto& result : results) {
            if (!planning_ || !result.version.SameRequest(*planning_))
                continue;
            if (!map_ || now > planning_->valid_until ||
                result.version.map_epoch != map_->map_epoch ||
                result.version.sensor_epoch != snapshot_.sensors.sensor_epoch ||
                ((result.continuation || result.adapted) &&
                 result.version.geometry_revision != map_->geometry_revision) ||
                (!result.returning && !result.estimate_only &&
                 result.version.requirements_revision !=
                     coverage_.requirements_revision())) {
                AINFO_EVERY(5)
                    << "EXPLORATION_PLAN_INVALID map_epoch="
                    << (map_ ? map_->map_epoch : 0)
                    << " result_epoch=" << result.version.map_epoch
                    << " sensor=" << result.version.sensor_epoch
                    << " current_sensor=" << snapshot_.sensors.sensor_epoch
                    << " phase="
                    << (result.phase ? static_cast<int>(*result.phase) : -1);
                CancelPlanning();
                FinishPhase(PhaseStatus::kInvalidated,
                            "planning evidence or deadline changed");
                continue;
            }
            planning_.reset();
            if (result.evidence_only) {
                if (!result.error.empty()) {
                    CancelRecoveryOperation();
                    loss_recovery_after_ = now + options_.blocked_retry_delay;
                    snapshot_.reason =
                        "route owner failed to update recovery evidence";
                }
                continue;
            }
            if (!result.error.empty()) {
                if (result.estimate_only) {
                    // A return-cost estimate is advisory while an admitted
                    // observation goal is executing. Missing this background
                    // estimate cannot cancel the safe command in flight.
                    return_cache_.reset();
                    snapshot_.reason =
                        "return estimate unavailable: " + result.error;
                    continue;
                }
                if (result.returning || finishing_) {
                    snapshot_.result.returned = ReturnOutcome::kReturnFailed;
                    AfterReturn(now);
                } else
                    Retry(now, result.error);
                continue;
            }
            if (options_.require_return) {
                if (result.return_route_valid)
                    return_cache_ =
                        ReturnCache{result.version, result.return_length,
                                    result.traveled_distance};
                else
                    return_cache_.reset();
            }
            if (result.returning) {
                if (result.route_valid)
                    SubmitNavigation(home_, now);
                else {
                    snapshot_.result.returned = ReturnOutcome::kReturnFailed;
                    AfterReturn(now);
                }
                continue;
            }
            if (result.estimate_only) {
                // A return-route estimate is advisory until the finish policy
                // actually requests the return phase. A map revision can make
                // this snapshot stale while the current safe observation goal
                // is still executing; retry the estimate without revoking it.
                if (!result.return_route_valid) {
                    return_cache_.reset();
                    snapshot_.reason =
                        "return route estimate awaits current map evidence";
                }
                continue;
            }
            if (result.continuation) {
                if (!snapshot_.goal || !result.observation_route ||
                    !result.observation_waypoint) {
                    Retry(now,
                          "observation continuation lost its admitted goal");
                    continue;
                }
                observation_leg_ = ActiveObservationLeg{
                    std::move(*result.observation_route),
                    *result.observation_waypoint, result.version};
                final_adapted_arrival_confirmed_ = false;
                snapshot_.state = ExplorationState::kExploring;
                SubmitNavigation(observation_leg_->waypoint.pose, now);
                continue;
            }
            if (result.phase == ExplorationPhase::kSelect &&
                result.decision.goal) {
                AINFO << "EXPLORATION_PLAN_SELECTED goal="
                      << result.decision.goal->goal_id << " units="
                      << result.decision.goal->required_coverage_units.size()
                      << " candidates="
                      << result.decision.local_route.views.size()
                      << " pending=" << coverage_.PendingCount();
                snapshot_.selection = std::move(result.decision.goal);
                selected_version_ = result.version;
                snapshot_.state = ExplorationState::kExploring;
                FinishPhase(PhaseStatus::kSucceeded,
                            "observation intent selected");
                continue;
            }
            if (result.adapted) {
                if (autonomy::navigation::Distance(result.origin, pose_) >
                    options_.position_tolerance) {
                    FinishPhase(PhaseStatus::kInvalidated,
                                "robot moved during adaptation");
                    continue;
                }
                if (options_.require_return && !result.return_route_valid) {
                    // A return route can be temporarily unavailable while the
                    // observation viewpoint is still safe and useful. Keep
                    // exploring; the explicit return phase re-plans against
                    // the final map and reports failure if it remains blocked.
                    return_cache_.reset();
                    snapshot_.reason =
                        "return route awaits updated map evidence";
                }
                double turn = std::abs(autonomy::navigation::NormalizeAngle(
                    result.adapted->intent.required_views.front() - pose_.yaw));
                const auto& views = result.adapted->intent.required_views;
                const auto view_count = views.size();
                for (std::size_t i = 1; i < views.size(); ++i)
                    turn += std::abs(autonomy::navigation::NormalizeAngle(
                        views[i] - views[i - 1]));
                const double next_distance = result.adapted->route_length;
                const double return_distance =
                    options_.require_return ? result.return_from_goal : 0;
                const double estimated_time =
                    (next_distance + return_distance) / options_.nominal_speed +
                    turn / options_.nominal_angular_speed +
                    views.size() * options_.minimum_observation_time +
                    (options_.require_return ? options_.return_margin : 0);
                if (now - started_ - paused_time_ + estimated_time >=
                        options_.time_budget ||
                    snapshot_.traveled_distance + next_distance +
                            return_distance >=
                        options_.distance_budget) {
                    BeginFinish(ExplorationOutcome::kPartialBudget, now);
                    continue;
                }
                if (!result.observation_route || !result.observation_waypoint) {
                    Retry(now, "adapted goal has no executable route waypoint");
                    continue;
                }
                snapshot_.goal = std::move(result.adapted);
                prepared_version_ = result.version;
                observation_leg_ = ActiveObservationLeg{
                    std::move(*result.observation_route),
                    *result.observation_waypoint, result.version};
                final_adapted_arrival_confirmed_ = false;
                if (result.phase == ExplorationPhase::kAdapt) {
                    snapshot_.state = ExplorationState::kExploring;
                    FinishPhase(
                        PhaseStatus::kSucceeded,
                        "adapted pose preserves observation requirements");
                    continue;
                }
                view_evidence_.assign(view_count, {});
                view_started_.assign(view_count, {});
                current_view_ = 0;
                snapshot_.observed_view_count = 0;
                snapshot_.state = ExplorationState::kExploring;
                SubmitNavigation(observation_leg_->waypoint.pose, now);
                continue;
            }
            if (result.phase == ExplorationPhase::kSelect &&
                !result.decision.goal && coverage_.PendingCount() &&
                (result.decision.sampling_limited ||
                 result.decision.sweep_checked) &&
                !recovery_rotation_ && !recovery_loss_ && !finishing_) {
                // The sampler found useful views but none has an admissible
                // route from the current pose. A bounded in-place
                // re-observation is safer than repeatedly choosing the same
                // unreachable intent or relaxing the command gate while the
                // robot is near an obstacle.
                FinishPhase(PhaseStatus::kRetryable,
                            "no admissible route; bounded re-observation");
                if (StartRecoveryRotation(now)) {
                    AINFO << "EXPLORATION_ROUTE_RECOVERY pending="
                          << coverage_.PendingCount();
                    continue;
                }
            }
            if (result.decision.goal) {
                Retry(now,
                      "no adaptation preserves required observation coverage");
                continue;
            }
            if (options_.prune_unobservable_units &&
                result.decision.sweep_checked &&
                !result.decision.sampling_limited && coverage_.PendingCount()) {
                if (++unobservable_sweeps_ >=
                    std::max(1U, options_.prune_after_sweeps)) {
                    std::vector<CoverageId> unobservable;
                    for (const auto& item : coverage_.units())
                        if (item.second.pending)
                            unobservable.push_back(item.first);
                    for (const auto& id : unobservable)
                        coverage_.Remove(id);
                    unobservable_sweeps_ = 0;
                    AINFO << "EXPLORATION_PRUNE unobservable="
                          << unobservable.size()
                          << " pending_after=" << coverage_.PendingCount();
                }
            } else {
                unobservable_sweeps_ = 0;
            }
            if (recovery_loss_) {
                if (!coverage_.PendingCount() && !snapshot_.live_frontiers &&
                    result.decision.sweep_checked &&
                    result.decision.grid_fallback_checked &&
                    !result.decision.sampling_limited)
                    BeginLossRecovery(result.version, now);
                if (result.phase == ExplorationPhase::kSelect)
                    FinishPhase(
                        PhaseStatus::kRetryable,
                        "observation loss requires verified scope recovery");
                continue;
            }
            CompletionEvidence evidence;
            evidence.map_epoch = map_->map_epoch;
            evidence.geometry_revision = map_->geometry_revision;
            evidence.map_watermark = map_->observation_watermark;
            evidence.graph_watermark =
                result.graph_evidence ? result.graph_evidence->watermark : 0;
            evidence.observation_watermark = observations_.watermark();
            evidence.require_source_cuts = true;
            evidence.mapping_synchronized = map_->mapping_synchronized;
            evidence.map_cut = map_->observation_cut;
            evidence.graph_cut = result.graph_evidence
                                     ? result.graph_evidence->observation_cut
                                     : perception::ObservationCut{};
            evidence.observation_cut = result.version.observation_cut;
            for (auto source : {perception::SensorKind::kLidar,
                                perception::SensorKind::kRgbd}) {
                const auto index =
                    source == perception::SensorKind::kLidar ? 0U : 1U;
                if ((valid_observation_cut_.present &
                     evidence.observation_cut.present &
                     perception::Mask(source)))
                    evidence.valid_observation_cut.Include(
                        source,
                        std::min(valid_observation_cut_.sequence[index],
                                 evidence.observation_cut.sequence[index]));
            }
            evidence.required_observation_sources =
                snapshot_.sensors.active_sensors;
            for (const auto& item : coverage_.units())
                evidence.required_observation_sources |=
                    item.second.required_sensors;
            evidence.mapping_synchronized =
                evidence.mapping_synchronized && result.graph_evidence &&
                result.graph_evidence->map_epoch == map_->map_epoch &&
                result.graph_evidence->geometry_revision ==
                    map_->geometry_revision &&
                BaselineMatches(*map_, result.version) &&
                BaselineMatches(*result.graph_evidence, result.version);
            evidence.sensing_healthy = snapshot_.sensors.ready;
            evidence.localization_healthy = PoseReady(now);
            evidence.observation_queue_empty =
                InputPrefixProcessed(result.version) &&
                !PendingInvalidation() &&
                !observations_.LossSnapshot().pending_sources &&
                !observations_.HasUnrecoveredLossThrough(
                    evidence.observation_cut);
            evidence.pending_observations = coverage_.PendingCount();
            evidence.reachable_frontiers = std::count_if(
                coverage_.units().begin(), coverage_.units().end(),
                [](const auto& entry) {
                    return entry.first.kind == CoverageKind::kFrontier &&
                           entry.second.pending;
                });
            evidence.unresolved_regions =
                result.decision.sampling_limited ? 1 : 0;
            // A complete enumeration over an empty/unobserved traversable
            // domain is not exploration completion. This also prevents a
            // newly known all-occupied/unknown map from erasing the last work
            // item and vacuously satisfying the frontier-free predicate.
            const auto current_cell =
                map_->grid ? map_->grid->WorldToCell(pose_) : std::nullopt;
            if (!current_cell ||
                map_->grid->At(current_cell->x, current_cell->y).state !=
                    autonomy::navigation::CellState::kFree)
                evidence.unresolved_regions =
                    std::max<std::size_t>(evidence.unresolved_regions, 1);
            evidence.grid_fallback_checked =
                result.decision.grid_fallback_checked;
            evidence.sweep_checked = result.decision.sweep_checked;
            evidence.sweep_revision = map_->geometry_revision;
            evidence.confirmation_round = ++confirmation_round_;
            const auto verified = completion_.Evaluate(evidence);
            snapshot_.reason = verified.reason;
            snapshot_.state = ExplorationState::kVerifyingCompletion;
            if (result.phase == ExplorationPhase::kSelect)
                FinishPhase(PhaseStatus::kRetryable, verified.reason);
            if (verified.complete) {
                completion_version_ = result.version;
                completion_version_->valid_until =
                    now + options_.navigation_timeout +
                    2 * options_.map_operation_timeout;
                snapshot_.completion_confirmed = true;
                BeginFinish(ExplorationOutcome::kCompleted, now);
            } else if (coverage_.PendingCount() &&
                       result.decision.sweep_checked &&
                       !result.decision.sampling_limited) {
                Retry(now,
                      "pending coverage has no confirmed reachable observation "
                      "pose");
            }
        }
    }

    /** @brief Confirms a fixed admission prefix while allowing newer queued
     * suffixes. */
    bool InputPrefixProcessed(const RequestVersion& version) const {
        std::lock_guard<std::mutex> lock(events_mutex_);
        return processed_event_sequence_ >= version.input_event_sequence &&
               priority_events_.empty() &&
               observations_.ProcessedThrough(version.observation_cut);
    }

    /** @brief Detects admitted work or health changes that cannot be hidden by
     * a cut. */
    bool PendingInvalidation() const {
        std::lock_guard<std::mutex> lock(events_mutex_);
        if (!priority_events_.empty())
            return true;
        for (const auto& event : events_) {
            if (std::holds_alternative<CoverageUnit>(event))
                return true;
            if (const auto* map = std::get_if<MapReadView>(&event)) {
                if (!map_ || map->map_epoch != map_->map_epoch ||
                    map->geometry_revision != map_->geometry_revision)
                    return true;
            } else if (const auto* pose = std::get_if<PoseEvent>(&event)) {
                if (!pose->localization || pose->epoch != pose_epoch_)
                    return true;
            } else if (const auto* packet = std::get_if<PacketEvent>(&event)) {
                if (!packet->packet.Valid(last_now_) ||
                    packet->packet.observation.map_epoch != pose_epoch_)
                    return true;
            }
        }
        return false;
    }

    /** @brief Keeps refresh/save receipts bound to the actual completed
     * evidence cut. */
    void BindCompletionPrefix(RequestVersion& version) const {
        if (snapshot_.result.exploration != ExplorationOutcome::kCompleted ||
            !completion_version_)
            return;
        version.observation_cut = completion_version_->observation_cut;
        version.input_event_sequence =
            completion_version_->input_event_sequence;
        version.requirements_revision =
            completion_version_->requirements_revision;
        version.observation_baseline =
            completion_version_->observation_baseline;
        version.observation_recovery_generation =
            completion_version_->observation_recovery_generation;
    }

    /** @brief Validates current health, new work and loss-free finite
     * completion evidence. */
    bool FinalCompletionInputsValid(double now) const {
        return completion_version_ && now <= completion_version_->valid_until &&
               !execution_fault_ && snapshot_.sensors.ready && PoseReady(now) &&
               completion_version_->map_epoch == map_->map_epoch &&
               completion_version_->geometry_revision ==
                   map_->geometry_revision &&
               completion_version_->sensor_epoch ==
                   snapshot_.sensors.sensor_epoch &&
               completion_version_->requirements_revision ==
                   coverage_.requirements_revision() &&
               InputPrefixProcessed(*completion_version_) &&
               !PendingInvalidation() &&
               !observations_.LossSnapshot().pending_sources &&
               !observations_.HasUnrecoveredLossThrough(
                   completion_version_->observation_cut) &&
               BaselineMatches(*map_, *completion_version_) &&
               !coverage_.PendingCount() && !snapshot_.live_frontiers;
    }

    ExplorationSnapshot Tick(double now) {
        if (!std::isfinite(now) || now < last_now_)
            throw std::invalid_argument(
                "exploration requires a monotonic clock");
        if (!owner_)
            owner_ = std::this_thread::get_id();
        if (*owner_ != std::this_thread::get_id())
            throw std::logic_error(
                "exploration Tick must have one owner thread");
        try {
            return TickOwner(now);
        } catch (...) {
            // Service exceptions must not escape the task owner or strand a
            // motion request. Cancellation callbacks themselves cannot throw.
            CancelPlanning();
            CancelNavigation();
            CancelMapOperation();
            CancelRecoveryOperation();
            FinishPhase(PhaseStatus::kFailed,
                        "exploration owner service raised an exception");
            snapshot_.completion_confirmed = false;
            snapshot_.result.exploration =
                ExplorationOutcome::kIncompleteObservation;
            if (options_.require_return)
                snapshot_.result.returned = ReturnOutcome::kReturnFailed;
            if (options_.require_save)
                snapshot_.result.saved = MapSaveOutcome::kSaveFailed;
            snapshot_.reason = "exploration owner service raised an exception";
            snapshot_.state = ExplorationState::kFinished;
            Publish();
            return Snapshot();
        }
    }

    ExplorationSnapshot TickOwner(double now) {
        last_now_ = now;
        if (snapshot_.state == ExplorationState::kIdle ||
            snapshot_.state == ExplorationState::kFinished)
            return Snapshot();
        std::deque<Event> events;
        {
            std::lock_guard<std::mutex> lock(events_mutex_);
            for (std::size_t count = 0;
                 count < options_.maximum_events_per_tick; ++count) {
                auto& queue =
                    !priority_events_.empty() ? priority_events_ : events_;
                if (queue.empty())
                    break;
                events.push_back(std::move(queue.front()));
                queue.pop_front();
            }
        }
        for (auto& event : events) {
            if (const auto* map = std::get_if<MapReadView>(&event))
                ApplyMap(*map);
            else if (const auto* pose = std::get_if<PoseEvent>(&event))
                ApplyPose(*pose);
            else if (auto* packet = std::get_if<PacketEvent>(&event))
                ApplyPacket(std::move(*packet), now);
            else if (const auto* unit = std::get_if<CoverageUnit>(&event)) {
                const Pose2d center{unit->center.x, unit->center.y, 0};
                const bool inside =
                    (!options_.allowed_region ||
                     options_.allowed_region->Contains(center)) &&
                    (!options_.allowed_polygon ||
                     PolygonBoundaryContains(*options_.allowed_polygon,
                                             center.x, center.y)) &&
                    PolygonBoundaryOutsideAll(options_.no_go_areas, center.x,
                                              center.y);
                if (!inside)
                    continue;
                const auto before = coverage_.revision();
                coverage_.Upsert(*unit);
                if (before != coverage_.revision())
                    RevokeCompletion();
            } else if (const auto* command =
                           std::get_if<SessionCommand>(&event))
                Command(*command, now);
            else if (const auto* phase =
                         std::get_if<PhaseCancelEvent>(&event)) {
                if (active_phase_ && active_phase_->SameRequest(phase->ticket))
                    CancelActivePhase(PhaseStatus::kCanceled,
                                      "phase halted by its owner");
            } else {
                const auto& version = std::get<CancelEvent>(event).version;
                if (navigation_ && navigation_->SameRequest(version)) {
                    CancelNavigation();
                    ReleaseGoal();
                    snapshot_.state = finishing_ ? ExplorationState::kReturning
                                                 : ExplorationState::kExploring;
                }
                if (planning_ && planning_->SameRequest(version))
                    CancelPlanning();
                if (map_operation_ && map_operation_->SameRequest(version)) {
                    CancelMapOperation();
                    snapshot_.result.saved = MapSaveOutcome::kSaveFailed;
                    snapshot_.state = ExplorationState::kFinished;
                }
                if (recovery_operation_ &&
                    recovery_operation_->SameRequest(version))
                    CancelRecoveryOperation();
            }
            if (!std::holds_alternative<SessionCommand>(event) &&
                !std::holds_alternative<CancelEvent>(event) &&
                !std::holds_alternative<PhaseCancelEvent>(event)) {
                std::lock_guard<std::mutex> lock(events_mutex_);
                ++processed_event_sequence_;
            }
            if (snapshot_.state == ExplorationState::kFinished)
                break;
        }
        if (snapshot_.state == ExplorationState::kFinished) {
            Publish();
            return Snapshot();
        }
        if (snapshot_.state == ExplorationState::kPaused) {
            Publish();
            return Snapshot();
        }
        SyncObservationLoss(now);
        for (const auto& observation :
             observations_.Drain(now, coverage_.epoch(),
                                 options_.maximum_observations_per_tick)) {
            if (map_ && observation.frame_id != map_->frame_id) {
                observations_.Reject(observation.sensor, observation.sequence);
                continue;
            }
            health_.Observe(observation);
            observed_cut_.Include(observation.sensor, observation.sequence);
            const auto acquisition =
                ObservationPoseAt(observation, observation.stamp);
            if (!observation.rays.empty() && acquisition &&
                (acquisition->localization &
                 perception::Mask(observation.sensor)) &&
                observation.valid_fraction >=
                    options_.health.minimum_valid_fraction &&
                observation.scan_completeness >=
                    options_.health.minimum_completeness) {
                valid_observation_cut_.Include(observation.sensor,
                                               observation.sequence);
                if (ReplacementEligible(observation)) {
                    CoverageEvaluator{}.Apply(observation, now, coverage_);
                    RecordObservation(observation, now);
                    const auto packet = packet_poses_.find(ObservationIdentity{
                        observation.sensor, observation.sequence});
                    if (packet != packet_poses_.end()) {
                        keyposes_.Record(
                            {acquisition->pose, observation.sensor,
                             observation.sequence, observation.map_epoch,
                             map_->geometry_revision,
                             packet->second.calibration_revision,
                             observation.stamp, observation.frame_id});
                    }
                }
            } else
                observations_.Reject(observation.sensor, observation.sequence);
        }
        PrunePacketPoses();
        snapshot_.sensors = health_.Evaluate(
            now, localization_, options_.validated_controller_modes);
        PollRecoveryRotation(now);
        SyncObservationLoss(now);
        snapshot_.pending_units = coverage_.PendingCount();
        if (snapshot_.state == ExplorationState::kSaving) {
            if (snapshot_.result.exploration ==
                    ExplorationOutcome::kCompleted &&
                (!snapshot_.sensors.ready || !PoseReady(now))) {
                snapshot_.result.exploration =
                    ExplorationOutcome::kIncompleteObservation;
                snapshot_.completion_confirmed = false;
                snapshot_.reason =
                    "localization or sensing became unavailable during map "
                    "saving";
            }
            PollMapOperation(now);
            Publish();
            return Snapshot();
        }
        if (execution_fault_) {
            BeginFinish(ExplorationOutcome::kIncompleteObservation, now);
            Publish();
            return Snapshot();
        }
        if (!snapshot_.sensors.ready || !PoseReady(now)) {
            worker_reset_requested_.store(true);
            completion_.Reset();
            completion_version_.reset();
            CancelActivePhase(PhaseStatus::kRetryable,
                              "sensor or localization unavailable");
            CancelPlanning();
            CancelNavigation();
            CancelObservationRotation();
            CancelStartupObservation();
            CancelRecoveryOperation();
            snapshot_.state = ExplorationState::kWaitingForSensor;
            if (!PoseReady(now)) {
                AINFO_EVERY(20)
                    << "EXPLORATION_SENSOR_WAIT ready="
                    << snapshot_.sensors.ready << " active="
                    << static_cast<unsigned>(snapshot_.sensors.active_sensors)
                    << " localization=" << static_cast<unsigned>(localization_)
                    << " pose_ready=" << PoseReady(now)
                    << " pose_age=" << (now - pose_stamp_);
                if (!wait_started_)
                    wait_started_ = now;
            } else {
                // Localization is back: the health monitor may still be
                // re-arming, but the outage window must not keep running on a
                // recovered pose.
                wait_started_.reset();
            }
            if (wait_started_ &&
                now - *wait_started_ > options_.maximum_sensor_wait)
                BeginFinish(!localization_
                                ? ExplorationOutcome::kFailedLocalization
                                : ExplorationOutcome::kIncompleteObservation,
                            now);
            Publish();
            return Snapshot();
        }
        wait_started_.reset();
        PollStartupObservation(now);
        if (snapshot_.state == ExplorationState::kFinished ||
            snapshot_.state == ExplorationState::kPaused) {
            Publish();
            return Snapshot();
        }
        if (resume_phase_ && active_phase_) {
            resume_phase_ = false;
            const auto phase = active_phase_->phase;
            if (phase == ExplorationPhase::kSelect ||
                phase == ExplorationPhase::kAdapt)
                QueuePlanning(now, false, false, phase);
            else if (phase == ExplorationPhase::kObserve && snapshot_.goal) {
                snapshot_.state = ExplorationState::kExploring;
                SubmitNavigation(snapshot_.goal->execution_pose, now);
            } else if (phase == ExplorationPhase::kReturn) {
                if (At(home_)) {
                    snapshot_.result.returned = ReturnOutcome::kReturned;
                    AfterReturn(now);
                } else
                    QueuePlanning(now, true, false, phase);
            }
        }
        if (active_phase_ && now > active_phase_->version.valid_until) {
            const auto phase = active_phase_->phase;
            CancelActivePhase(PhaseStatus::kFailed, "phase deadline exceeded");
            if (phase == ExplorationPhase::kReturn)
                snapshot_.result.returned = ReturnOutcome::kReturnFailed;
            else if (phase == ExplorationPhase::kSave)
                snapshot_.result.saved = MapSaveOutcome::kSaveFailed;
            snapshot_.reason = "phase deadline exceeded";
        }
        const double elapsed = now - started_ - paused_time_;
        const bool return_known =
            return_cache_ && map_ &&
            return_cache_->version.map_epoch == map_->map_epoch &&
            return_cache_->version.geometry_revision ==
                map_->geometry_revision &&
            return_cache_->version.sensor_epoch ==
                snapshot_.sensors.sensor_epoch;
        // Re-tracing the measured executed path to a route anchor is a
        // conservative route cost. Never substitute a straight line home.
        const double return_distance =
            options_.require_return && return_known
                ? return_cache_->length +
                      std::max(0.0, snapshot_.traveled_distance -
                                        return_cache_->traveled_distance)
                : 0;
        const double return_reserve =
            options_.require_return && return_known
                ? options_.return_margin +
                      return_distance / options_.nominal_speed
                : 0;
        if (!finishing_ && (elapsed + return_reserve >= options_.time_budget ||
                            snapshot_.traveled_distance + return_distance >=
                                options_.distance_budget))
            BeginFinish(ExplorationOutcome::kPartialBudget, now);
        if (!finishing_ && options_.require_return && !home_valid_)
            BeginFinish(ExplorationOutcome::kIncompleteObservation, now);
        if (!finishing_ && options_.require_return && !return_known &&
            navigation_ && !planning_)
            QueuePlanning(now, false, true);
        if (planning_ && now > planning_->valid_until) {
            CancelPlanning();
            if (finishing_) {
                snapshot_.result.returned = ReturnOutcome::kReturnFailed;
                AfterReturn(now);
            } else
                Retry(now, "planning deadline exceeded");
        }
        ConsumePlan(now);
        PollLossRecovery(now);
        PollObservationRotation(now);
        PollRecoveryRotation(now);
        if (navigation_) {
            const auto reply = services_.poll_navigation();
            if (reply && reply->version.SameRequest(*navigation_) &&
                now <= navigation_->valid_until) {
                navigation_.reset();
                if (finishing_) {
                    snapshot_.result.returned =
                        reply->success && AtHome()
                            ? ReturnOutcome::kReturned
                            : ReturnOutcome::kReturnFailed;
                    AfterReturn(now);
                } else if (reply->success && snapshot_.goal &&
                           observation_leg_) {
                    const auto waypoint = observation_leg_->waypoint;
                    observation_leg_.reset();
                    if (!At(waypoint.pose)) {
                        Retry(now, "route waypoint arrival was not measured");
                    } else if (!waypoint.final_endpoint) {
                        CancelPlanning();
                        snapshot_.state = ExplorationState::kReplanning;
                        QueueObservationContinuation(now);
                    } else if (!At(snapshot_.goal->execution_pose)) {
                        Retry(now,
                              "final adapted pose arrival was not measured");
                    } else {
                        final_adapted_arrival_confirmed_ = true;
                        BeginObservationAtFinal(now);
                    }
                } else if (reply->success && snapshot_.goal &&
                           final_adapted_arrival_confirmed_ &&
                           observation_view_target_ &&
                           AtCurrentRequiredView()) {
                    arrived_at_ = now;
                    snapshot_.state = ExplorationState::kObserving;
                } else {
                    Retry(now,
                          "navigation failed or arrival pose not confirmed");
                    if (StartRecoveryRotation(now))
                        snapshot_.reason =
                            "navigation failed; bounded re-observation started";
                }
            } else if (now > navigation_->valid_until) {
                if (finishing_) {
                    CancelNavigation();
                    snapshot_.result.returned = ReturnOutcome::kReturnFailed;
                    AfterReturn(now);
                } else {
                    Retry(now, "navigation deadline exceeded");
                    if (StartRecoveryRotation(now))
                        snapshot_.reason =
                            "navigation deadline; bounded re-observation "
                            "started";
                }
            }
        }
        if (snapshot_.state == ExplorationState::kObserving) {
            if (GoalObserved(now)) {
                snapshot_.last_observed_goal_id =
                    snapshot_.goal->intent.goal_id;
                snapshot_.last_observed_goal_version =
                    snapshot_.goal->intent.goal_version;
                snapshot_.observed_view_count = view_evidence_.size();
                ReleaseGoal();
                observation_leg_.reset();
                observation_view_target_.reset();
                final_adapted_arrival_confirmed_ = false;
                snapshot_.selection.reset();
                prepared_version_.reset();
                CancelObservationRotation();
                FinishPhase(PhaseStatus::kSucceeded,
                            "required views and actual observations confirmed");
                recovery_count_ = 0;
                snapshot_.state = ExplorationState::kExploring;
            } else if (current_view_ + 1 < view_evidence_.size() &&
                       ViewObserved(current_view_, now)) {
                ++current_view_;
                snapshot_.observed_view_count = current_view_;
                auto view_pose = snapshot_.goal->execution_pose;
                view_pose.yaw =
                    snapshot_.goal->intent.required_views[current_view_];
                snapshot_.state = ExplorationState::kExploring;
                if (HasObservationRotation()) {
                    if (!StartObservationRotation(view_pose.yaw, now))
                        Retry(now, "next viewpoint rotation could not start");
                } else
                    SubmitNavigation(view_pose, now);
            } else if (now - arrived_at_ > options_.observation_timeout)
                Retry(now, "arrival did not realize required observation");
        }
        if (!BtDriven() && options_.startup_observation.enabled &&
            !startup_observation_confirmed_ && !startup_observation_) {
            if (!BeginStartupObservation(now))
                BeginFinish(ExplorationOutcome::kIncompleteObservation, now);
        }
        if (!BtDriven() && startup_observation_confirmed_ && !planning_ &&
            !navigation_ && !recovery_operation_ && !observation_rotation_ &&
            !recovery_rotation_ && now >= selection_after_ &&
            snapshot_.state != ExplorationState::kObserving &&
            snapshot_.state != ExplorationState::kSaving &&
            snapshot_.state != ExplorationState::kFinished)
            QueuePlanning(now, finishing_);
        Publish();
        return Snapshot();
    }

    void Publish() {
        if (map_ && map_->grid && coverage_.epoch() == map_->map_epoch &&
            (!snapshot_.measured_coverage_map ||
             snapshot_.measured_coverage_revision != coverage_.revision() ||
             snapshot_.measured_coverage_map_epoch != coverage_.epoch() ||
             snapshot_.measured_coverage_geometry_revision !=
                 map_->geometry_revision)) {
            snapshot_.measured_coverage_map = std::make_shared<
                const ::automsgs::msgs::map_msgs::OccupancyGrid>(
                BuildMeasuredCoverageMap(coverage_, *map_, last_now_));
            snapshot_.measured_coverage_revision = coverage_.revision();
            snapshot_.measured_coverage_map_epoch = coverage_.epoch();
            snapshot_.measured_coverage_geometry_revision =
                map_->geometry_revision;
        }
        if (!snapshot_.acquisition_keyposes ||
            snapshot_.acquisition_keypose_generation !=
                keyposes_.generation()) {
            snapshot_.acquisition_keyposes =
                std::make_shared<const std::vector<AcquisitionKeypose>>(
                    keyposes_.entries().begin(), keyposes_.entries().end());
            snapshot_.acquisition_keypose_generation = keyposes_.generation();
        }
        // Early paused/terminal paths still consume admitted requirement
        // events. Publish ledger counts from actual state on every path, never
        // from a prediction or a previous running tick's cached progress.
        snapshot_.pending_units = coverage_.PendingCount();
        snapshot_.motion_ready = !execution_fault_ && snapshot_.sensors.ready &&
                                 PoseReady(last_now_) &&
                                 snapshot_.state != ExplorationState::kPaused &&
                                 snapshot_.state != ExplorationState::kIdle &&
                                 snapshot_.state != ExplorationState::kFinished;
        snapshot_.active_request =
            startup_observation_
                ? startup_observation_
                : (navigation_
                       ? navigation_
                       : (planning_ ? planning_
                                    : (map_operation_ ? map_operation_
                                                      : recovery_operation_)));
        snapshot_.startup_observation_required =
            options_.startup_observation.enabled;
        snapshot_.startup_observation_confirmed =
            startup_observation_confirmed_;
        if (snapshot_.startup_observation_reason.empty())
            snapshot_.startup_observation_reason =
                startup_observation_confirmed_ ? "startup observation confirmed"
                                               : "startup observation pending";
        snapshot_.active_phase = active_phase_;
        snapshot_.finishing = finishing_;
        snapshot_.observation_losses = observations_.LossSnapshot();
        snapshot_.accepted_observations = observations_.accepted_cut();
        snapshot_.processed_observations = observations_.processed_cut();
        snapshot_.valid_observations = valid_observation_cut_;
        snapshot_.observation_baseline = observation_baseline_;
        snapshot_.observation_recovery_generation =
            observation_recovery_generation_;
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        published_ = snapshot_;
        published_phases_ = phase_history_;
    }
    ExplorationSnapshot Snapshot() const {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        return published_;
    }

    std::shared_ptr<ExplorationStrategy> strategy_;
    std::shared_ptr<const GoalPoseAdapter> adapter_;
    std::shared_ptr<const PathProvider> paths_;
    ExplorationServices services_;
    ExplorationSessionOptions options_;
    perception::ObservationBuffer observations_;
    AcquisitionKeyposeLedger keyposes_;
    perception::SensorHealthMonitor health_;
    CompletionEvaluator completion_;
    CoverageState coverage_;
    std::set<CoverageId> dormant_requirements_;
    perception::ObservationCut observed_cut_;
    perception::ObservationCut valid_observation_cut_;
    std::optional<MapReadView> map_;
    Pose2d pose_, home_;
    std::uint64_t pose_epoch_ = 0, home_epoch_ = 0, pose_sequence_ = 0;
    perception::SensorMask localization_ = 0;
    double pose_stamp_ = 0, started_ = 0, last_now_ = 0, paused_time_ = 0;
    double arrived_at_ = 0, selection_after_ = 0;
    std::optional<double> paused_at_, wait_started_;
    std::optional<ExplorationState> paused_state_;
    std::optional<ReturnCache> return_cache_;
    std::optional<ActiveObservationLeg> observation_leg_;
    std::optional<Pose2d> observation_view_target_;
    bool final_adapted_arrival_confirmed_ = false;
    std::deque<PoseEvent> pose_history_;  ///< Bounded fallback history and
                                          ///< current-motion accounting.
    std::map<ObservationIdentity, PacketPoses>
        packet_poses_;  ///< Owner-only proofs for bounded pending observations.
    std::vector<std::map<CoverageId, std::array<double, 2>>> view_evidence_;
    std::vector<std::optional<double>> view_started_;
    std::size_t current_view_ = 0;
    bool home_valid_ = false, finishing_ = false, saving_map_ = false;
    bool execution_fault_ = false;
    unsigned recovery_count_ = 0;
    std::uint64_t request_counter_ = 0, confirmation_round_ = 0;
    std::uint64_t worker_session_ = 0;
    std::uint64_t worker_motion_sequence_ = 0;
    std::atomic<bool> worker_reset_requested_{
        true};  ///< Rebuild proofs after real sensing/pose loss.
    std::optional<std::thread::id> owner_;
    ExplorationSnapshot snapshot_, published_;
    std::optional<RequestVersion> planning_, navigation_, map_operation_;
    std::optional<RequestVersion> observation_rotation_;
    std::optional<RequestVersion>
        recovery_rotation_;  ///< Loss-only yaw sweep without a coverage goal.
    std::optional<Pose2d> pending_navigation_pose_;
    bool rotation_for_navigation_ = false;
    std::optional<RequestVersion>
        startup_observation_;  ///< Bounded native in-place yaw sweep awaiting a
                               ///< measured result.
    bool startup_observation_confirmed_ =
        true;  ///< Default remains true when the deployment policy is disabled.
    bool startup_observation_failed_ =
        false;  ///< Native rejection or validation failure remains fail-closed.
    std::optional<RequestVersion> selected_version_, prepared_version_;
    std::optional<RequestVersion>
        completion_version_;  ///< Finite, expiring completed evidence cut.
    std::optional<perception::ObservationLossSnapshot>
        recovery_loss_;  ///< Loss checkpoint whose scope is reopened.
    std::optional<RequestVersion>
        recovery_operation_;  ///< Independently cancellable recovery refresh
                              ///< request.
    std::optional<MappingReply>
        recovery_reply_;  ///< Matched receipt retained until consumer map/graph
                          ///< catch up.
    perception::ObservationCut
        observation_baseline_{};  ///< Last verified excluded historical prefix.
    std::uint64_t observation_recovery_generation_ =
        0;  ///< Last verified replacement-baseline identity.
    std::uint64_t recovery_geometry_ = 0, recovery_requirements_ = 0,
                  recovery_sensor_epoch_ = 0;
    unsigned unobservable_sweeps_ =
        0;  ///< Consecutive zero-gain complete sweeps.
    unsigned loss_recovery_attempts_ =
        0;  ///< Bounded refresh attempts for one compatible loss scope.
    double loss_recovery_after_ = 0;  ///< Earliest owner-clock time of a retry.
    std::optional<PhaseTicket> active_phase_;
    std::deque<PhaseResult> phase_history_, published_phases_;
    bool resume_phase_ = false;
    std::shared_ptr<std::atomic<bool>> planning_cancel_;
    mutable std::mutex snapshot_mutex_;
    mutable std::mutex events_mutex_;
    std::mutex work_mutex_;
    std::condition_variable work_ready_;
    std::deque<Event> events_;
    std::deque<Event> priority_events_;
    std::uint64_t accepted_event_sequence_ =
        0;  ///< Finite ordinary-event admission endpoint.
    std::uint64_t admission_epoch_ =
        0;  ///< Packet epoch protected by events_mutex_.
    mutable std::mutex
        depth_mutex_;  ///< Protects immutable depth history readers.
    std::deque<std::shared_ptr<const perception::LocalizedDepthView>>
        depth_history_;  ///< At most four real views.
    std::uint64_t processed_event_sequence_ =
        0;  ///< Owner-processed ordinary-event endpoint.
    std::optional<Job> queued_job_;
    std::deque<Planned> planned_results_;
    std::thread worker_;
    bool stopping_ = false;
};

ExplorationSession::ExplorationSession(
    std::shared_ptr<ExplorationStrategy> strategy,
    std::shared_ptr<const GoalPoseAdapter> adapter,
    std::shared_ptr<const PathProvider> paths, ExplorationServices services,
    ExplorationSessionOptions options)
    : impl_(std::make_unique<Impl>(std::move(strategy), std::move(adapter),
                                   std::move(paths), std::move(services),
                                   options)) {}
ExplorationSession::~ExplorationSession() = default;
void ExplorationSession::Start(const Pose2d& home, std::uint64_t epoch,
                               double now) {
    impl_->Start(home, epoch, now);
}
bool ExplorationSession::PushMap(MapReadView map) {
    return impl_->Enqueue(std::move(map));
}
bool ExplorationSession::PushPose(Pose2d pose, double stamp,
                                  std::uint64_t epoch,
                                  perception::SensorMask localization) {
    return impl_->Enqueue(Impl::PoseEvent{pose, stamp, epoch, localization});
}
bool ExplorationSession::PushObservation(
    perception::SensorObservation observation, double now) {
    return impl_->observations_.Push(std::move(observation), now);
}
bool ExplorationSession::PushSensorPacket(perception::ObservationPacket packet,
                                          double now) {
    return impl_->PushPacket(std::move(packet), now);
}
void ExplorationSession::RejectSensorInput(perception::SensorKind sensor,
                                           std::uint64_t sequence) {
    impl_->observations_.Reject(sensor, sequence);
}
std::vector<std::shared_ptr<const perception::LocalizedDepthView>>
ExplorationSession::RecentDepthViews(double now) const {
    return impl_->DepthViews(now);
}
bool ExplorationSession::PushCoverageUnit(CoverageUnit unit) {
    return impl_->Enqueue(std::move(unit));
}
bool ExplorationSession::PostCommand(SessionCommand command) {
    return impl_->Enqueue(command);
}
bool ExplorationSession::CancelRequest(const RequestVersion& version) {
    return impl_->Enqueue(Impl::CancelEvent{version});
}
std::optional<PhaseTicket> ExplorationSession::RequestPhase(
    ExplorationPhase phase, double now) {
    return impl_->RequestPhase(phase, now);
}
PhaseResult ExplorationSession::PhaseState(const PhaseTicket& ticket) const {
    return impl_->PhaseState(ticket);
}
bool ExplorationSession::CancelPhase(const PhaseTicket& ticket) {
    return impl_->Enqueue(Impl::PhaseCancelEvent{ticket});
}
ExplorationDriver ExplorationSession::driver() const {
    return impl_->options_.driver;
}
ExplorationSnapshot ExplorationSession::Tick(double now) {
    return impl_->Tick(now);
}
ExplorationSnapshot ExplorationSession::Snapshot() const {
    return impl_->Snapshot();
}
}  // namespace autonomy::task::exploration
