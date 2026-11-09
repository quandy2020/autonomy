/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

/**
 * @file runner.hpp
 * @brief Framework-agnostic business core of Navigation.
 */

#pragma once

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>

#include "navigation/common/joinable_thread.hpp"
#include "navigation/common/pose2d.hpp"
#include "navigation/common/status_or.hpp"
#include "navigation/common/twist2d.hpp"
#include "navigation/memory/context_provider.hpp"
#include "navigation/memory/context_view.hpp"
#include "navigation/perception/observation_encoder.hpp"
#include "navigation/planning/chunk_blender.hpp"
#include "navigation/planning/interface/policy_interface.hpp"
#include "navigation/planning/scenario_manager.hpp"
#include "navigation/planning/tasks/recovery_maneuver.hpp"
#include "navigation/proto/config.pb.h"
#include "navigation/proto/navigation.pb.h"
#include "navigation/proto/observation.pb.h"

namespace navigation {

/**
 * @brief Business core: observation -> memory -> policy -> chunk -> command.
 *
 * @details
 * Lifecycle: construct -> Start() -> Tick() per observation -> Stop().
 * Policy inference runs on a dedicated worker; Tick never blocks on it. The
 * worker receives the latest job only (single-flight), and stale results are
 * discarded by sequence number.
 *
 * @par Threading
 * Pipeline operations are serialized, including goal setters called from an
 * action-server thread. The policy worker calls only PolicyInterface::Query.
 *
 * @par Ownership
 * Runner owns all injected dependencies and the policy worker thread.
 */
class Runner
{
public:
    /**
     * @brief Injected pipeline dependencies.
     *
     * @details Used by tests and composition roots; every pointer may be
     * nullptr and is checked by Start().
     */
    struct Dependencies {
        /** @brief Encodes Observation into DepthFeature; required. */
        std::unique_ptr<perception::ObservationEncoder> encoder{nullptr};
        /** @brief Pose-indexed context provider; required. */
        std::unique_ptr<memory::ContextProvider> memory{nullptr};
        /** @brief End-to-end policy plugin; required. */
        std::unique_ptr<planning::PolicyInterface> policy{nullptr};
        /** @brief Action-chunk blender; required. */
        std::unique_ptr<planning::ChunkBlender> blender{nullptr};
        /** @brief Optional scenario + rule pipeline applied after blending. */
        std::unique_ptr<planning::ScenarioManager> scenario{nullptr};
        /** @brief Optional steady clock for deterministic control tests. */
        std::function<std::int64_t()> steady_now_ns;
    };

    /**
     * @brief Construct Runner from configuration and injected dependencies.
     *
     * @param[in] configuration Validated navigation configuration.
     * @param[in] dependencies Pipeline components; ownership transferred.
     */
    Runner(config::NavigationConfig configuration, Dependencies dependencies);

    /** @brief Stops the worker and releases dependencies. */
    ~Runner();

    Runner(const Runner&) = delete;
    Runner& operator=(const Runner&) = delete;

    /**
     * @brief Create the default pipeline from configuration.
     *
     * @param[in] configuration Navigation configuration.
     * @return Runner on success, or nullptr when a plugin is not registered.
     */
    static std::unique_ptr<Runner> Create(
        const config::NavigationConfig& configuration);

    /**
     * @brief Start the policy worker.
     *
     * @return false when a required dependency is missing.
     * @note Idempotent while already running.
     */
    bool Start();

    /**
     * @brief Advance one tick.
     *
     * @param[in] observation Fused sensor snapshot for this tick.
     * @param[in] current_pose Map-frame planar pose.
     * @param[in] current_velocity Measured planar velocity.
     * @return Command on success; nullopt when stopped, no goal, or invalid
     *         input. A nullopt must be treated as "hold zero" by the caller.
     */
    std::optional<common::Twist2D> Tick(
        const observation::Observation& observation,
        const common::Pose2D& current_pose,
        const common::Twist2D& current_velocity,
        bool recovery_permitted = true);

    /**
     * @brief Stop the worker and clear the action chunk.
     *
     * @note Blocks until an in-flight policy query finishes.
     */
    void Stop();

    /**
     * @brief Set the active goal and reset scenario/blender state.
     *
     * @param[in] goal New goal; goal_id is treated as an idempotency key.
     */
    void SetGoal(const GoalSpec& goal);

    /** @brief Return whether a goal is currently active. */
    [[nodiscard]] bool HasActiveGoal() const;

    /** @brief Clear the goal and reset downstream state. */
    void ClearGoal();

    /** @brief Return whether the last Tick observed the goal tolerance. */
    [[nodiscard]] bool IsGoalReached() const;

    bool RequestRecovery();
    void FailGoal();
    [[nodiscard]] bool IsGoalFailed() const;
    [[nodiscard]] RecoveryState RecoveryStatus() const;
    [[nodiscard]] std::uint32_t RecoveryCount() const;
    [[nodiscard]] std::string RecoveryPhase() const;

    /**
     * @brief Reset spatial memory.
     *
     * @param[in] keep_pose_index Keep the pose index when true.
     * @return false when no memory provider is configured.
     */
    bool ResetMemory(bool keep_pose_index);

    /** @brief Return the number of stored keyframes. */
    [[nodiscard]] std::size_t MemoryEntryCount() const;

private:
    /** @brief Work item handed to the policy worker. */
    struct PolicyJob {
        /** @brief Observation used for this inference. */
        observation::Observation observation;
        /** @brief Pre-encoded depth feature (reused memory feature). */
        observation::DepthFeature feature;
        /** @brief Goal snapshot. */
        GoalSpec goal;
        /** @brief Immutable context snapshot. */
        memory::ContextView context;
        /** @brief Monotonic job sequence. */
        std::uint64_t sequence{0};
        std::uint64_t generation{0};
    };

    /** @brief Result produced by the policy worker. */
    struct PolicyResult {
        /** @brief Intent or error; stale results are discarded. */
        common::StatusOr<NavigationIntent> intent;
        /** @brief Sequence copied from the job. */
        std::uint64_t sequence{0};
    };

    /** @brief Worker thread entry point; runs until worker_stop_. */
    void WorkerLoop();

    /**
     * @brief Non-blocking result drain.
     *
     * @param[out] result Latest result when available.
     * @return true when a result was moved out.
     */
    bool TryTakeResult(PolicyResult* result);

    /** @brief Drop queued/completed work and invalidate an in-flight query. */
    void InvalidatePolicyWork();

    /**
     * @brief Submit the latest job when the decision period elapsed.
     *
     * @return true when a job was queued.
     */
    bool PostJobIfDue(const observation::Observation& observation,
                      const observation::DepthFeature& feature,
                      const common::Pose2D& pose, std::int64_t now_ns,
                      double dt_s);

    /**
     * @brief Update the goal-reached latch from the current pose.
     *
     * @param[in] pose Current map-frame pose.
     */
    void UpdateGoalReached(const common::Pose2D& pose);

    std::int64_t SteadyNowNs() const;
    void ResetRecovery();
    void UpdateProgress(const common::Pose2D& pose, std::int64_t now_ns);

    /** @brief Configuration snapshot. */
    config::NavigationConfig configuration_;
    /** @brief Pipeline components owned by Runner. */
    Dependencies dependencies_;

    /** @brief Serializes pipeline state across tick and action-server threads.
     */
    mutable std::mutex pipeline_mutex_;

    /** @brief Protects active_goal_ and goal_reached_. */
    mutable std::mutex goal_mutex_;
    /** @brief Current goal, if any. */
    std::optional<GoalSpec> active_goal_;
    /** @brief Goal-reached latch updated by Tick. */
    bool goal_reached_{false};

    /** @brief Protects worker job/result handoff. */
    std::mutex worker_mutex_;
    /** @brief Signals the worker when a job arrives or stop is requested. */
    std::condition_variable worker_cv_;
    /** @brief Latest queued job (single-flight). */
    std::optional<PolicyJob> pending_job_;
    /** @brief Latest undrained result. */
    std::optional<PolicyResult> pending_result_;
    /** @brief Stop flag observed by the worker. */
    bool worker_stop_{false};
    /** @brief True while the worker is executing a query. */
    bool worker_busy_{false};
    /** @brief True when pending_result_ is valid. */
    bool result_ready_{false};
    /** @brief True after Start() and before Stop(). */
    bool running_{false};
    /** @brief Next job sequence. */
    std::uint64_t next_sequence_{1};
    /** @brief Oldest accepted job sequence, protected by worker_mutex_. */
    std::uint64_t minimum_sequence_{1};
    /** @brief Policy worker; joins on destruction/Stop. */
    common::JoinableThread worker_;

    /** @brief Last Tick time on the monotonic clock, nanoseconds. */
    std::int64_t last_tick_ns_{0};
    /** @brief Last policy submission time, nanoseconds. */
    std::int64_t last_decision_ns_{0};
    /** @brief Minimum interval between policy submissions, nanoseconds. */
    std::uint64_t decision_period_ns_{200000000};
    planning::RecoveryManeuver recovery_;
    RecoveryState recovery_state_{RECOVERY_IDLE};
    std::uint32_t recovery_count_{0};
    bool goal_failed_{false}, have_pose_{false};
    common::Pose2D latest_pose_;
    double best_progress_error_{1e9};
    bool progress_is_yaw_{false};
    std::int64_t last_progress_ns_{0};
};

}  // namespace navigation
