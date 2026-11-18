/* Copyright 2026 The Openbot Authors. */
/**
 * @file motion_dispatch_gate.hpp
 * @brief Revocable ownership of native asynchronous action sends.
 */
#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autonomy::task::common {

/// Blackboard key installed by the owner of one navigation invocation.
inline constexpr char kMotionDispatchGateBlackboardKey[] = "motion_dispatch_gate";

/**
 * @brief Register cancellation before an asynchronous send can be committed.
 *
 * Start and Close serialize on one mutex. A send committed before Close remains
 * owned until rejection or a terminal result, including late acceptance after
 * revocation. Network calls and cancellation callbacks never hold this mutex.
 * Closing is not a physical stop acknowledgment. A new invocation must drain
 * the preceding movement operations before acquiring movement ownership.
 */
class MotionDispatchGate {
public:
    /// Bounded cancellation initiation; must capture the specific goal identity.
    using Cancel = std::function<void()>;
    /// Nonblocking query for a cancellation/transport failure.
    using Failure = std::function<bool()>;
    /**
     * @brief Cooperative permission retained for one invocation, never rebound.
     * Queries must be bounded, thread-safe and nonblocking. They must not call
     * task control/admission methods or reenter this gate. The callback runs
     * outside the gate mutex; false or an exception permanently revokes this
     * invocation. Existing sends still require cancellation and terminal drain.
     */
    using Permit = std::function<bool()>;
    /// Planning may be revoked without participating in physical-motion drain.
    enum class Kind { kMotion, kPlanning };

private:
    /** @brief One registered send and its cancellation responsibility. */
    struct Operation {
        Cancel cancel;                    ///< Goal-specific cancellation callback.
        Failure failure;                  ///< Nonblocking transport health query.
        Kind kind{Kind::kMotion};           ///< Determines movement drain ownership.
        std::atomic<bool> done{false};      ///< Motion terminal or locally retired planning.
        std::atomic<bool> failed{false};    ///< Sticky locally observed failure.
        bool started{false};               ///< Send commitment, protected by state mutex.
        bool cancel_started{false};        ///< Cancellation responsibility already invoked.
    };
    /** @brief Shared state; no pointer to a task, tree or node is retained. */
    struct State {
        /** @brief Retain an immutable external query for this invocation. */
        explicit State(Permit permission) : permit(std::move(permission)) {}
        const Permit permit;               ///< Immutable external owner query, if supplied.
        std::atomic<bool> permit_revoked{false}; ///< Sticky external denial for this invocation.
        std::mutex mutex;                  ///< Orders enable, commitment and revocation.
        std::condition_variable wake;      ///< Terminal events wake bounded drain waits.
        bool enabled{false};               ///< New commitments are currently permitted.
        bool suspended{false};             ///< Pause retains intent while preventing sends.
        uint64_t revision{0};              ///< Old tickets never revive after resume.
        uint64_t next_id{0};               ///< Stable operation identity in this gate.
        std::atomic<bool> fault{false};     ///< Prevents reuse after uncertain cancellation.
        std::unordered_map<uint64_t, std::shared_ptr<Operation>> operations;
    };

    /** @brief Poll the external owner without the gate lock; denial never revives. */
    static bool CheckPermit(const std::shared_ptr<State>& state) noexcept {
        if (state->permit_revoked.load()) return false;
        if (state->permit) {
            bool allowed = false;
            try { allowed = state->permit(); } catch (...) {}
            if (!allowed) {
                state->permit_revoked.store(true);
                state->wake.notify_all();
            }
        }
        return !state->permit_revoked.load();
    }

public:
    /** @brief A single send reservation, retained by its transport callbacks. */
    class Ticket {
    public:
        /**
         * @brief Commit at most one send against the captured revision.
         * @return False if closed, superseded, failed or already committed.
         * A refused first commitment is settled because no network send occurred.
         */
        bool Start() {
            auto state = state_.lock();
            auto operation = operation_.lock();
            if (!state || !operation) return false;
            CheckPermit(state);
            std::lock_guard<std::mutex> lock(state->mutex);
            if (operation->started || operation->done.load()) return false;
            if (!state->enabled || state->revision != revision_ || state->fault.load() ||
                state->permit_revoked.load()) {
                operation->done.store(true);
                state->wake.notify_all();
                return false;
            }
            operation->started = true;
            return true;
        }
        /** @brief Confirm rejection or an authoritative terminal result. */
        void Finish() {
            if (auto operation = operation_.lock()) operation->done.store(true);
            if (auto state = state_.lock()) state->wake.notify_all();
        }
        /** @brief Record uncertainty; this does not fabricate a terminal result. */
        void Fail() {
            auto operation = operation_.lock();
            auto state = state_.lock();
            if (!operation) return;
            operation->failed.store(true);
            if (state && operation->kind == Kind::kMotion) {
                state->fault.store(true);
                state->wake.notify_all();
            }
        }
        /** @return True if this revision may no longer publish action success. */
        bool Revoked() const {
            auto state = state_.lock();
            if (!state) return true;
            CheckPermit(state);
            std::lock_guard<std::mutex> lock(state->mutex);
            return !state->enabled || state->revision != revision_ || state->fault.load() ||
                   state->permit_revoked.load();
        }

        /**
         * @brief Keep this owner's intent while a paused tick finishes.
         * @return True only for suspension without a cancellation or sticky fault.
         * This never grants dispatch or success publication. A later pause may
         * suspend a not-yet-resent ticket from an earlier resumed revision.
         */
        bool Suspended() const {
            auto state = state_.lock();
            if (!state) return false;
            CheckPermit(state);
            std::lock_guard<std::mutex> lock(state->mutex);
            return state->suspended && !state->enabled &&
                   state->revision >= revision_ && !state->fault.load() &&
                   !state->permit_revoked.load();
        }

        /**
         * @brief Observe a later enabled revision without granting a new send.
         * @return True only after the owner has drained motion and resumed.
         * The caller must still reserve and commit a fresh ticket. This query
         * cannot revive the old ticket or bypass a concurrent Close().
         */
        bool HasResumedRevision() const {
            auto state = state_.lock();
            if (!state) return false;
            CheckPermit(state);
            std::lock_guard<std::mutex> lock(state->mutex);
            return state->enabled && !state->fault.load() && !state->permit_revoked.load() &&
                   state->revision > revision_;
        }

        /**
         * @brief Retire canceled local planning ownership without faking a result.
         * @return False for motion, which must retain authoritative termination.
         * The planning caller must request cancellation first. Transport-owned
         * callbacks retain their UUID cancellation state for a late acceptance.
         */
        bool RetirePlanning() {
            auto operation = operation_.lock();
            if (!operation || operation->kind != Kind::kPlanning) return false;
            operation->done.store(true);
            if (auto state = state_.lock()) state->wake.notify_all();
            return true;
        }

    private:
        friend class MotionDispatchGate;
        Ticket(const std::shared_ptr<State>& state,
               const std::shared_ptr<Operation>& operation, uint64_t revision)
            : state_(state), operation_(operation), revision_(revision) {}
        std::weak_ptr<State> state_;         ///< Does not prolong the task owner lifetime.
        std::weak_ptr<Operation> operation_; ///< Avoids cycles through native goal handles.
        uint64_t revision_;                 ///< Revision captured by Reserve.
    };

    /// Construct a disabled gate; the navigation owner explicitly enables it.
    explicit MotionDispatchGate(Permit permit = {})
        : state_(std::make_shared<State>(std::move(permit))) {}
    /// Initiate outstanding cancellation before releasing owned registrations.
    ~MotionDispatchGate() { Close(); Revoke(); }
    MotionDispatchGate(const MotionDispatchGate&) = delete;
    MotionDispatchGate& operator=(const MotionDispatchGate&) = delete;

    /** @return Whether this invocation retained an external owner query. */
    bool HasExternalPermit() const { return static_cast<bool>(state_->permit); }
    /**
     * @brief Query permission without enabling sends or claiming physical stop.
     * @return False after the first external denial or query exception.
     */
    bool PermitValid() const noexcept { return CheckPermit(state_); }

    /** @return True when a fresh revision can be enabled without uncertainty. */
    bool Enable() {
        if (!PermitValid() || HasFault() || !Drained()) return false;
        std::lock_guard<std::mutex> lock(state_->mutex);
        if (state_->fault.load() || state_->permit_revoked.load() ||
            state_->revision == std::numeric_limits<uint64_t>::max()) return false;
        for (const auto& entry : state_->operations)
            if (entry.second->kind == Kind::kMotion && !entry.second->done.load()) return false;
        if (!state_->enabled) {
            ++state_->revision;
            state_->enabled = true;
            state_->suspended = false;
        }
        return true;
    }
    /**
     * @brief Close admission without running any user/transport callback.
     * Safe while the navigation owner holds its lifecycle mutex.
     */
    void Close() {
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->enabled = false;
        state_->suspended = false;
        state_->wake.notify_all();
    }
    /**
     * @brief Close send admission while retaining a paused invocation's intent.
     * The owner must still Revoke() and obtain terminal evidence before Enable().
     * Close() overrides this marker for cancellation, replacement or shutdown.
     */
    void Suspend() {
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->enabled = false;
        state_->suspended = true;
        state_->wake.notify_all();
    }
    /** @return True when admission is paused without a sticky movement fault. */
    bool Suspended() const {
        CheckPermit(state_);
        std::lock_guard<std::mutex> lock(state_->mutex);
        return state_->suspended && !state_->enabled && !state_->fault.load() &&
               !state_->permit_revoked.load();
    }
    /**
     * @brief Close and invoke each outstanding goal cancellation outside locks.
     * An explicit Suspend() remains distinguishable from final Close().
     */
    void Revoke() noexcept {
        std::vector<std::shared_ptr<Operation>> to_cancel;
        try {
            {
                std::lock_guard<std::mutex> lock(state_->mutex);
                state_->enabled = false;
                for (auto& entry : state_->operations) {
                    auto& operation = entry.second;
                    // No send commitment occurred. Settling an abandoned
                    // reservation cannot fabricate a remote terminal result.
                    if (!operation->started) operation->done.store(true);
                    if (!operation->done.load() && !operation->cancel_started) {
                        operation->cancel_started = true;
                        to_cancel.push_back(operation);
                    }
                }
                state_->wake.notify_all();
            }
            for (const auto& operation : to_cancel) {
                try { if (operation->cancel) operation->cancel(); }
                catch (...) {
                    operation->failed.store(true);
                    if (operation->kind == Kind::kMotion) state_->fault.store(true);
                }
            }
        } catch (...) { state_->fault.store(true); }
    }
    /**
     * @brief Own a goal's cancellation before committing its network send.
     * @param cancel Goal-specific bounded cancellation initiation.
     * @param failure Nonblocking query; never called with the gate mutex held.
     * @param kind Whether this operation can actuate the robot.
     * @return A ticket, or null when admission is closed or failed.
     */
    std::shared_ptr<Ticket> Reserve(Cancel cancel, Failure failure,
                                    Kind kind = Kind::kMotion) {
        if (!PermitValid() || HasFault()) return {};
        auto operation = std::make_shared<Operation>();
        operation->cancel = std::move(cancel);
        operation->failure = std::move(failure);
        operation->kind = kind;
        // Destruct captured callbacks only after releasing the gate mutex.
        std::vector<std::shared_ptr<Operation>> discarded;
        std::shared_ptr<Ticket> ticket;
        {
            std::lock_guard<std::mutex> lock(state_->mutex);
            if (!state_->enabled || state_->fault.load() || state_->permit_revoked.load()) return {};
            for (auto it = state_->operations.begin(); it != state_->operations.end();) {
                if (it->second->done.load()) {
                    discarded.push_back(std::move(it->second));
                    it = state_->operations.erase(it);
                } else ++it;
            }
            if (state_->next_id == std::numeric_limits<uint64_t>::max()) {
                state_->fault.store(true);
                return {};
            }
            state_->operations.emplace(++state_->next_id, operation);
            ticket = std::shared_ptr<Ticket>(new Ticket(state_, operation, state_->revision));
        }
        return ticket;
    }
    /** @return True only when every registered movement operation is terminal. */
    bool Drained() const {
        std::lock_guard<std::mutex> lock(state_->mutex);
        for (const auto& entry : state_->operations)
            if (entry.second->kind == Kind::kMotion && !entry.second->done.load()) return false;
        return true;
    }
    /** @return Sticky movement transport/cancellation failure, polled without locks. */
    bool HasFault() const {
        if (state_->fault.load()) return true;
        std::vector<std::shared_ptr<Operation>> operations;
        {
            std::lock_guard<std::mutex> lock(state_->mutex);
            for (const auto& entry : state_->operations)
                if (entry.second->kind == Kind::kMotion && !entry.second->done.load())
                    operations.push_back(entry.second);
        }
        for (const auto& operation : operations) {
            bool failed = operation->failed.load();
            try { failed = failed || (operation->failure && operation->failure()); }
            catch (...) { failed = true; }
            if (failed) state_->fault.store(true);
        }
        return state_->fault.load();
    }
    /**
     * @brief Wait for terminal evidence after Revoke, with a fixed deadline.
     * @param timeout Maximum duration; no task lifecycle or gate lock is retained.
     * @return False on fault or timeout. Cancel RPC acceptance alone is insufficient.
     */
    bool WaitForDrain(std::chrono::milliseconds timeout) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        for (;;) {
            if (HasFault()) return false;
            if (Drained()) return true;
            const auto now = std::chrono::steady_clock::now();
            if (now >= deadline) {
                state_->fault.store(true);
                Close();
                return false;
            }
            std::unique_lock<std::mutex> lock(state_->mutex);
            state_->wake.wait_until(lock,
                std::min(deadline, now + std::chrono::milliseconds(10)));
        }
    }

private:
    std::shared_ptr<State> state_; ///< Owns registrations until terminal or destruction.
};

}  // namespace autonomy::task::common
