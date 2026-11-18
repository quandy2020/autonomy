/* Copyright 2026 The Openbot Authors. */
/**
 * @file navigation_dispatcher.hpp
 * @brief Cancellable asynchronous dispatch of versioned navigation requests.
 */
#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "autonomy/task/exploration/exploration_context.hpp"

namespace autonomy::task::exploration {

/**
 * @brief Issues revocable movement leases for one exploration generation.
 *
 * Pause revokes existing leases. Resume permits new leases without making old
 * leases valid again. Retirement is permanent, including across late callbacks.
 */
class MotionGate {
public:
    /// A lease is checked by the native task while waiting and before execution.
    using Permit = std::function<bool()>;
    /** @brief Construct a disabled generation gate with optional immutable parent ownership. */
    explicit MotionGate(Permit parent = {});
    /**
     * @brief Publish pause/resume intent and revoke earlier revision leases.
     * @param enabled Whether newly captured leases may authorize movement.
     */
    void SetEnabled(bool enabled);
    /** @brief Permanently prohibit further movement by this generation. */
    void Retire();
    /** @return Predicate bound to the current revision and shared state lifetime. */
    Permit Lease() const;
    /** @return True only if new leases may currently authorize movement. */
    bool Enabled() const;

private:
    /// State shared with leases, without a pointer to an ExplorationTask.
    struct State;
    /// Survives task destruction until every outstanding lease is released.
    std::shared_ptr<State> state_;
};

/** @brief Immutable native dispatch payload, independent of protobuf bindings. */
struct NavigationDispatchRequest {
    /// Native TaskHeader identity, including process and task generation.
    std::string task_id;
    /// Selected and validated observation/return target.
    Pose2d pose;
    /// Algorithm identity used by the owning Session.
    RequestVersion version;
};

/**
 * @brief Serializes potentially blocking native submissions off the BT owner.
 *
 * Enqueue/CancelRequest are bounded queue operations. Native Submit may wait
 * cooperatively on the independent worker. The submit callback must observe
 * its permit throughout readiness waiting and before starting motion. Native
 * cancellation and result reads are conditional on the supplied task ID.
 * The owner shuts this dispatcher down before releasing native dependencies.
 */
class NavigationDispatcher {
public:
    /// Native acceptance; the callback must cooperate with revoked permits.
    using Submit = std::function<bool(const NavigationDispatchRequest&,
                                     const MotionGate::Permit&)>;
    /// Nullopt means pending; a value is an identity-checked terminal result.
    using Result = std::function<std::optional<bool>(const std::string&)>;
    /// Cancel only the task with this identity; callbacks run without our locks.
    using Cancel = std::function<void(const std::string&)>;

    /**
     * @brief Start a dispatch worker using immutable callbacks and identity.
     * @param gate Movement authorization for this task generation.
     * @param identity_prefix Process/task/generation namespace.
     * @param submit Cooperative native START dispatcher.
     * @param result Identity-aware terminal read.
     * @param cancel Identity-aware native cancellation.
     */
    NavigationDispatcher(std::shared_ptr<MotionGate> gate,
        std::string identity_prefix, Submit submit, Result result, Cancel cancel);
    /// Destroy on an external owner after revoking and joining native callbacks.
    ~NavigationDispatcher();
    NavigationDispatcher(const NavigationDispatcher&) = delete;
    NavigationDispatcher& operator=(const NavigationDispatcher&) = delete;

    /**
     * @brief Enqueue at most one current request without blocking on native START.
     * @param pose Validated local observation/return target.
     * @param version Immutable algorithm request identity.
     * @return True for a new queued request or its identical duplicate payload.
     */
    bool Enqueue(const Pose2d& pose, const RequestVersion& version);
    /** @return Matching terminal reply, or nullopt while pending/canceled. */
    std::optional<NavigationReply> Poll();
    /**
     * @brief Revoke matching work immediately and queue its native cancellation.
     * @param version Only this current request may be revoked; old IDs are ignored.
     */
    void CancelRequest(const RequestVersion& version);
    /** @return True while any submission or owned cancellation remains in flight. */
    bool HasInFlight() const;
    /** @return True if a native cancellation could not be confirmed. */
    bool Failed() const;
    /// Close admission and drain work; worker reentry requests shutdown only.
    void Shutdown();

private:
    /// Worker and queue implementation; no mutable Session/Tree is held here.
    class Impl;
    /// Join ownership remains with this wrapper, never with callback state.
    std::unique_ptr<Impl> impl_;
};

}  // namespace autonomy::task::exploration
