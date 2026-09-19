/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file cancel_registry.hpp
 * @brief Named cancel hooks via autonomy::common::Factory.
 *
 * @details
 * DomainBundle registers one cancel action per stub at construction; Estop /
 * CancelAllTasks iterate the retained instances. Creators capture non-owning
 * Stub* (Bundle UniquePtr stubs outlive CancelRegistry teardown).
 *
 * @par Invariants
 * - Register once at DomainBundle construction; CancelAll runs stored actions.
 * - Hooks capture raw Stub*; DomainBundle destroys registry_ before stubs.
 * - Unnamed RegisterCancel(hook) auto-assigns hook_N ids for compat callers.
 * - Clear drops both Factory creators and retained action instances.
 *
 * @par Threading
 * Not internally synchronized; register during single-threaded DomainBundle
 * construction, CancelAll from the gRPC / control path that already serializes
 * Estop.
 *
 * @see DomainBundle
 * @see RegisterDomainCancel
 */

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "autonomy/common/factory.hpp"
#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {

/**
 * @brief Polymorphic cancel action (Factory product).
 *
 * Each registered domain hook is materialized as a CancelAction instance and
 * retained by CancelRegistry until Clear / destruction. Run() must be
 * idempotent-friendly (stubs tolerate cancel when idle).
 *
 * @par Ownership
 * Factory creates via raw new; CancelRegistry stores unique_ptr.
 *
 * @par Threading
 * Run() is invoked from CancelAll; do not assume a specific worker.
 */
class CancelAction
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(CancelAction)

    virtual ~CancelAction() = default;

    /**
     * @brief Execute the cancel / reset side-effect for one domain.
     *
     * @note Implementations should not throw; stubs catch or no-op when idle.
     */
    virtual void Run() = 0;
};

/**
 * @brief CancelAction that invokes an owned callable.
 *
 * @tparam Fn Move-only or copyable callable with signature compatible with
 *           void(). Typically CancelRegistry::CancelHook (std::function<void()>).
 *
 * @par Ownership
 * owns @p Fn by value; Run() no-ops if the callable is empty.
 */
template <typename Fn>
class FnCancelAction : public CancelAction
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(FnCancelAction<Fn>)

    /**
     * @brief Construct from a callable that performs cancel work.
     *
     * @param[in] fn Callable invoked by Run(); moved into this object.
     */
    explicit FnCancelAction(Fn fn) : fn_(std::move(fn)) {}

    /**
     * @brief Invoke the stored callable if non-empty.
     */
    void Run() override {
        if (fn_) {
            fn_();
        }
    }

private:
    /**
     * @brief Owned cancel callable invoked by Run() when non-empty.
     */
    Fn fn_;
};

/**
 * @brief Factory: string id → CancelAction creator.
 *
 * Keyed by human-readable hook id (e.g. "navigator", "teleop"). Creators
 * return raw CancelAction* ownership transferred to the Factory / registry.
 */
using CancelFactory = ::autonomy::common::Factory<
    std::string, CancelAction, std::function<CancelAction*()>>;

/**
 * @brief Collects domain cancel actions for CancelAll / Estop.
 *
 * Dual storage: CancelFactory retains creators (Contains / Unregister), while
 * actions_ retains live unique_ptr<CancelAction> instances so CancelAll does
 * not re-CreateObject each time.
 *
 * @par Ownership
 * owns Factory + action instances; hooks capture non-owning Stub*
 * (see RegisterDomainCancel).
 *
 * @par Threading
 * Register* expected during construction; CancelAll / Clear from
 * the control path that owns DomainBundle.
 */
class CancelRegistry
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(CancelRegistry)

    /**
     * @brief Callable registered as a cancel hook (no arguments).
     */
    using CancelHook = std::function<void()>;

    /**
     * @brief Register a named cancel action (Factory + retain instance).
     *
     * Creates an FnCancelAction<CancelHook> via the Factory, then stores the
     * resulting unique_ptr in actions_. On CreateObject failure the Factory
     * entry is Unregistered and false is returned.
     *
     * @param[in] id   Unique hook id (e.g. "navigator").
     * @param[in] hook Callable invoked by CancelAll; must be non-empty.
     * @return         false if @p hook is empty, @p id was already registered, or
     *                CreateObject failed.
     *
     * @note Duplicate ids are rejected; use Clear before re-wiring in tests.
     */
    bool RegisterCancel(const std::string& id, CancelHook hook) {
        if (!hook) {
            return false;
        }
        auto creator = [hook = std::move(hook)]() -> CancelAction* {
            return new FnCancelAction<CancelHook>(hook);
        };
        if (!factory_.Register(id, std::move(creator))) {
            return false;
        }
        auto action = factory_.CreateObject(id);
        if (!action) {
            factory_.Unregister(id);
            return false;
        }
        actions_.push_back(std::move(action));
        return true;
    }

    /**
     * @brief Append an unnamed hook (compat for RegisterDomainCancel).
     *
     * Auto-assigns id "hook_N" where N is the current actions_.size() before
     * insertion. Failures of the named overload are ignored (compat path).
     *
     * @param[in] hook Callable invoked by CancelAll.
     */
    void RegisterCancel(CancelHook hook) {
        const std::string id = "hook_" + std::to_string(actions_.size());
        RegisterCancel(id, std::move(hook));
    }

    /**
     * @brief Run every retained CancelAction in registration order.
     *
     * @note Null entries are skipped; individual Run() failures are the
     * stub's responsibility (should not throw).
     */
    void CancelAll() {
        for (auto& action : actions_) {
            if (action) {
                action->Run();
            }
        }
    }

    /**
     * @brief Drop all retained actions and Factory creators.
     *
     * @warning After Clear, Contains() is false for all previously registered
     * ids; DomainBundle does not re-register automatically.
     */
    void Clear() {
        actions_.clear();
        factory_.Clear();
    }

    /**
     * @brief Whether a named creator is present in the Factory.
     *
     * @param[in] id Hook id previously passed to RegisterCancel.
     * @return       true if the Factory still contains @p id.
     */
    bool Contains(const std::string& id) const { return factory_.Contains(id); }

private:
    /**
     * @brief Factory of named CancelAction creators (Contains / Unregister).
     */
    CancelFactory factory_;

    /**
     * @brief Retained CancelAction instances run by CancelAll in order.
     */
    std::vector<CancelAction::UniquePtr> actions_;
};

}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
