/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file domains.hpp
 * @brief DomainBundle: owns domain stubs (UniquePtr) + CancelRegistry (by value).
 *
 * @details
 * Ownership model:
 * - Each stub is UniquePtr; DomainBundle is the sole owner.
 * - CancelRegistry is a value member declared *after* stubs so it is
 *   destroyed first (hooks capture raw Stub* valid until Clear).
 * - MapServiceStub holds a non-owning MappingStub* (Bundle lifetime).
 * - Shared infra (muxer / scheduler / idempotency) is injected, not owned.
 *
 * @par Invariants
 * - Construction order: Mapping before MapService; other domains independent.
 * - CancelAll only calls stub cancel/reset; never touches gRPC writers.
 * - Accessors return T&; callers must not store dangling references past
 *   DomainBundle destruction.
 *
 * @see Context
 * @see CancelRegistry
 * @see RegisterDomainCancel
 */

#pragma once

#include <memory>
#include <string>

#include "autolink/common/log.hpp"
#include "autolink/node/node.hpp"
#include "autonomy/bridge/grpc/cancel_registry.hpp"
#include "autonomy/bridge/grpc/clients/charge_stub.hpp"
#include "autonomy/bridge/grpc/clients/exploration_stub.hpp"
#include "autonomy/bridge/grpc/clients/follow_stub.hpp"
#include "autonomy/bridge/grpc/clients/localization_stub.hpp"
#include "autonomy/bridge/grpc/clients/map_service_stub.hpp"
#include "autonomy/bridge/grpc/clients/mapping_stub.hpp"
#include "autonomy/bridge/grpc/clients/navigator_stub.hpp"
#include "autonomy/bridge/grpc/clients/sensor_stub.hpp"
#include "autonomy/bridge/grpc/clients/system_monitor_stub.hpp"
#include "autonomy/bridge/grpc/clients/teleop_stub.hpp"
#include "autonomy/bridge/grpc/clients/voice_stub.hpp"
#include "autonomy/bridge/grpc/idempotency.hpp"
#include "autonomy/bridge/grpc/task_muxer.hpp"
#include "autonomy/bridge/grpc/work_scheduler.hpp"
#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {

/**
 * @brief Register a named cancel hook that invokes `fn(*stub)`.
 *
 * @details
 * Captures a non-owning @p stub pointer inside the CancelRegistry hook.
 * Safe only while DomainBundle keeps UniquePtr stubs alive at least as long
 * as CancelRegistry (registry_ is declared after stubs so it is destroyed
 * first; Clear / CancelAll therefore see live Stub*).
 *
 * @tparam Stub Domain stub type (NavigatorStub, TeleopStub, …).
 * @tparam Fn   Callable taking `Stub&` (typically a lambda calling Cancel /
 *              Reset / CancelSession).
 *
 * @param[in,out] registry Cancel registry that retains the hook.
 * @param[in]     id       Human-readable hook id (e.g. `"navigator"`).
 * @param[in]     stub     Non-owning pointer; may be null (hook no-ops).
 * @param[in]     fn       Cancel / reset functor applied to `*stub`.
 */
template <typename Stub, typename Fn>
void RegisterDomainCancel(CancelRegistry& registry, const std::string& id,
                          Stub* stub, Fn&& fn) {
    registry.RegisterCancel(
        id, [stub, fn = std::forward<Fn>(fn)]() mutable {
            if (stub) {
                fn(*stub);
            }
        });
}

/**
 * @brief Owns all domain stubs and their CancelAll hooks.
 *
 * @details
 * Constructed once by Context. Handlers never construct stubs; they obtain
 * `T&` via navigator() / teleop() / … (or Context facade accessors).
 * CancelAll walks CancelRegistry and invokes each domain's cancel/reset
 * without touching gRPC writers.
 *
 * @par Ownership
 * Sole owner of every domain stub UniquePtr. Injected muxer / scheduler /
 * idempotency are non-owning (Context / Server own them).
 *
 * @par Threading
 * Construction and RegisterCancels are single-threaded. CancelAll is invoked
 * from Estop / CancelAll RPC paths that already serialize control.
 *
 * @see Context
 * @see clients::NavigatorStub
 * @see clients::MapServiceStub
 */
class DomainBundle
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases for DomainBundle.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(DomainBundle)

    /**
     * @brief Construct stubs and register CancelAll hooks.
     *
     * @details
     * Builds stubs in dependency order: MappingStub before MapServiceStub
     * (MapService receives `mapping_.get()`). Then RegisterCancels() wires
     * named hooks into registry_.
     *
     * @param[in] node        Autolink node for stub readers / writers / clients.
     * @param[in] muxer       Shared exclusive-task muxer (retained by stubs).
     * @param[in] scheduler   Background work pool (non-owning; may be null).
     * @param[in] idempotency Optional cmd_id cache (non-owning; Teleop uses it).
     */
    DomainBundle(std::shared_ptr<autolink::Node> node,
                 TaskMuxer::SharedPtr muxer, WorkScheduler* scheduler,
                 CommandIdempotencyCache* idempotency);

    DomainBundle(const DomainBundle&) = delete;
    DomainBundle& operator=(const DomainBundle&) = delete;

    /**
     * @brief Run every registered domain cancel / reset hook.
     *
     * @details Invokes CancelRegistry::CancelAll in registration order.
     * Does not Clear the TaskMuxer slot; Context::CancelAllTasks does that.
     */
    void CancelAll() { registry_.CancelAll(); }

    /**
     * @brief Navigation GoalChannel stub (Navigate / lifecycle).
     * @return Live reference; valid for DomainBundle lifetime.
     */
    clients::NavigatorStub& navigator() { return Ref(navigator_); }

    /**
     * @brief Teleop stub (Velocity GoalChannel + relative Action backend).
     * @return Live reference; valid for DomainBundle lifetime.
     */
    clients::TeleopStub& teleop() { return Ref(teleop_); }

    /**
     * @brief Follow GoalChannel stub.
     * @return Live reference; valid for DomainBundle lifetime.
     */
    clients::FollowStub& follow() { return Ref(follow_); }

    /**
     * @brief Auto return-to-dock / leave-dock GoalChannel stub.
     * @return Live reference; valid for DomainBundle lifetime.
     */
    clients::ChargeStub& charge() { return Ref(charge_); }

    /**
     * @brief Mapping GoalChannel stub (load / session goals).
     * @return Live reference; valid for DomainBundle lifetime.
     */
    clients::MappingStub& mapping() { return Ref(mapping_); }

    /**
     * @brief MapService unary facade (catalog + session UX over MappingStub).
     * @return Live reference; valid for DomainBundle lifetime.
     */
    clients::MapServiceStub& map_service() { return Ref(map_service_); }

    /**
     * @brief Localization GoalChannel / pose helpers stub.
     * @return Live reference; valid for DomainBundle lifetime.
     */
    clients::LocalizationStub& localization() { return Ref(localization_); }

    /**
     * @brief Exploration GoalChannel stub.
     * @return Live reference; valid for DomainBundle lifetime.
     */
    clients::ExplorationStub& exploration() { return Ref(exploration_); }

    /**
     * @brief Voice GoalChannel stub.
     * @return Live reference; valid for DomainBundle lifetime.
     */
    clients::VoiceStub& voice() { return Ref(voice_); }

    /**
     * @brief Sensor list / sample / record facade stub.
     * @return Live reference; valid for DomainBundle lifetime.
     */
    clients::SensorStub& sensor() { return Ref(sensor_); }

    /**
     * @brief System health / monitor registry facade stub.
     * @return Live reference; valid for DomainBundle lifetime.
     */
    clients::SystemMonitorStub& system_monitor() {
        return Ref(system_monitor_);
    }

private:
    /**
     * @brief Dereference a stub UniquePtr with a null log (debug aid).
     *
     * @tparam Stub Domain stub type.
     * @param[in] ptr Owned UniquePtr; must be non-null after construction.
     * @return        Reference to `*ptr` (UB if null after the AERROR log).
     */
    template <typename Stub>
    static Stub& Ref(const typename Stub::UniquePtr& ptr) {
        if (!ptr) {
            AERROR << "DomainBundle: null stub UniquePtr";
        }
        return *ptr;
    }

    /**
     * @brief Wire named CancelRegistry hooks for every cancelable domain.
     *
     * @details Called once from the constructor after UniquePtr stubs exist.
     * Captures raw Stub* via RegisterDomainCancel.
     */
    void RegisterCancels();

    /**
     * @brief Owned NavigatorStub (Navigate / Pause / Resume / Cancel).
     */
    clients::NavigatorStub::UniquePtr navigator_{nullptr};

    /**
     * @brief Owned TeleopStub (velocity + DriveOnHeading / BackUp / Spin).
     */
    clients::TeleopStub::UniquePtr teleop_{nullptr};

    /**
     * @brief Owned FollowStub (person / target follow session).
     */
    clients::FollowStub::UniquePtr follow_{nullptr};

    /**
     * @brief Owned ChargeStub (return-to-dock / leave-dock).
     */
    clients::ChargeStub::UniquePtr charge_{nullptr};

    /**
     * @brief Owned MappingStub (GoalChannel map load / mapping session).
     *
     * @details Constructed before map_service_; MapServiceStub holds a
     * non-owning pointer into this object.
     */
    clients::MappingStub::UniquePtr mapping_{nullptr};

    /**
     * @brief Owned MapServiceStub (unary MapService over MappingStub + catalog).
     */
    clients::MapServiceStub::UniquePtr map_service_{nullptr};

    /**
     * @brief Owned LocalizationStub (pose / initial pose / status).
     */
    clients::LocalizationStub::UniquePtr localization_{nullptr};

    /**
     * @brief Owned ExplorationStub (explore session / area / save-map).
     */
    clients::ExplorationStub::UniquePtr exploration_{nullptr};

    /**
     * @brief Owned VoiceStub (voice execute GoalChannel).
     */
    clients::VoiceStub::UniquePtr voice_{nullptr};

    /**
     * @brief Owned SensorStub (list / sample / parameters / record).
     */
    clients::SensorStub::UniquePtr sensor_{nullptr};

    /**
     * @brief Owned SystemMonitorStub (health snapshot via MonitorRegistry).
     */
    clients::SystemMonitorStub::UniquePtr system_monitor_{nullptr};

    /**
     * @brief Named cancel hooks for Estop / CancelAllTasks.
     *
     * @details Declared last so destruction order clears hooks while UniquePtr
     * stubs are still alive (C++ destroys members in reverse declaration order).
     */
    CancelRegistry registry_;
};

}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
