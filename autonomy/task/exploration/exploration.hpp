/* Copyright 2026 The Openbot Authors. */
/**
 * @file exploration.hpp
 * @brief Native exploration task owning the session and behavior-tree loop.
 */
#pragma once

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <optional>
#include <string>

#include <automsgs/task/exploration.pb.h>
#include <automsgs/task/mapping.pb.h>
#include <automsgs/task/navigation.pb.h>
#include "autonomy/common/macros.hpp"
#include "autonomy/perception/observation/native_observation_source.hpp"
#include "autonomy/task/common/motion_admission.hpp"
#include "autonomy/task/common/typed_task.hpp"
#include "autonomy/task/exploration/exploration_session.hpp"
#include "autonomy/task/exploration/integration/navigation_dispatcher.hpp"

namespace autolink {
class Node;
}

namespace autonomy {
namespace task {

/** @brief Real mapper operation executed by the task owner after packet
 * validation. Generation zero updates bootstrap geometry only and is excluded
 * from receipts.
 */
using ExplorationPacketProcessor =
    std::function<std::optional<exploration::MapReadView>(
        std::uint64_t, const perception::ObservationPacket&, double)>;

/**
 * @brief Dependencies supplied by the native observation/route integration.
 *
 * Factory execution, Session::Start and tree ticks share the task owner thread.
 * The factory returns an unstarted BT-driven session with real versioned
 * services. Callbacks deliver observations through the task's bounded inbox.
 */
struct ExplorationTaskBindings {
    /// Builds a fresh session; must not start it or retain a mutable owner.
    std::function<std::shared_ptr<exploration::ExplorationSession>(
        const proto::ExplorationGoal&,
        const exploration::ExplorationSessionOptions&)>
        create_session;
    /// Confirms an actual mapping session, rather than accepting a map name.
    std::function<bool(const std::string&)> prepare_mapping;
    /// Clock shared by observations, poses, map stamps and the owner loop.
    std::function<double()> clock;
    ExplorationPacketProcessor
        process_sensor_packet;  ///< Actual mapper operation pinned to the
                                ///< native source revision.
    /// Initial immutable policy; per-goal map name/boundary are versioned.
    exploration::ExplorationSessionOptions options;
};

/** @brief Fixed sensor destination captured before asynchronous decoding.
 * Generation zero permits readiness seeding only before the first START.
 */
struct ExplorationSensorInputIdentity {
    std::uint64_t instance = 0;  ///< Unique task object identity.
    std::uint64_t generation =
        0;  ///< Accepted START generation at binding time.
};

/** @brief Sensor callbacks bound to one task generation through a weak owner.
 * TaskServer owns the task during execution. Replacement START needs new
 * callbacks.
 */
struct ExplorationSensorInputBinding {
    ExplorationSensorInputIdentity
        identity;  ///< Fixed destination for this source.
    std::function<bool(perception::ObservationPacket, double)>
        sink;  ///< Atomic packet admission.
    std::function<void(perception::SensorKind, std::uint64_t)>
        loss;  ///< Rejection for the same destination.
};

/** @brief Native input lifecycle evidence, separate from task/motion
 * completion. */
struct ExplorationObservationSourceSnapshot {
    bool configured = false;  ///< A real Node, source policy and localization
                              ///< lookup were installed.
    bool retiring = false;  ///< Old Readers still reserve their channels until
                            ///< actual cleanup ends.
    bool faulted = false;   ///< Startup, dependency or cleanup failure prevents
                            ///< automatic replacement.
    std::string error;  ///< Concrete cause; a retirement timeout never asserts
                        ///< completed cleanup.
    std::optional<ExplorationSensorInputIdentity>
        identity;  ///< Fixed destination of the current/retiring source.
    perception::NativeObservationSourceSnapshot
        source;  ///< Actual Reader/callback evidence.
};

/**
 * @brief First-class exploration API with one Session/BT owner thread.
 *
 * Inputs and commands are queued; producer callbacks never tick the session.
 * Coverage, return and save outcomes determine terminal status. Legacy
 * waypoint/finished messages cannot submit navigation or finish this task.
 */
class ExplorationTask : public TypedTaskAppBase<proto::ExplorationGoal,
                                                proto::ExplorationFeedback,
                                                proto::ExplorationResult>
{
public:
    /// TaskServer injects versioned session services, not a NavigationClient.
    static constexpr bool kUsesNavigationClient = false;
    /// Compatibility dispatcher type used by TaskServer.
    using SubmitNavigation = std::function<bool(const proto::NavigationGoal&)>;
    /**
     * @brief Native acceptance that cooperatively observes revoked movement.
     * @param goal Versioned navigation goal owned by this exploration session.
     * @param permit Checked during readiness and before/through execution.
     * @return True only when the native task accepted this request identity.
     */
    using GuardedSubmitNavigation = std::function<bool(
        const proto::NavigationGoal&, const exploration::MotionGate::Permit&)>;
    /// Compatibility mapping dispatcher; not a save/fusion acknowledgement.
    using SubmitMapping = std::function<bool(const proto::MappingGoal&)>;
    /// Legacy active probe retained as an API compatibility surface.
    using IsNavigationActive = std::function<bool()>;
    /// Legacy result probe cannot prove the identity of a completion.
    using GetNavigationResult = std::function<bool(proto::NavigationResult*)>;
    /// Atomic identity-aware terminal result read from the real navigation
    /// task.
    using GetVersionedNavigationResult =
        std::function<bool(const std::string&, proto::NavigationResult*)>;
    /// Cancels only the navigation goal carrying the given task identity.
    using CancelNavigationRequest = std::function<bool(const std::string&)>;

    AUTONOMY_SMART_PTR_DEFINITIONS(ExplorationTask)

    /// Construct an idle task; Initialize starts its owned worker.
    ExplorationTask();
    /// Stop the worker before destroying dependencies and state.
    ~ExplorationTask() override;
    /// Return the native exploration domain type.
    ::automsgs::msgs::vehicle_msgs::RobotTaskType GetTaskType() const override;
    /// Preserve native TaskServer node injection without legacy subscriptions.
    void SetNode(std::shared_ptr<autolink::Node> node);
    /// Install dispatcher while idle; factories may obtain versioned services.
    void SetSubmitNavigation(SubmitNavigation submit);
    /// Install cooperative native acceptance while no runtime is in flight.
    void SetGuardedSubmitNavigation(GuardedSubmitNavigation submit);
    /** @brief Install root movement admission while no start or runtime is in
     * flight. */
    void SetMotionAdmission(common::MotionAdmission::Factory factory);
    /** @return True only after pending starts, runtime disposal and owned
     * dispatcher cleanup. */
    bool MotionDrained() const;
    /// Retain mapping dispatcher; SAVE_MAP never maps to MAP_CMD_SWITCH.
    void SetSubmitMapping(SubmitMapping submit);
    /// Preserve old probe API; it is not used for authoritative completion.
    void SetNavigationProbe(IsNavigationActive active,
                            GetNavigationResult result);
    /// Install identity-aware native result/cancel operations while idle.
    void SetVersionedNavigationProbe(GetVersionedNavigationResult result,
                                     CancelNavigationRequest cancel);
    /// Configure real session dependencies while idle or terminal.
    bool ConfigureSession(ExplorationTaskBindings bindings);
    /** @brief Atomically install a session factory and its matching sensor
     * dependencies.
     * @param task Shared owner of this exact task; callbacks retain a weak
     * reference.
     * @param node Same native Node already installed by TaskServer.
     * @param bindings Owner-executed factory, clock and immutable session
     * policy.
     * @param input Validated native source policy with the same sensor mode.
     * @param localization Independent acquisition-time localization provider.
     * @param retirement_timeout Bounded wait for an old source's real
     * retirement.
     * @return False without replacing either dependency for active work or
     * invalid input.
     * @details The owner observes a single configuration revision, so it never
     * starts a new factory with the previous source policy during a mode
     * change.
     */
    bool ConfigureNativeRuntime(
        const std::shared_ptr<ExplorationTask>& task,
        const std::shared_ptr<autolink::Node>& node,
        ExplorationTaskBindings bindings,
        perception::NativeObservationSourceOptions input,
        perception::ObservationIngress::LocalizationLookup localization,
        std::chrono::milliseconds retirement_timeout = std::chrono::seconds(2));
    /** @brief Install native sensor input on an idle, shared-owned configured
     * task.
     * @param task TaskServer's actual task owner; callbacks retain only a weak
     * reference.
     * @param options Explicit channels, calibration, units and the session's
     * exact sensor mode.
     * @param localization Healthy acquisition-time poses supplied by the real
     * localization owner.
     * @param retirement_timeout Maximum replacement wait, measured with
     * steady_clock.
     * @return False for missing dependencies, active task, mode mismatch or
     * latched input failure.
     * @details The task worker starts readiness input before the first START,
     * rebuilds Readers for every accepted START and keeps the same input
     * sequence across pause. Replacement requires successful old Reader
     * retirement. This does not configure a localization backend, mapping
     * session or exploration session factory.
     */
    static bool ConfigureObservationSource(
        const std::shared_ptr<ExplorationTask>& task,
        perception::NativeObservationSourceOptions options,
        perception::ObservationIngress::LocalizationLookup localization,
        std::chrono::milliseconds retirement_timeout = std::chrono::seconds(5));
    /** @brief Read native input lifecycle independently of the Session
     * snapshot. */
    ExplorationObservationSourceSnapshot GetObservationSourceSnapshot() const;
    /** @brief Retain the most recently requested Reader cleanup result.
     * @return Invalid future until a Source has existed; readiness means actual
     * cleanup ended. After Shutdown and owner-thread joining, this handle
     * audits input retirement. It is not evidence that the robot stopped or
     * that map fusion completed.
     */
    std::shared_future<bool> ObservationSourceShutdownCompletion() const;
    /**
     * @brief Obtain this generation's asynchronous navigation services.
     * @pre Called by create_session on the task owner thread.
     * @throws std::logic_error For another thread, superseded factory or
     * missing cooperative native services. Legacy Submit is insufficient.
     */
    exploration::ExplorationServices NavigationServices(
        const std::string& frame_id,
        const std::string& planner_id = "vista_planner",
        const std::string& controller_id = "",
        const std::string& navigation_tree = "") const;

    /// Queue an owned map snapshot; missing source evidence is not fabricated.
    bool PushMap(exploration::MapReadView map);
    /// Queue a pose using the configured clock and localization support mask.
    bool PushPose(exploration::Pose2d pose, double stamp, std::uint64_t epoch,
                  perception::SensorMask localization_support);
    /// Queue a real sensor observation for the current task generation.
    bool PushObservation(perception::SensorObservation observation, double now);
    /** @brief Queue a whole localized sensor packet under one task generation.
     * @details Before START only the current pose can seed readiness; pre-start
     *          observations never become the new generation's coverage.
     */
    bool PushSensorPacket(perception::ObservationPacket packet, double now);
    /** @brief Capture the destination before asynchronous decoding.
     * @return No identity when the worker is stopped or dependencies are
     * absent.
     */
    std::optional<ExplorationSensorInputIdentity> SensorInputIdentity() const;
    /** @brief Admit or record loss atomically for the captured generation.
     * Validation is outside the inbox lock; replacement START cannot retarget
     * it.
     */
    bool PushSensorPacket(ExplorationSensorInputIdentity identity,
                          perception::ObservationPacket packet, double now);
    /** @brief Admit native input with its captured platform operation and
     * configuration revision.
     * @details A stale revision is rejected at admission and owner delivery.
     * Processor execution remains off sensor callbacks and bound to the source
     * generation; ordinary injected packet APIs retain their existing
     * semantics.
     */
    bool PushNativeSensorPacket(ExplorationSensorInputIdentity identity,
                                perception::ObservationPacket packet,
                                double now,
                                ExplorationPacketProcessor processor,
                                std::uint64_t input_revision);
    /** @brief Record source rejection in a bounded, generation-scoped loss
     * slot. Loss notification remains available when the ordinary inbox is
     * full.
     */
    bool ReportSensorLoss(perception::SensorKind sensor,
                          std::uint64_t sequence);
    /** @brief Reject stale source callbacks without changing the newer task. */
    bool ReportSensorLoss(ExplorationSensorInputIdentity identity,
                          perception::SensorKind sensor,
                          std::uint64_t sequence);
    /** @brief Bind source callbacks to a weak task and a fixed generation.
     * @return No binding for a null, stopped or unconfigured task.
     * Old callbacks fail after replacement START. Pre-start bindings expire at
     * the first START and must be replaced for coverage input.
     */
    static std::optional<ExplorationSensorInputBinding> BindSensorInput(
        const std::shared_ptr<ExplorationTask>& task);
    /// Queue an explicit coverage requirement for the current task generation.
    bool PushCoverageUnit(exploration::CoverageUnit unit);
    /// Read a consistent algorithm snapshot; absent until a session is started.
    std::optional<exploration::ExplorationSnapshot> GetExplorationSnapshot()
        const;
    /// Publish goal identity only after a START has been accepted.
    bool SubmitGoal(const proto::ExplorationGoal& goal) override;
    /// Stop input admission, cancel owned work, then join without lifecycle
    /// lock.
    void Shutdown() override;
    /// Queue cancellation of this exploration generation.
    bool Cancel() override;
    /// Queue pause; queued paused time cannot satisfy observation requirements.
    bool Pause() override;
    /// Resume the original stage after inputs have been revalidated.
    bool Resume() override;

protected:
    /// Start the owner loop using the native scheduler period.
    bool OnInitialize(const proto::TaskServerOptions& options) override;
    /// Validate and enqueue the native command without joining a worker.
    bool OnGoal(const proto::ExplorationGoal& goal) override;
    /// Fill feedback from one state snapshot with mutex_ already held.
    void FillFeedback(proto::ExplorationFeedback* feedback) const override;
    /// Fill terminal result with mutex_ already held; tree status is ignored.
    void FillResult(proto::ExplorationResult* result) const override;

private:
    /// Inbox, thread, immutable dependencies and owner-only runtime state.
    class Impl;
    /// Implementation lifetime ends only after the owner has stopped.
    std::unique_ptr<Impl> impl_;
};

}  // namespace task
}  // namespace autonomy
