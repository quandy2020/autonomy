/* Copyright 2026 The Openbot Authors. */
/**
 * @file exploration.hpp
 * @brief Native exploration task owning the session and behavior-tree loop.
 */
#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "autonomy/common/macros.hpp"
#include "autonomy/task/common/typed_task.hpp"
#include "autonomy/task/exploration/exploration_session.hpp"
#include "autonomy/task/exploration/integration/navigation_dispatcher.hpp"
#include <automsgs/task/exploration.pb.h>
#include <automsgs/task/mapping.pb.h>
#include <automsgs/task/navigation.pb.h>

namespace autolink { class Node; }

namespace autonomy {
namespace task {

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
        const exploration::ExplorationSessionOptions&)> create_session;
    /// Confirms an actual mapping session, rather than accepting a map name.
    std::function<bool(const std::string&)> prepare_mapping;
    /// Clock shared by observations, poses, map stamps and the owner loop.
    std::function<double()> clock;
    /// Initial immutable policy; per-goal map name/boundary are versioned.
    exploration::ExplorationSessionOptions options;
};

/**
 * @brief First-class exploration API with one Session/BT owner thread.
 *
 * Inputs and commands are queued; producer callbacks never tick the session.
 * Coverage, return and save outcomes determine terminal status. Legacy
 * waypoint/finished messages cannot submit navigation or finish this task.
 */
class ExplorationTask
    : public TypedTaskAppBase<proto::ExplorationGoal,
                              proto::ExplorationFeedback,
                              proto::ExplorationResult> {
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
    /// Atomic identity-aware terminal result read from the real navigation task.
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
    /// Retain mapping dispatcher; SAVE_MAP never maps to MAP_CMD_SWITCH.
    void SetSubmitMapping(SubmitMapping submit);
    /// Preserve old probe API; it is not used for authoritative completion.
    void SetNavigationProbe(IsNavigationActive active, GetNavigationResult result);
    /// Install identity-aware native result/cancel operations while idle.
    void SetVersionedNavigationProbe(GetVersionedNavigationResult result,
                                    CancelNavigationRequest cancel);
    /// Configure real session dependencies while idle or terminal.
    bool ConfigureSession(ExplorationTaskBindings bindings);
    /**
     * @brief Obtain this generation's asynchronous navigation services.
     * @pre Called by create_session on the task owner thread.
     * @throws std::logic_error For another thread, superseded factory or missing
     *         cooperative native services. Legacy Submit is insufficient.
     */
    exploration::ExplorationServices NavigationServices(
        const std::string& frame_id, const std::string& planner_id = "vista_planner") const;

    /// Queue an owned map snapshot; missing source evidence is not fabricated.
    bool PushMap(exploration::MapReadView map);
    /// Queue a pose using the configured clock and localization support mask.
    bool PushPose(exploration::Pose2d pose, double stamp, std::uint64_t epoch,
                  perception::SensorMask localization_support);
    /// Queue a real sensor observation for the current task generation.
    bool PushObservation(perception::SensorObservation observation, double now);
    /// Queue an explicit coverage requirement for the current task generation.
    bool PushCoverageUnit(exploration::CoverageUnit unit);
    /// Read a consistent algorithm snapshot; absent until a session is started.
    std::optional<exploration::ExplorationSnapshot> GetExplorationSnapshot() const;
    /// Publish goal identity only after a START has been accepted.
    bool SubmitGoal(const proto::ExplorationGoal& goal) override;
    /// Stop input admission, cancel owned work, then join without lifecycle lock.
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
