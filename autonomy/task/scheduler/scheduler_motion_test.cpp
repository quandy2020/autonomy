/* Copyright 2026 The Openbot Authors. */
/**
 * @file scheduler_motion_test.cpp
 * @brief Actual Scheduler, NavigationTask, ExplorationTask and BT ownership checks.
 * @details Only RPC availability/cancel replies and sensor inputs are controlled.
 * These checks establish software ownership and cleanup, not chassis stopping.
 */
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include "autonomy/task/behavior_tree/bt_node_registry.hpp"
#include "autonomy/task/common/motion_admission.hpp"
#include "autonomy/task/exploration/exploration.hpp"
#include "autonomy/task/exploration/exploration_test_support.hpp"
#include "autonomy/task/navigation/navigation.hpp"
#include "autonomy/task/scheduler/scheduler.hpp"

namespace {
namespace t = autonomy::task;
namespace e = autonomy::task::exploration;
namespace p = autonomy::perception;
namespace v = autonomy::planning::planner::vista;
namespace n = autonomy::navigation;
namespace proto = autonomy::task::proto;
using Type = t::TaskScheduler::RobotTaskType;
using namespace std::chrono_literals;
using exploration_test::Require;

/** @brief Wait for actual observed effects with a bounded test deadline. */
template<class Predicate> void Until(Predicate predicate, const char* message) {
    const auto end = std::chrono::steady_clock::now() + 3s;
    while (!predicate()) {
        Require(std::chrono::steady_clock::now() < end, message);
        std::this_thread::sleep_for(1ms);
    }
}

/** @brief Observe real native tree starts and immutable dispatch gates. */
struct Evidence {
    std::atomic<unsigned> starts{0};  ///< Committed native invocation starts.
    std::atomic<bool> done{false};  ///< Controlled terminal tick result.
    std::mutex mutex;  ///< Gate records are published by native blackboard setup.
    std::vector<std::shared_ptr<t::common::MotionDispatchGate>> gates;  ///< Retained old permissions.
    /** @return The latest gate without retaining a task lifecycle mutex. */
    std::shared_ptr<t::common::MotionDispatchGate> Latest() {
        std::lock_guard<std::mutex> lock(mutex);
        return gates.empty() ? nullptr : gates.back();
    }
};

/** @brief Real stateful BT node, driven and halted by production NavigationTask. */
class OwnerProbe final : public BT::StatefulActionNode {
public:
    OwnerProbe(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config), evidence_(config.blackboard->get<std::shared_ptr<Evidence>>("owner_probe")) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus onStart() override { ++evidence_->starts; return BT::NodeStatus::RUNNING; }
    BT::NodeStatus onRunning() override { return evidence_->done ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING; }
    void onHalted() override {}
private:
    std::shared_ptr<Evidence> evidence_;  ///< Owned effects outlive every native callback.
};
/** @brief Add fixture effects through the real factory registration mechanism. */
void RegisterProbe(BT::BehaviorTreeFactory& factory) { factory.registerNodeType<OwnerProbe>("SchedulerOwnerProbe"); }

/** @brief Unique XML file used as the actual native navigation default. */
class Xml {
public:
    Xml() {
        static std::atomic<unsigned> count{0};
        file = std::filesystem::temp_directory_path() / ("autonomy-scheduler-" +
            std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + "-" +
            std::to_string(++count) + ".xml");
        std::ofstream out(file);
        out << "<root BTCPP_format=\"4\" main_tree_to_execute=\"Test\"><BehaviorTree ID=\"Test\">"
               "<SchedulerOwnerProbe/></BehaviorTree></root>";
        Require(static_cast<bool>(out), "native XML written");
    }
    ~Xml() { std::error_code error; std::filesystem::remove(file, error); }
    std::filesystem::path file;  ///< Fixture file persists until all native workers stop.
};

/** @brief Keep real admission/runner/gates, replacing only external RPC seams. */
class Navigation final : public t::NavigationTask {
public:
    ~Navigation() override { fail_cancel = false; try { Shutdown(); } catch (...) {} }
    std::shared_ptr<Evidence> evidence = std::make_shared<Evidence>();  ///< Native node effects.
    std::atomic<bool> ready{true}, fail_cancel{false}, fail_readiness{false};  ///< External dependency responses.
    mutable std::atomic<unsigned> probes{0};  ///< Actual readiness entries.
    std::function<void()> readiness_hook;  ///< Controlled synchronous availability callback.
    std::function<void()> cancel_hook;  ///< Controlled synchronous cancel callback reentry.
    std::atomic<unsigned> cancellations{0};  ///< Calls through the real cancellation path.
protected:
    bool EnsureNavigationClient() override { return true; }
    bool NavigationServersReady() const override {
        ++probes;
        if (readiness_hook) readiness_hook();
        if (fail_readiness) throw std::runtime_error("controlled readiness exception");
        return ready;
    }
    std::chrono::milliseconds NavigationReadyTimeout() const override { return 400ms; }
    void CancelRemoteNavigation() override {
        ++cancellations;
        if (cancel_hook) cancel_hook();
        if (fail_cancel) throw std::runtime_error("controlled cancel exception");
    }
    void PopulateBlackboard(const BT::Blackboard::Ptr& board) override {
        NavigationTask::PopulateBlackboard(board);
        board->set("owner_probe", evidence);
        std::lock_guard<std::mutex> lock(evidence->mutex);
        evidence->gates.push_back(board->get<std::shared_ptr<t::common::MotionDispatchGate>>(
            t::common::kMotionDispatchGateBlackboardKey));
    }
};

/** @brief Preserve actual automsgs Goal payloads and stable geometry. */
proto::NavigationGoal Goal(const std::string& id) {
    proto::NavigationGoal goal;
    goal.set_command(proto::NAV_CMD_START);
    goal.mutable_header()->set_task_id(id);
    auto* pose = goal.add_goals();
    pose->mutable_header()->set_frame_id("map");
    pose->mutable_pose()->mutable_position()->set_x(2);
    pose->mutable_pose()->mutable_orientation()->set_w(1);
    return goal;
}
/** @brief Native server options use a real XML default and short reaper period. */
proto::TaskServerOptions Options(const Xml& xml, bool exclusive = true, bool preempt = true) {
    proto::TaskServerOptions options;
    options.mutable_scheduler()->set_exclusive_navigation_tasks(exclusive);
    options.mutable_scheduler()->set_allow_preemption(preempt);
    options.mutable_scheduler()->set_feedback_period_ms(2);
    options.mutable_behavior_trees()->mutable_navigation()->set_default_tree_file(xml.file.string());
    return options;
}

/** @brief Legacy task fixture exposes cancel failure and reentrant getter effects. */
class Legacy final : public t::TaskInterface {
public:
    explicit Legacy(Type type) : type_(type) {}
    Type GetTaskType() const override { if (type_hook) type_hook(); return type_; }
    t::TaskLifecycle GetLifecycle() const override {
        const auto snapshot = state.load();
        if (lifecycle_hook) lifecycle_hook();
        return snapshot;
    }
    bool Initialize(const proto::TaskServerOptions&) override { return true; }
    void Shutdown() override { state = t::TaskLifecycle::kIdle; }
    bool Cancel() override {
        ++cancels;
        if (cancel_hook) cancel_hook();
        if (throw_cancel) throw std::runtime_error("legacy cancel threw");
        if (!accept_cancel) return false;
        if (finish_cancel) state = t::TaskLifecycle::kCanceled;
        return true;
    }
    bool Pause() override { state = t::TaskLifecycle::kPaused; return true; }
    bool Resume() override { state = t::TaskLifecycle::kRunning; return true; }
    proto::ActiveTaskSnapshot GetSnapshot() const override { proto::ActiveTaskSnapshot out; out.set_task_type(type_); return out; }
    std::atomic<t::TaskLifecycle> state{t::TaskLifecycle::kIdle};  ///< Software lifecycle only.
    std::atomic<unsigned> cancels{0};  ///< Observed cancel attempts.
    bool accept_cancel = true, throw_cancel = false, finish_cancel = true;  ///< Controlled legacy cancellation response.
    std::function<void()> type_hook, lifecycle_hook, cancel_hook;  ///< Reentry outside scheduler locks.
private:
    const Type type_;  ///< Immutable registered type.
};

/** @brief Real exploration session with current map/sensor evidence. */
struct Exploration {
    std::shared_ptr<t::ExplorationTask> task = std::make_shared<t::ExplorationTask>();  ///< Native task owner.
    e::MapReadView map;  ///< Immutable real route map snapshot.
    std::atomic<unsigned> created{0};  ///< Actual native session construction.
    Exploration() {
        n::GridMap grid(20, 20, .2); grid.Fill({n::CellState::kFree, 0});
        map = e::MapReader().Read(grid, 1, 0, 1);
        n::SafetyOptions safety; safety.robot_radius = .1; safety.horizontal_margin = .01;
        auto planner = std::make_shared<v::VistaRoutePlanner>(safety);
        auto paths = std::make_shared<e::VistaPathProvider>(planner);
        p::RigidTransform extrinsics;
        extrinsics.rotation = {0, 0, 1, -1, 0, 0, 0, -1, 0}; extrinsics.translation.z = .5;
        auto visibility = std::make_shared<e::CameraVisibilityModel>(p::CameraModel{80, 60, 80, 80, 40, 30, .1, 8}, extrinsics);
        auto evaluator = std::make_shared<e::ViewpointEvaluator>(
            std::vector<std::shared_ptr<const e::SensorVisibilityModel>>{visibility});
        auto adapter = std::make_shared<e::GoalPoseAdapter>(paths, evaluator, safety);
        t::ExplorationTaskBindings bindings;
        bindings.clock = [] { return 1.; };
        // This ownership fixture uses the actual BT driver and declares only
        // its controlled RGB-D execution mode. Default production policy must
        // not infer controller readiness from synthetic sensor observations.
        bindings.options.driver = e::ExplorationDriver::kBehaviorTree;
        bindings.options.validated_controller_modes = {p::Mask(p::SensorMode::kRgbdOnly)};
        bindings.options.allowed_region = e::Bounds2d{0, 0, 4, 4};
        bindings.options.map_max_age = bindings.options.pose_max_age = 10;
        bindings.options.health.maximum_age = 10;
        bindings.options.health.recovery_hold = 0;
        bindings.options.require_return = bindings.options.require_save = false;
        bindings.options.maximum_sensor_wait = .2;
        const std::weak_ptr<t::ExplorationTask> weak = task;
        bindings.create_session = [this, weak, planner, paths, adapter](const auto&, const auto& options) {
            const auto owner = weak.lock();
            if (!owner) throw std::runtime_error("factory owner expired");
            auto services = owner->NavigationServices("map");
            services.update_route_map = [planner](const auto& view) { planner->Update(view); };
            services.navigation_still_valid = [](const auto&, const auto&, const auto&) { return true; };
            ++created;
            return std::make_shared<e::ExplorationSession>(
                std::make_shared<exploration_test::OneGoal>(std::vector<double>{0}), adapter, paths, services, options);
        };
        Require(task->ConfigureSession(bindings), "native exploration configured");
    }
    ~Exploration() { task->Shutdown(); }
    /** @brief Feed real typed inputs; no task/session is ticked by this producer. */
    void Start(const std::string& id) {
        proto::ExplorationGoal goal; goal.set_command(proto::EXPLORATION_CMD_START);
        goal.mutable_header()->set_task_id(id);
        Require(task->SubmitGoal(goal), "exploration root accepted");
        Require(task->PushMap(map), "seed native map");
        Require(task->PushPose({1, 1, 0}, 1, 1, p::Mask(p::SensorKind::kRgbd)), "seed native pose");
        e::CoverageUnit unit; unit.id = {10, 5, 2, 0, e::CoverageKind::kSurface};
        unit.center = {2, 1, .5}; unit.normal = {-1, 0, 0}; unit.required_quality = .5;
        Require(task->PushCoverageUnit(unit), "seed coverage obligation");
        p::SensorObservation observation;
        observation.sensor = p::SensorKind::kRgbd; observation.sequence = 1; observation.map_epoch = 1;
        observation.stamp = 1; observation.valid_until = 11; observation.valid_fraction = 1; observation.localization_valid = true;
        observation.rays.push_back({{1, 1, .5}, {0, 1, 0}, .2, true, 1, 1});
        Require(task->PushObservation(std::move(observation), 1), "seed RGB-D observation");
    }
};

/** @brief Completion is exactly once; denied parent gates cannot revive. */
void AdmissionAndParentGate() {
    std::atomic<bool> live{true};
    std::atomic<unsigned> accepted{0}, rejected{0};
    t::common::MotionAdmission::Permit retained;
    {
        auto admission = std::make_shared<t::common::MotionAdmission>([&] { return live.load(); },
            [&](bool ok) { ok ? ++accepted : ++rejected; });
        retained = admission->PermitQuery();
        admission->Complete(true); admission->Complete(false);
    }
    Require(accepted == 1 && rejected == 0 && retained(), "accepted handle completes once and retains permission");
    { t::common::MotionAdmission admission([&] { return live.load(); }, [&](bool ok) { ok ? ++accepted : ++rejected; }); }
    Require(rejected == 1, "uncommitted destructor rejects exactly once");
    e::MotionGate gate(retained); gate.SetEnabled(true);
    const auto lease = gate.Lease(); Require(lease(), "parent lease starts live");
    live = false; Require(!lease() && !gate.Enabled(), "parent loss retires local permission");
    live = true; gate.SetEnabled(true); Require(!lease() && !gate.Enabled(), "resume cannot revive denied parent");
    e::MotionGate exceptional([]() -> bool { throw std::runtime_error("permit"); });
    exceptional.SetEnabled(true); Require(!exceptional.Lease()(), "exception fails closed");
}

/** @brief Both registration orders initialize before exposing a dispatchable task. */
void RegistrationAndReservations() {
    Xml xml; t::TaskScheduler scheduler;
    auto nav = std::make_shared<Navigation>();
    scheduler.RegisterTask(nav);
    Require(!scheduler.GetTask(Type::ROBOT_TASK_NAVIGATION), "uninitialized registration hidden");
    Require(scheduler.Initialize(Options(xml)), "registered tasks initialized");
    Require(scheduler.GetTask(Type::ROBOT_TASK_NAVIGATION) == nav, "ready registration visible");
    Require(scheduler.Start(), "real reaper starts");
    Require(scheduler.RequestActivation(nav), "preliminary reservation acquired");
    auto unrelated = std::make_shared<Legacy>(Type::ROBOT_TASK_TELEOP);
    scheduler.RegisterTask(unrelated);
    // Reaper must retain an idle RequestActivation reservation until submission.
    Require(nav->SubmitGoal(Goal("reserved")), "reservation consumed by domain admission");
    Until([&] { return nav->evidence->starts == 1; }, "reserved native node starts");
    auto unregistered = std::make_shared<Legacy>(Type::ROBOT_TASK_FOLLOW);
    Require(!scheduler.RequestActivation(unregistered) && nav->IsActive(), "unregistered request never preempts current owner");
    auto duplicate = std::make_shared<Navigation>(); scheduler.RegisterTask(duplicate);
    Require(scheduler.GetTask(Type::ROBOT_TASK_NAVIGATION) == nav, "duplicate cannot replace live plugin");
    nav->evidence->done = true;
    Until([&] { return nav->MotionDrained(); }, "native completion drains");
    scheduler.ReleaseActivation(nav);
    unrelated->state = t::TaskLifecycle::kRunning;
    Require(scheduler.RequestActivation(unrelated), "idle native owner released");
    Require(nav->SubmitGoal(Goal("active-domain")), "ordinary domain takes owner without RequestActivation");
    Require(unrelated->cancels == 1 && !unrelated->IsActive(), "active dispatch still uses scheduler admission");
    scheduler.Stop(); Require(!scheduler.IsRunning(), "real reaper stops");
    Require(scheduler.Start(), "reaper restarts with new run epoch");
    scheduler.Shutdown(); Require(!scheduler.IsRunning(), "shutdown joins reaper");
}

/** @brief Cancel failure and exceptions retain the old ownership barrier. */
void CancellationFailures() {
    for (const bool throwing : {false, true}) {
        Xml xml; t::TaskScheduler scheduler; Require(scheduler.Initialize(Options(xml)), "scheduler ready");
        auto old = std::make_shared<Legacy>(Type::ROBOT_TASK_TELEOP);
        auto nav = std::make_shared<Navigation>();
        scheduler.RegisterTask(old); scheduler.RegisterTask(nav);
        Require(scheduler.RequestActivation(old), "legacy reservation accepted");
        old->state = t::TaskLifecycle::kRunning; old->accept_cancel = false; old->throw_cancel = throwing;
        auto invalid = Goal("invalid"); invalid.clear_goals();
        Require(!nav->SubmitGoal(invalid) && old->cancels == 0 && old->IsActive(),
                "geometry preflight rejects before touching another owner");
        old->cancel_hook = [&] {
            Require(scheduler.GetTask(Type::ROBOT_TASK_NAVIGATION) == nav, "cancel callback can look up registry");
            Require(!scheduler.RequestActivation(nav), "transition rejects competing reentrant activation");
        };
        Require(!nav->SubmitGoal(Goal("denied")) && nav->evidence->starts == 0,
                "failed cancellation prevents new native invocation");
        Require(old->IsActive(), "old plugin remains active on cancel failure");
        old->accept_cancel = true; old->throw_cancel = false;
        Require(nav->SubmitGoal(Goal("after-confirmed-cancel")), "retry after confirmed cleanup accepted");
        Require(!old->IsActive(), "previous owner cleaned before retry grants");
        scheduler.Shutdown();
    }
}

/** @brief A stale outside-lock drain snapshot cannot release a newly reserved owner. */
void ReleaseSnapshotRace() {
    Xml xml; t::TaskScheduler scheduler; Require(scheduler.Initialize(Options(xml, true, false)), "scheduler ready");
    auto owner = std::make_shared<Legacy>(Type::ROBOT_TASK_TELEOP);
    auto nav = std::make_shared<Navigation>(); scheduler.RegisterTask(owner); scheduler.RegisterTask(nav);
    Require(scheduler.RequestActivation(owner), "initial legacy owner reserved");
    owner->state = t::TaskLifecycle::kCanceled;
    std::promise<void> entered, release;
    const auto released = release.get_future().share();
    std::atomic<unsigned> queried{0};
    owner->lifecycle_hook = [&] {
        if (++queried == 2) {
            entered.set_value();
            Require(released.wait_for(3s) == std::future_status::ready, "release snapshot barrier timed out");
        }
    };
    auto entered_future = entered.get_future();
    auto reclaim = std::async(std::launch::async, [&] { scheduler.ReleaseActivation(owner); });
    const bool observed = entered_future.wait_for(3s) == std::future_status::ready;
    if (!observed) { release.set_value(); reclaim.get(); Require(false, "real domain drain getter entered"); }
    // These calls would deadlock if a task getter ran under scheduler mutex.
    owner->type_hook = [&] { Require(scheduler.GetTask(Type::ROBOT_TASK_TELEOP) == owner, "type query may reenter registry"); };
    const bool renewed = scheduler.RequestActivation(owner);
    owner->state = t::TaskLifecycle::kRunning;
    release.set_value(); reclaim.get();
    Require(renewed && !nav->SubmitGoal(Goal("stale-reclaim-denied")), "stale drain snapshot preserves new generation reservation");
    owner->type_hook = {}; owner->lifecycle_hook = {};
    scheduler.Shutdown();
}

/** @brief Legacy terminal sessions are reclaimed even without native admission hooks. */
void LegacyReaper() {
    Xml xml; t::TaskScheduler scheduler; Require(scheduler.Initialize(Options(xml, true, false)), "scheduler ready");
    auto old = std::make_shared<Legacy>(Type::ROBOT_TASK_TELEOP); auto nav = std::make_shared<Navigation>();
    scheduler.RegisterTask(old); scheduler.RegisterTask(nav);
    Require(scheduler.RequestActivation(old), "legacy root reserved"); old->state = t::TaskLifecycle::kRunning;
    Require(scheduler.Start(), "native reaper started");
    old->state = t::TaskLifecycle::kSucceeded;
    Until([&] { return nav->SubmitGoal(Goal("after-legacy-reap")); }, "real reaper releases terminal legacy reservation");
    Require(old->cancels == 0, "completed legacy session does not require preemption");
    scheduler.Shutdown();
}

/** @brief Shutdown callback reentry cannot recursively dispose the same native task. */
void ShutdownReentryAndWorkerEpochs() {
    Xml xml; t::TaskScheduler scheduler; Require(scheduler.Initialize(Options(xml)), "scheduler ready");
    auto nav = std::make_shared<Navigation>(); scheduler.RegisterTask(nav);
    Require(nav->SubmitGoal(Goal("shutdown-reentry")), "native goal starts");
    nav->cancel_hook = [&] { scheduler.Shutdown(); };
    scheduler.Shutdown(); Require(nav->MotionDrained(), "reentrant shutdown drains native worker once");
    nav->cancel_hook = {};
    Require(scheduler.Initialize(Options(xml)), "new empty scheduler epoch ready");
    std::vector<std::thread> threads;
    for (unsigned i = 0; i < 4; ++i) threads.emplace_back([&] {
        for (unsigned round = 0; round < 20; ++round) { scheduler.Start(); scheduler.Stop(); }
    });
    for (auto& thread : threads) thread.join();
    scheduler.Stop(); Require(!scheduler.IsRunning(), "concurrent start/stop leaves no enabled reaper");
    scheduler.Shutdown();
}

/** @brief Native cancellation ambiguity cannot be erased by terminal lifecycle. */
void NativeDrainFailure() {
    Xml xml; t::TaskScheduler scheduler; Require(scheduler.Initialize(Options(xml)), "scheduler ready");
    auto nav = std::make_shared<Navigation>(); auto next = std::make_shared<Legacy>(Type::ROBOT_TASK_TELEOP);
    scheduler.RegisterTask(nav); scheduler.RegisterTask(next);
    Require(nav->SubmitGoal(Goal("ambiguous")), "native goal accepted");
    nav->fail_cancel = true;
    Require(!scheduler.RequestActivation(next) && !nav->MotionDrained(), "native failed drain blocks takeover");
    scheduler.ReleaseActivation(nav);
    Require(!scheduler.RequestActivation(next), "terminal lifecycle does not free ambiguous owner");
    nav->fail_cancel = false;
    scheduler.Shutdown();
}

/** @brief Terminal publication cannot release a native worker still executing cleanup. */
void NativeTerminalDrain() {
    Xml xml; t::TaskScheduler scheduler; Require(scheduler.Initialize(Options(xml)), "scheduler ready");
    auto nav = std::make_shared<Navigation>(); auto next = std::make_shared<Legacy>(Type::ROBOT_TASK_TELEOP);
    scheduler.RegisterTask(nav); scheduler.RegisterTask(next);
    Require(nav->SubmitGoal(Goal("terminal-cleanup")), "native goal accepted");
    Until([&] { return nav->evidence->starts == 1; }, "native worker entered real node");
    std::promise<void> entered, release;
    auto entered_future = entered.get_future();
    const auto released = release.get_future().share();
    std::atomic<unsigned> cancellations{0};
    nav->cancel_hook = [&] {
        if (++cancellations == 1) {
            entered.set_value();
            Require(released.wait_for(3s) == std::future_status::ready, "native terminal cleanup barrier timed out");
        }
    };
    nav->evidence->done = true;
    const bool observed = entered_future.wait_for(3s) == std::future_status::ready;
    if (!observed) { release.set_value(); Require(false, "native terminal callback entered"); }
    const bool terminal = !nav->IsActive();
    const bool drained_early = nav->MotionDrained();
    auto takeover = std::async(std::launch::async, [&] { return scheduler.RequestActivation(next); });
    const bool waited = takeover.wait_for(20ms) == std::future_status::timeout;
    release.set_value();
    const bool accepted = takeover.get();
    nav->cancel_hook = {};
    Require(terminal && !drained_early && waited && accepted && nav->MotionDrained(),
            "terminal native callback keeps ownership until actual worker cleanup finishes");
    scheduler.Shutdown();
}

/** @brief Pending native readiness is revocable before any committed tree start. */
void PendingAdmission() {
    Xml xml; t::TaskScheduler scheduler; Require(scheduler.Initialize(Options(xml)), "scheduler ready");
    auto nav = std::make_shared<Navigation>(); auto next = std::make_shared<Legacy>(Type::ROBOT_TASK_TELEOP);
    scheduler.RegisterTask(nav); scheduler.RegisterTask(next); nav->ready = false;
    auto pending = std::async(std::launch::async, [&] { return nav->SubmitGoal(Goal("pending")); });
    Until([&] { return nav->probes > 0; }, "real readiness entered");
    Require(scheduler.RequestActivation(next), "pending canceled and its domain handoff drained");
    Require(!pending.get() && nav->MotionDrained() && nav->evidence->starts == 0, "pending permission cannot start late");
    scheduler.ReleaseActivation(next);
    nav->ready = true; Require(nav->SubmitGoal(Goal("fresh")), "new native generation accepted");
    // SubmitGoal starts the real worker asynchronously. Its blackboard and
    // retained gate are published before OwnerProbe::onStart, not at admission.
    Until([&] { return nav->evidence->starts == 1; }, "fresh native worker publishes its gate");
    const auto old = nav->evidence->Latest();
    Require(static_cast<bool>(old), "fresh native gate is retained before shutdown");
    scheduler.Shutdown(); Require(!old->PermitValid(), "shutdown invalidates retained permit");
    Require(scheduler.Initialize(Options(xml)), "clean scheduler can initialize again");
    scheduler.RegisterTask(nav); Require(nav->SubmitGoal(Goal("after-reinit")), "new registration gets new generation");
    Require(!old->PermitValid(), "initialize cannot renew old permit");
    scheduler.Shutdown();
}

/** @brief Real exploration delegates navigation, pauses, resumes and yields ownership. */
void ParentChild(bool nav_first, bool exclusive, bool preempt) {
    Xml xml; Exploration exploration;
    t::TaskScheduler scheduler; Require(scheduler.Initialize(Options(xml, exclusive, preempt)), "scheduler ready");
    auto nav = std::make_shared<Navigation>();
    if (nav_first) { scheduler.RegisterTask(nav); scheduler.RegisterTask(exploration.task); }
    else { scheduler.RegisterTask(exploration.task); scheduler.RegisterTask(nav); }
    exploration.Start("exploration-root");
    Until([&] { return nav->evidence->starts >= 1; }, "actual exploration BT delegated native navigation");
    Require(exploration.task->IsActive() && nav->IsActive(), "parent and delegated child remain active together");
    const auto child = nav->evidence->Latest(); const auto child_id = nav->GetSnapshot().task_id();
    Require(child->PermitValid(), "child retains parent ownership");
    Require(exploration.task->Pause(), "real parent pauses");
    Until([&] { return nav->MotionDrained(); }, "pause cancels actual native child");
    Require(!child->PermitValid(), "paused child permission is retired");
    Require(exploration.task->Resume(), "real parent resumes");
    Until([&] { return nav->evidence->starts >= 2; }, "resume delegates new native identity");
    Require(nav->GetSnapshot().task_id() != child_id && !child->PermitValid(), "old child never revives after resume");
    if (!preempt) {
        Require(!nav->SubmitGoal(Goal("ordinary-denied")) && exploration.task->IsActive(),
                "preemption policy rejects active ordinary navigation takeover");
    } else {
        Require(nav->SubmitGoal(Goal("ordinary")), "active ordinary dispatch preempts exploration parent");
        Require(exploration.task->MotionDrained() && !exploration.task->IsActive(), "parent fully disposes before ordinary child engine reuse");
        Require(nav->GetSnapshot().task_id() == "ordinary" && !child->PermitValid(), "ordinary root has fresh identity and permission");
        proto::NavigationResult stale;
        Require(!nav->GetResultForTask(child_id, &stale), "old child result cannot masquerade as new root");
    }
    scheduler.Shutdown();
    Require(exploration.task->MotionDrained() && nav->MotionDrained(), "shutdown drains both native domains");
}

}  // namespace

int main() {
    std::cout << std::unitbuf;
    try {
        t::AddBtNodeRegistrar(&RegisterProbe);
        AdmissionAndParentGate(); std::cout << "PASS admission_parent_gate\n";
        RegistrationAndReservations(); std::cout << "PASS registration_reservations_active_domain\n";
        CancellationFailures(); std::cout << "PASS cancellation_failures_reentry\n";
        ReleaseSnapshotRace(); std::cout << "PASS stale_release_snapshot\n";
        LegacyReaper(); std::cout << "PASS legacy_terminal_reaper\n";
        ShutdownReentryAndWorkerEpochs(); std::cout << "PASS shutdown_reentry_worker_epochs\n";
        NativeDrainFailure(); std::cout << "PASS native_cleanup_barrier\n";
        NativeTerminalDrain(); std::cout << "PASS native_terminal_callback_drain\n";
        PendingAdmission(); std::cout << "PASS pending_shutdown_reinitialize\n";
        ParentChild(true, true, true); std::cout << "PASS navigation_first_parent_child\n";
        ParentChild(false, true, true); std::cout << "PASS exploration_first_parent_child\n";
        ParentChild(false, false, true); std::cout << "PASS shared_engine_without_global_exclusivity\n";
        ParentChild(true, true, false); std::cout << "PASS no_preemption_parent_child\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << "FAIL " << error.what() << '\n'; return 1; }
}
