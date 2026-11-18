/* Copyright 2026 The Openbot Authors. */
/**
 * @file exploration_owner_test.cpp
 * @brief Native ExplorationTask ownership and cancellation integration tests.
 * @details Links the real TypedTask, Session, BT registry and Vista route core.
 * Only external navigation and session construction timing are controlled.
 */
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include "autonomy/task/exploration/exploration.hpp"
#include "autonomy/task/exploration/exploration_test_support.hpp"

namespace {
namespace e = autonomy::task::exploration;
namespace p = autonomy::perception;
namespace t = autonomy::task;
namespace proto = autonomy::task::proto;
namespace v = autonomy::planning::planner::vista;
namespace n = opt::navigation;
using exploration_test::Require;
using namespace std::chrono_literals;

/** @brief Deterministic rendezvous; no test relies on a guessed thread delay. */
class Rendezvous {
public:
    /// Mark the callback entered and block until the test releases it.
    void Enter() {
        std::unique_lock<std::mutex> lock(mutex_);
        entered_ = true;
        changed_.notify_all();
        if (!changed_.wait_for(lock, 3s, [this] { return released_; }))
            throw std::runtime_error("test callback release timed out");
    }
    /// Wait for the controlled callback to reach its blocking point.
    void WaitEntered() {
        std::unique_lock<std::mutex> lock(mutex_);
        Require(changed_.wait_for(lock, 3s, [this] { return entered_; }),
                "controlled callback was reached");
    }
    /// Release on both normal and exception cleanup paths.
    void Release() {
        std::lock_guard<std::mutex> lock(mutex_);
        released_ = true;
        changed_.notify_all();
    }
private:
    /// Protects this callback's state independently of task synchronization.
    std::mutex mutex_;
    /// Signals callback entry and test release.
    std::condition_variable changed_;
    /// Callback reached the controlled point.
    bool entered_ = false;
    /// The test permitted callback completion.
    bool released_ = false;
};

/** @brief Shared evidence remains valid until every native callback exits. */
struct Evidence {
    /// Finite acquisition/owner clock; advanced only explicitly by tests.
    std::atomic<double> now{1};
    /// Counts actual construction and completed destruction, not strategy work.
    std::atomic<int> created{0}, destroyed{0};
    /// Counts real mapping preparation and native cancellation calls.
    std::atomic<int> prepared{0}, canceled{0};
    /// Counts native goal admissions without conflating them with arrival.
    std::atomic<int> submitted{0};
    /// Session construction and disposal must share the native task owner.
    std::atomic<bool> wrong_owner{false};
    /// Protects nonowning session inspection against concurrent owner disposal.
    mutable std::mutex sessions_mutex;
    /// Registry contains live sessions without extending their ownership.
    std::vector<e::ExplorationSession*> sessions;
    /// Started sessions already disposed; their identity remains evidence.
    int released_started = 0;
    /** @brief Count real Start publications, independent of strategy Reset. */
    int StartedCount() const {
        std::lock_guard<std::mutex> lock(sessions_mutex);
        int count = released_started;
        for (const auto* session : sessions)
            if (session->Snapshot().session_id != 0) ++count;
        return count;
    }
    /// Optional rendezvous at session-factory return, clock and native submit.
    std::shared_ptr<Rendezvous> factory, clock, submit;
    /// Only the first factory/clock call is blocked during preemption tests.
    std::atomic<int> factory_calls{0}, clock_calls{0};
    /// Arm the next clock invocation while a deterministic callback is blocked.
    std::atomic<bool> block_next_clock{false};
    /// Mapping backend can explicitly reject initialization.
    bool mapping_ok = true;
    /// Optional native callback reentry requests task shutdown.
    std::function<void()> native_reentry;
};

/** @brief Poll a real asynchronous result with a finite test deadline. */
template <typename Predicate>
void Await(Predicate predicate, const char* reason) {
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (!predicate()) {
        Require(std::chrono::steady_clock::now() < deadline, reason);
        std::this_thread::sleep_for(1ms);
    }
}

/** @brief Fresh, unstarted native Session factory with real route geometry. */
class Harness {
public:
    /// Install cooperative, versioned native services before starting a task.
    explicit Harness(std::shared_ptr<Evidence> evidence = std::make_shared<Evidence>(),
                     bool cooperative = true)
        : evidence(std::move(evidence)) {
        grid.Fill({n::CellState::kFree, 0});
        map = reader.Read(grid, 1, 0, 1);
        n::SafetyOptions safety;
        safety.robot_radius = .1;
        safety.horizontal_margin = .01;
        planner = std::make_shared<v::VistaRoutePlanner>(safety);
        paths = std::make_shared<e::VistaPathProvider>(planner);
        p::RigidTransform extrinsics;
        extrinsics.rotation = {0, 0, 1, -1, 0, 0, 0, -1, 0};
        extrinsics.translation.z = .5;
        auto visibility = std::make_shared<e::CameraVisibilityModel>(
            p::CameraModel{80, 60, 80, 80, 40, 30, .1, 8}, extrinsics);
        auto evaluator = std::make_shared<e::ViewpointEvaluator>(
            std::vector<std::shared_ptr<const e::SensorVisibilityModel>>{visibility});
        adapter = std::make_shared<e::GoalPoseAdapter>(paths, evaluator, safety);
        if (cooperative) task.SetGuardedSubmitNavigation(
            [state = this->evidence](const auto&, const auto& permit) {
                if (!permit()) return false;
                ++state->submitted;
                if (state->native_reentry) state->native_reentry();
                if (state->submit) state->submit->Enter();
                // Deliberately return a late acceptance after revocation.
                return true;
            });
        task.SetVersionedNavigationProbe(
            [](const auto&, auto*) { return false; },
            [state = this->evidence](const auto&) { ++state->canceled; return true; });
        bindings.clock = [state = this->evidence] {
            ++state->clock_calls;
            if (state->clock && state->block_next_clock.exchange(false)) state->clock->Enter();
            return state->now.load();
        };
        bindings.prepare_mapping = [state = this->evidence](const auto&) {
            ++state->prepared;
            return state->mapping_ok;
        };
        bindings.options.driver = e::ExplorationDriver::kBehaviorTree;
        bindings.options.validated_controller_modes = {p::Mask(p::SensorMode::kRgbdOnly)};
        bindings.options.allowed_region = e::Bounds2d{0, 0, 4, 4};
        bindings.options.map_max_age = bindings.options.pose_max_age = 10;
        bindings.options.health.maximum_age = 10;
        bindings.options.health.recovery_hold = 0;
        bindings.options.require_return = bindings.options.require_save = false;
        bindings.options.maximum_sensor_wait = .2;
        bindings.create_session = [this, state = this->evidence](const auto&, const auto& options) {
            auto services = task.NavigationServices("map");
            services.update_route_map = [routes = planner](const auto& view) { routes->Update(view); };
            services.navigation_still_valid = [](const auto&, const auto&, const auto&) { return true; };
            const auto owner = std::this_thread::get_id();
            auto candidate = std::shared_ptr<e::ExplorationSession>(
                new e::ExplorationSession(std::make_shared<exploration_test::OneGoal>(
                    std::vector<double>{0}), adapter, paths, services, options),
                [state, owner](e::ExplorationSession* session) {
                    {
                        std::lock_guard<std::mutex> lock(state->sessions_mutex);
                        if (session->Snapshot().session_id != 0) ++state->released_started;
                        const auto found = std::find(state->sessions.begin(), state->sessions.end(), session);
                        if (found != state->sessions.end()) state->sessions.erase(found);
                    }
                    if (owner != std::this_thread::get_id()) state->wrong_owner = true;
                    delete session;
                    ++state->destroyed;
                });
            {
                std::lock_guard<std::mutex> lock(state->sessions_mutex);
                state->sessions.push_back(candidate.get());
            }
            ++state->created;
            if (state->factory && state->factory_calls++ == 0) state->factory->Enter();
            return candidate;
        };
        Require(task.ConfigureSession(bindings), "native dependencies configured while idle");
        proto::TaskServerOptions options;
        options.mutable_scheduler()->set_feedback_period_ms(2);
        Require(task.Initialize(options), "native owner initialized");
    }
    /// Release controlled callbacks before joining native runtime cleanup.
    ~Harness() {
        if (evidence->factory) evidence->factory->Release();
        if (evidence->clock) evidence->clock->Release();
        if (evidence->submit) evidence->submit->Release();
        task.Shutdown();
    }
    /// Submit a distinct native task identity; rejections preserve prior ID.
    bool Start(const std::string& id, bool mapping = false) {
        proto::ExplorationGoal goal;
        goal.set_command(proto::EXPLORATION_CMD_START);
        goal.mutable_header()->set_task_id(id);
        goal.set_enable_mapping(mapping);
        return task.SubmitGoal(goal);
    }
    /// Deliver current immutable seed evidence without manufacturing coverage.
    void Seed() {
        map.source_stamp = evidence->now;
        Require(task.PushMap(map), "seed map admitted");
        Require(task.PushPose({1, 1, 0}, evidence->now, map.map_epoch,
                              p::Mask(p::SensorKind::kRgbd)), "seed pose admitted");
    }
    /// Admit a real requirement and an observation looking away from its surface.
    void Observe() {
        e::CoverageUnit unit;
        unit.id = {10, 5, 2, 0, e::CoverageKind::kSurface};
        unit.center = {2, 1, .5}; unit.normal = {-1, 0, 0}; unit.required_quality = .5;
        Require(task.PushCoverageUnit(unit), "coverage requirement admitted");
        p::SensorObservation observation;
        observation.sensor = p::SensorKind::kRgbd;
        observation.sequence = 1; observation.map_epoch = map.map_epoch;
        observation.stamp = evidence->now; observation.valid_until = observation.stamp + 10;
        observation.valid_fraction = 1; observation.localization_valid = true;
        observation.rays.push_back({{1, 1, .5}, {0, 1, 0}, .2, true, 1, observation.stamp});
        Require(task.PushObservation(std::move(observation), evidence->now),
                "real source observation admitted");
    }
    /// Lifecycle counters, backend timing and shared clock.
    const std::shared_ptr<Evidence> evidence;
    /// Native consumer geometry; no autonomy/map files or interfaces changed.
    n::GridMap grid{20, 20, .2};
    /// Real immutable snapshot adapter and its current view.
    e::MapReader reader;
    e::MapReadView map;
    /// Real Vista route and observation-goal adapters.
    std::shared_ptr<v::VistaRoutePlanner> planner;
    std::shared_ptr<e::VistaPathProvider> paths;
    std::shared_ptr<e::GoalPoseAdapter> adapter;
    /// Captured dependencies can be reconfigured only after runtime cleanup.
    t::ExplorationTaskBindings bindings;
    /// Constructed last and explicitly closed before dependencies are released.
    t::ExplorationTask task;
};

/** @brief Cancel during factory construction must prevent actual Session::Start. */
void FactoryCancellation() {
    auto state = std::make_shared<Evidence>(); state->factory = std::make_shared<Rendezvous>();
    Harness h(state); Require(h.Start("cancel-factory"), "START accepted"); h.Seed();
    state->factory->WaitEntered(); Require(h.task.Cancel(), "cancel accepted during factory");
    state->factory->Release();
    Await([&] { return h.task.GetLifecycle() == t::TaskLifecycle::kCanceled; }, "factory cancel reaches terminal state");
    Await([&] { return state->destroyed == 1; }, "unstarted candidate released");
    Require(state->StartedCount() == 0 && !state->wrong_owner, "canceled factory never starts and is disposed on owner");
}

/** @brief A paused factory candidate is retained and starts once after resume. */
void FactoryPauseResume() {
    auto state = std::make_shared<Evidence>(); state->factory = std::make_shared<Rendezvous>();
    Harness h(state); Require(h.Start("pause-factory"), "START accepted"); h.Seed();
    state->factory->WaitEntered(); Require(h.task.Pause(), "pause accepted during factory");
    state->factory->Release(); Require(state->StartedCount() == 0, "paused candidate is unstarted");
    Require(h.task.Resume(), "resume accepted");
    Await([&] { return state->StartedCount() == 1; }, "resume starts retained candidate");
    Require(state->created == 1, "pause does not recreate factory or retire its resumable services");
    Require(h.task.Cancel(), "cancel active runtime");
    Await([&] { return state->destroyed == 1; }, "active candidate cleaned up");
    Require(!state->wrong_owner, "construction and disposal share real owner");
}

/** @brief Pause before mapping initialization must not let Resume bypass it. */
void MappingPreparationAfterPause() {
    auto state = std::make_shared<Evidence>(); state->clock = std::make_shared<Rendezvous>();
    state->block_next_clock = true;
    Harness h(state); Require(h.Start("pause-mapping", true), "mapping START accepted");
    state->clock->WaitEntered(); Require(h.task.Pause(), "pause before prepare");
    h.Seed(); state->clock->Release();
    Await([&] { return state->clock_calls > 1; }, "owner processed paused initialization");
    Require(h.task.Resume(), "resume mapping initialization");
    Await([&] { return state->StartedCount() == 1; }, "mapping runtime eventually starts");
    Require(state->prepared == 1, "actual mapping preparation is required after resume");
}

/** @brief Failed initialization releases runtime admission before reconfiguration. */
void InitializationFailureCleanup() {
    auto state = std::make_shared<Evidence>(); state->mapping_ok = false;
    Harness h(state); Require(h.Start("mapping-refused", true), "mapping request admitted");
    Await([&] { return h.task.GetLifecycle() == t::TaskLifecycle::kFailed; }, "mapping refusal is explicit failure");
    Await([&] { return h.task.ConfigureSession(h.bindings); }, "failed preparation releases idle runtime");
    Require(state->StartedCount() == 0, "refused mapping never starts Session");
}

/** @brief Missing seed evidence times out and releases runtime ownership. */
void SeedTimeoutCleanup() {
    auto state = std::make_shared<Evidence>(); state->clock = std::make_shared<Rendezvous>();
    state->block_next_clock = true;
    Harness h(state); Require(h.Start("no-seed"), "START accepted without fabricated seed");
    state->clock->WaitEntered(); state->clock->Release();
    Await([&] { return state->clock_calls > 1; }, "initial clock captured before timeout advance");
    state->now = 2;
    Await([&] { return h.task.GetLifecycle() == t::TaskLifecycle::kFailed; }, "seed timeout fails explicitly");
    Await([&] { return h.task.ConfigureSession(h.bindings); }, "seed timeout releases runtime admission");
    Require(state->created == 0, "no Session constructed without seed evidence");
}

/** @brief A superseded factory cannot start or publish under a newer identity. */
void FactoryPreemption() {
    auto state = std::make_shared<Evidence>(); state->factory = std::make_shared<Rendezvous>();
    Harness h(state); Require(h.Start("old"), "old START accepted"); h.Seed();
    state->factory->WaitEntered(); Require(h.Start("new"), "replacement START accepted");
    h.Seed(); state->factory->Release();
    Await([&] { return state->StartedCount() == 1; }, "only replacement Session starts");
    Require(state->created == 2 && h.task.GetSnapshot().task_id() == "new", "replacement retains correct native identity");
    h.task.Shutdown(); Require(state->destroyed == 2 && !state->wrong_owner, "both generations disposed on owner");
}

/** @brief Cancel during final clock validation cannot start an old session. */
void FactoryClockCancellation() {
    auto state = std::make_shared<Evidence>();
    state->factory = std::make_shared<Rendezvous>();
    state->clock = std::make_shared<Rendezvous>();
    Harness h(state); Require(h.Start("cancel-start-clock"), "START accepted"); h.Seed();
    state->factory->WaitEntered();
    state->block_next_clock = true;
    state->factory->Release();
    state->clock->WaitEntered();
    Require(h.task.Cancel(), "cancel accepted while final seed clock is blocked");
    state->clock->Release();
    Await([&] { return state->destroyed == 1; }, "canceled prepared candidate released");
    Require(state->StartedCount() == 0, "final clock return cannot start a revoked generation");
}

/** @brief A pause during final seed validation retains an unstarted candidate. */
void FactoryClockPauseResume() {
    auto state = std::make_shared<Evidence>();
    state->factory = std::make_shared<Rendezvous>();
    state->clock = std::make_shared<Rendezvous>();
    Harness h(state); Require(h.Start("pause-start-clock"), "START accepted"); h.Seed();
    state->factory->WaitEntered();
    state->block_next_clock = true;
    state->factory->Release();
    state->clock->WaitEntered();
    Require(h.task.Pause(), "pause accepted while final seed clock is blocked");
    const auto calls = state->clock_calls.load();
    state->clock->Release();
    Await([&] { return state->clock_calls > calls; }, "owner processed the paused start window");
    Require(state->StartedCount() == 0, "paused generation remains unstarted after clock return");
    Require(h.task.Resume(), "prepared start resumes with a new permit");
    Await([&] { return state->StartedCount() == 1; }, "retained candidate starts after resume");
    Require(state->created == 1, "final validation pause preserves factory candidate");
}

/** @brief Legacy submission alone cannot manufacture cooperative acceptance. */
void CooperativeBindingsRequired() {
    Harness h(std::make_shared<Evidence>(), false);
    Require(h.Start("missing-cooperation"), "initial request accepted for dependency validation"); h.Seed();
    Await([&] { return h.task.GetLifecycle() == t::TaskLifecycle::kFailed; }, "missing guarded backend fails explicitly");
    Require(h.evidence->StartedCount() == 0, "missing native cooperation never starts a Session");
}

/** @brief Paused initialization does not consume the resumed seed wait budget. */
void PausedInitializationBudget() {
    auto state = std::make_shared<Evidence>();
    state->clock = std::make_shared<Rendezvous>(); state->block_next_clock = true;
    Harness h(state); Require(h.Start("paused-seed-budget"), "START admitted without seed");
    state->clock->WaitEntered(); Require(h.task.Pause(), "initialization pause admitted");
    state->clock->Release();
    Await([&] { return state->clock_calls > 1; }, "owner reached paused seed wait");
    state->now = 100;
    const auto before = state->clock_calls.load();
    Await([&] { return state->clock_calls > before; }, "paused clock advance processed");
    Require(h.task.Resume(), "resume permits a fresh bounded seed wait");
    const auto resumed = state->clock_calls.load();
    Await([&] { return state->clock_calls > resumed + 2; }, "resumed owner advanced without expired wait");
    Require(h.task.GetLifecycle() != t::TaskLifecycle::kFailed, "paused duration does not cause seed timeout");
    h.Seed();
    Await([&] { return state->StartedCount() == 1; }, "fresh post-resume seed starts session");
}

/** @brief Paused owner consumes requirements without advancing navigation nodes. */
void PausedMonitoring() {
    Harness h; Require(h.Start("paused-monitor"), "START accepted"); h.Seed();
    Await([&] { return h.evidence->StartedCount() == 1; }, "actual Session Start published");
    Require(h.task.Pause(), "active pause admitted");
    Await([&] {
        const auto snapshot = h.task.GetExplorationSnapshot();
        return snapshot && snapshot->state == e::ExplorationState::kPaused;
    }, "authoritative pause reaches session before stage advancement");
    h.Observe();
    Await([&] {
        const auto snapshot = h.task.GetExplorationSnapshot();
        return snapshot && snapshot->pending_units == 1;
    }, "paused monitoring consumes real coverage requirement");
    Require(h.evidence->submitted == 0, "paused tree does not submit navigation");
    Require(h.task.Resume(), "original stage resumes");
    Await([&] { return h.evidence->submitted == 1; }, "resumed stage issues one navigation request");
}

/** @brief Cancel stays admissible while native submit blocks; late accept is canceled. */
void NativeSubmissionCancellation() {
    auto state = std::make_shared<Evidence>(); state->submit = std::make_shared<Rendezvous>();
    Harness h(state); Require(h.Start("blocked-native"), "START accepted"); h.Seed(); h.Observe();
    state->submit->WaitEntered(); Require(h.task.Cancel(), "cancel admission is independent of native readiness");
    state->submit->Release();
    Await([&] { return state->canceled == 1; }, "late accepted native request canceled exactly once");
    Await([&] { return h.task.GetLifecycle() == t::TaskLifecycle::kCanceled; }, "task finishes canceled");
    Require(state->submitted == 1, "no duplicate native request");
}

/** @brief Native callback reentrant Shutdown cannot deadlock owner/dispatcher join. */
void NativeShutdownReentry() {
    auto state = std::make_shared<Evidence>(); Harness h(state);
    state->native_reentry = [&h] { h.task.Shutdown(); };
    Require(h.Start("native-close"), "START accepted"); h.Seed(); h.Observe();
    Await([&] { return state->canceled == 1; }, "reentrant close drains late acceptance");
    h.task.Shutdown();
    Require(state->destroyed == 1 && !state->wrong_owner, "external close joins owner after callback returns");
}
}  // namespace

/** @brief Execute all real native lifecycle scenarios and retain each result. */
int main() {
    const std::vector<std::pair<const char*, void(*)()>> cases = {
        {"factory_cancel", FactoryCancellation}, {"factory_pause_resume", FactoryPauseResume},
        {"mapping_pause_prepare", MappingPreparationAfterPause},
        {"mapping_refusal_cleanup", InitializationFailureCleanup}, {"seed_timeout_cleanup", SeedTimeoutCleanup},
        {"factory_preemption", FactoryPreemption}, {"cooperative_bindings", CooperativeBindingsRequired},
        {"factory_clock_cancel", FactoryClockCancellation},
        {"factory_clock_pause_resume", FactoryClockPauseResume},
        {"paused_initialization_budget", PausedInitializationBudget},
        {"paused_monitoring", PausedMonitoring},
        {"native_submit_cancel", NativeSubmissionCancellation}, {"native_shutdown_reentry", NativeShutdownReentry}};
    int failures = 0;
    for (const auto& test : cases) {
        try { test.second(); std::cout << "PASS " << test.first << std::endl; }
        catch (const std::exception& error) {
            ++failures; std::cerr << "FAIL " << test.first << ": " << error.what() << std::endl;
        }
    }
    return failures ? 1 : 0;
}
