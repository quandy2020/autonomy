/* Copyright 2026 The Openbot Authors. */
/**
 * @file bt_runner_test.cpp
 * @brief Real native BT effects, admission, cancellation and exception checks.
 */
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include "autonomy/task/behavior_tree/bt_node_registry.hpp"
#include "autonomy/task/behavior_tree/bt_runner.hpp"
#include "autonomy/task/behavior_tree/bt_task_app.hpp"
#include <automsgs/task/navigation.pb.h>

namespace {
using autonomy::task::BtProfile;
using autonomy::task::BtRunner;
using autonomy::task::BtRunnerHooks;
using autonomy::task::BtRunState;
using namespace std::chrono_literals;

/** @brief Report a failed invariant after all controlled callbacks are released. */
void Require(bool value, const char* message)
{
    if (!value) {
        throw std::runtime_error(message);
    }
}

/** @brief Poll observable completion within a bounded deadline. */
template <class Predicate>
bool Await(Predicate predicate, std::chrono::milliseconds timeout = 3000ms)
{
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (!predicate() && std::chrono::steady_clock::now() < end) {
        std::this_thread::sleep_for(1ms);
    }
    return predicate();
}

/** @brief A bounded synchronization point, rather than a guessed thread delay. */
class Gate {
public:
    /// Notify the observer before waiting for its explicit release.
    void Enter()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        entered_ = true;
        changed_.notify_all();
        if (!changed_.wait_for(lock, 3s, [this] { return released_; })) {
            throw std::runtime_error("test gate was not released");
        }
    }
    /// Return whether the worker reached the controlled callback.
    bool WaitEntered()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        return changed_.wait_for(lock, 3s, [this] { return entered_; });
    }
    /// Release before joining, including after a failed timing observation.
    void Release()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        released_ = true;
        changed_.notify_all();
    }
private:
    /// Protects both callback-handshake flags.
    std::mutex mutex_;
    /// Worker entry and external release notification.
    std::condition_variable changed_;
    /// The callback has reached the synchronization point.
    bool entered_{false};
    /// The observer has authorized the callback to return.
    bool released_{false};
};

/** @brief Observable effects of a real stateful BT action. */
struct Probe {
    /// Number of action starts, independently of runner status publication.
    std::atomic<unsigned> starts{0};
    /// Number of continuing action ticks.
    std::atomic<unsigned> continues{0};
    /// Number of cancellation/cleanup calls.
    std::atomic<unsigned> halts{0};
    /// Exercise a nonstandard cleanup failure without a simulated runner.
    bool fail_halt{false};
};

/** @brief Stateful native action with observable execution and halt side effects. */
class ProbeAction final : public BT::StatefulActionNode {
public:
    /// Resolve the fixture's owned probe from the worker blackboard.
    ProbeAction(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config),
          probe_(config.blackboard->get<std::shared_ptr<Probe>>("probe")) {}
    /// The probe uses a typed blackboard value and has no XML ports.
    static BT::PortsList providedPorts() { return {}; }
    /// Count an actual node start and retain RUNNING for cancellation tests.
    BT::NodeStatus onStart() override
    {
        ++probe_->starts;
        return BT::NodeStatus::RUNNING;
    }
    /// Count actual continuing execution, not a runner callback.
    BT::NodeStatus onRunning() override
    {
        ++probe_->continues;
        return BT::NodeStatus::RUNNING;
    }
    /// Count cleanup and optionally throw a nonstandard exception.
    void onHalted() override
    {
        ++probe_->halts;
        if (probe_->fail_halt) {
            throw 23;
        }
    }
private:
    /// Shared probe survives destruction of the worker's tree.
    std::shared_ptr<Probe> probe_;
};

/** @brief Add the test action through the actual native registration mechanism. */
void RegisterProbe(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<ProbeAction>("AdmissionProbe");
}

/** @brief Own a uniquely named real XML file, removed after the runner has joined. */
class XmlFile {
public:
    /// Write one native BehaviorTree.CPP invocation.
    explicit XmlFile(const std::string& nodes)
    {
        static std::atomic<unsigned> sequence{0};
        path_ = std::filesystem::temp_directory_path() /
            ("autonomy-admission-" + std::to_string(
                std::chrono::steady_clock::now().time_since_epoch().count()) +
             "-" + std::to_string(++sequence) + ".xml");
        std::ofstream file(path_);
        file << "<root BTCPP_format=\"4\" main_tree_to_execute=\"Test\">"
                "<BehaviorTree ID=\"Test\">" << nodes << "</BehaviorTree></root>";
        Require(static_cast<bool>(file), "write native XML fixture");
    }
    /// Remove only the fixture's own file.
    ~XmlFile()
    {
        std::error_code error;
        std::filesystem::remove(path_, error);
    }
    /// Native runner input path.
    std::string path() const { return path_.string(); }
private:
    /// Unique fixture path; outlives its runner.
    std::filesystem::path path_;
};

/** @brief Hook host with real blackboard effects and controlled callback faults. */
class Hooks : public BtRunnerHooks {
public:
    /// Publish fixture inputs before building nodes, then run optional setup fault.
    void SetupBlackboard(const BT::Blackboard::Ptr& board) override
    {
        board->set("effects", 0);
        board->set("probe", probe);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            board_ = board;
        }
        if (setup) setup();
    }
    /// Call the real admission hook, preserving the default when unconfigured.
    bool BeforeTick() override
    {
        ++before;
        return admit ? admit() : BtRunnerHooks::BeforeTick();
    }
    /// Observe monitor iterations and optionally throw from the worker callback.
    void OnTick() override
    {
        ++after;
        if (monitor) monitor();
    }
    /// Exercise the actual status logger callback path.
    void OnStatusLog(const std::vector<autonomy::task::BtStatusEvent>&) override
    {
        if (status) status();
    }
    /// Read the script's real blackboard counter under its native locking.
    int effects() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return board_ ? board_->get<int>("effects") : 0;
    }
    /// Stateful action evidence, separate from scripts and monitoring.
    std::shared_ptr<Probe> probe = std::make_shared<Probe>();
    /// Number of admission calls actually reached by the runner.
    std::atomic<unsigned> before{0};
    /// Number of post-iteration/terminal callbacks actually reached.
    std::atomic<unsigned> after{0};
    /// Callback policies are installed before starting and remain immutable.
    std::function<bool()> admit;
    /// Optional setup exception injection.
    std::function<void()> setup;
    /// Optional monitoring callback or exception injection.
    std::function<void()> monitor;
    /// Optional logger exception injection.
    std::function<void()> status;
private:
    /// Synchronizes publication of the blackboard pointer.
    mutable std::mutex mutex_;
    /// Retain script evidence after tree destruction.
    BT::Blackboard::Ptr board_;
};

/// Each admitted iteration changes a real script counter and runs a real action.
constexpr char kRunningTree[] =
    "<ReactiveSequence><Script code=\"effects = effects + 1\"/>"
    "<AdmissionProbe/></ReactiveSequence>";

/** @brief Start a real configured runner with a short fixture loop period. */
void Start(BtRunner& runner, Hooks& hooks, const XmlFile& file,
           uint32_t period_ms = 1)
{
    BtProfile profile;
    profile.loop_period_ms = period_ms;
    Require(runner.Configure(profile), "configure actual native factory");
    runner.SetHooks(&hooks);
    Require(runner.Run(file.path()), "start actual native XML");
}

/** @brief The default hook admits an ordinary invocation without changing behavior. */
void DefaultAdmission()
{
    XmlFile file("<Script code=\"effects = effects + 1\"/>");
    Hooks hooks;
    BtRunner runner;
    Start(runner, hooks, file);
    Require(Await([&] { return !runner.IsRunning(); }), "default tree terminates");
    Require(runner.state() == BtRunState::kSucceeded && hooks.effects() == 1,
            "default hook preserves actual script execution");
}

/** @brief Rejecting the first tick creates no script or action side effects. */
void DeniedBeforeFirstTick()
{
    XmlFile file(kRunningTree);
    Hooks hooks;
    hooks.admit = [] { return false; };
    BtRunner runner;
    Start(runner, hooks, file);
    Require(Await([&] { return !runner.IsRunning(); }), "denied tree terminates");
    Require(runner.state() == BtRunState::kCanceled && hooks.before == 1 &&
            hooks.effects() == 0 && hooks.probe->starts == 0,
            "denied admission executes no nodes");
}

/** @brief Revocation on a later iteration halts the already running native action. */
void RevokedRunningTree()
{
    XmlFile file(kRunningTree);
    Hooks hooks;
    hooks.admit = [&] { return hooks.before == 1; };
    BtRunner runner;
    Start(runner, hooks, file);
    Require(Await([&] { return !runner.IsRunning(); }), "revoked tree terminates");
    Require(runner.state() == BtRunState::kCanceled && hooks.effects() == 1 &&
            hooks.probe->starts == 1 && hooks.probe->continues == 0 &&
            hooks.probe->halts == 1, "revocation prevents next effects and halts owner");
}

/** @brief Cancellation while admission is blocked prevents its otherwise allowed tick. */
void CancelDuringAdmissionAndReentry()
{
    XmlFile file(kRunningTree);
    Gate gate;
    Hooks hooks;
    BtRunner runner;
    std::atomic<bool> rejected_restarts{false};
    hooks.admit = [&] {
        gate.Enter();
        BtProfile profile;
        rejected_restarts = !runner.Configure(profile) && !runner.Run(file.path());
        runner.SetHooks(nullptr);
        runner.Cancel();
        return true;
    };
    Start(runner, hooks, file);
    const bool entered = gate.WaitEntered();
    auto cancel = std::async(std::launch::async, [&] { return runner.Cancel(); });
    const bool joining = cancel.wait_for(10ms) == std::future_status::timeout;
    gate.Release();
    const bool ended = cancel.wait_for(3s) == std::future_status::ready;
    Require(entered && joining && ended && cancel.get(), "external cancel joins reentrant hook");
    Require(!runner.IsRunning() && runner.state() == BtRunState::kCanceled &&
            rejected_restarts && hooks.effects() == 0 && hooks.probe->starts == 0,
            "cancel during admission prevents execution and self-join");
}

/** @brief Pause arriving during admission suppresses effects until a newly admitted tick. */
void PauseDuringAdmission()
{
    XmlFile file(kRunningTree);
    Gate gate;
    Hooks hooks;
    hooks.admit = [&] {
        if (hooks.before == 1) gate.Enter();
        return hooks.before <= 2;
    };
    BtRunner runner;
    Start(runner, hooks, file);
    const bool entered = gate.WaitEntered();
    runner.Pause();
    gate.Release();
    const bool monitored = Await([&] { return hooks.after >= 1; });
    const bool no_effects = hooks.effects() == 0 && hooks.probe->starts == 0;
    runner.Resume();
    const bool ended = Await([&] { return !runner.IsRunning(); });
    Require(entered && monitored && no_effects && ended,
            "pause suppresses the checked tick and resumes monitoring");
    Require(hooks.effects() == 1 && hooks.before == 3 && hooks.probe->halts == 1,
            "resume rechecks admission and cannot reuse the paused check");
}

/** @brief A long polling period does not delay cancellation of a bounded action. */
void CancelInterruptsWait()
{
    XmlFile file(kRunningTree);
    Hooks hooks;
    BtRunner runner;
    Start(runner, hooks, file, 5000);
    Require(Await([&] { return hooks.after >= 1; }), "worker has ticked before timed wait");
    const auto start = std::chrono::steady_clock::now();
    runner.Cancel();
    const auto duration = std::chrono::steady_clock::now() - start;
    Require(duration < 2s && !runner.IsRunning() && hooks.probe->halts == 1,
            "cancel interrupts five-second interval and halts actual action");
}

/** @brief Pause/resume arriving before a timed wait must not lose their wakeup. */
void PauseResumeInterruptsWait()
{
    XmlFile file(kRunningTree);
    Gate gate;
    Hooks hooks;
    BtRunner runner;
    hooks.monitor = [&] {
        if (hooks.after == 1) runner.Pause();
        if (hooks.after == 2) gate.Enter();
    };
    Start(runner, hooks, file, 5000);
    const bool entered = gate.WaitEntered();
    const bool paused_effects = hooks.effects() == 1;
    runner.Resume();
    gate.Release();
    const bool advanced = Await([&] { return hooks.probe->continues >= 1; }, 2000ms);
    runner.Cancel();
    Require(entered && paused_effects && advanced && hooks.probe->halts == 1,
            "pause and resume interrupt the long interval without lost notifications");
}

/** @brief A callback fault remains visible even when it also requests cancel. */
void CancelWithFaultKeepsFailure()
{
    for (const bool standard : {true, false}) {
        XmlFile file(kRunningTree);
        Hooks hooks;
        BtRunner runner;
        hooks.admit = [&]() -> bool {
            runner.Cancel();
            if (standard) throw std::runtime_error("cancel with callback fault");
            throw 19;
        };
        Start(runner, hooks, file);
        Require(Await([&] { return !runner.IsRunning(); }), "cancel fault terminates");
        runner.Cancel();
        Require(runner.state() == BtRunState::kFailed && hooks.effects() == 0,
                "cancel does not mask a callback fault or execute nodes");
    }
}

/** @brief Standard and nonstandard exceptions cannot escape the native worker. */
void CallbackExceptionBoundaries()
{
    for (const bool standard : {true, false}) {
        for (const unsigned stage : {0u, 1u, 2u, 3u}) {
            XmlFile file(kRunningTree);
            Hooks hooks;
            const auto fail = [standard] {
                if (standard) throw std::runtime_error("controlled hook fault");
                throw 17;
            };
            if (stage == 0) hooks.setup = fail;
            if (stage == 1) hooks.admit = [fail]() -> bool { fail(); return true; };
            if (stage == 2) hooks.monitor = fail;
            if (stage == 3) hooks.status = fail;
            BtRunner runner;
            Start(runner, hooks, file);
            Require(Await([&] { return !runner.IsRunning(); }), "callback failure terminates");
            Require(runner.state() == BtRunState::kFailed, "worker publishes callback failure");
            if (stage < 2) {
                Require(hooks.effects() == 0 && hooks.probe->starts == 0,
                        "setup/admission exceptions execute no actions");
            } else {
                Require(hooks.probe->starts == 1 && hooks.probe->halts == 1,
                        "post-tick/status exceptions halt actual running action");
            }
        }
    }
}

/** @brief A failed halt stays failed rather than becoming a clean cancel. */
void HaltFailureIsRetained()
{
    XmlFile file(kRunningTree);
    Hooks hooks;
    hooks.probe->fail_halt = true;
    hooks.admit = [&] { return hooks.before == 1; };
    BtRunner runner;
    Start(runner, hooks, file);
    Require(Await([&] { return !runner.IsRunning(); }), "halt failure terminates safely");
    runner.Cancel();
    Require(runner.state() == BtRunState::kFailed && hooks.probe->halts >= 1,
            "management cancellation does not erase cleanup fault");
}

/** @brief A typed task must reach its domain gate through the real runner bridge. */
class AdmissionHost final : public autonomy::task::BtTaskApp<
    autonomy::task::proto::NavigationGoal,
    autonomy::task::proto::NavigationFeedback,
    autonomy::task::proto::NavigationResult> {
public:
    /// Stop callbacks before derived fixture fields are destroyed.
    ~AdmissionHost() override { Shutdown(); }
    /// Use the existing typed navigation profile and task interface.
    automsgs::msgs::vehicle_msgs::RobotTaskType GetTaskType() const override
    {
        return automsgs::msgs::vehicle_msgs::ROBOT_TASK_NAVIGATION;
    }
    /// Start the native host without replacing its runner or execution loop.
    bool Launch(const std::string& file) { return StartTree(file); }
    /// Read final worker completion through the native host accessor.
    bool done() { return !runner()->IsRunning(); }
    /// Read the actual canceled outcome from the native runner.
    BtRunState outcome() { return runner()->state(); }
    /// Read actual script effects only after the worker has finished.
    int effects() const { return board_ ? board_->get<int>("effects") : 0; }
    /// Number of domain admission calls, independent of post-tick callbacks.
    std::atomic<unsigned> calls{0};
protected:
    /// A denied domain lease must prevent the next script from running.
    bool BeforeTreeTick() override { ++calls; return false; }
    /// Prepare a real script counter for the typed host.
    void PopulateBlackboard(const BT::Blackboard::Ptr& board) override
    {
        board->set("effects", 0);
        board_ = board;
    }
    /// Goal submission is outside this host-bridge test.
    bool OnGoal(const autonomy::task::proto::NavigationGoal&) override { return false; }
    /// This fixture observes native runner effects, not synthetic feedback.
    void FillFeedback(autonomy::task::proto::NavigationFeedback*) const override {}
    /// This fixture observes native runner effects, not synthetic results.
    void FillResult(autonomy::task::proto::NavigationResult*) const override {}
private:
    /// Preserve blackboard evidence after tree cleanup; read after done().
    BT::Blackboard::Ptr board_;
};

/** @brief Exercise BtTaskApp's BeforeTreeTick forwarding instead of a test adapter. */
void TypedHostAdmissionBridge()
{
    XmlFile file("<Script code=\"effects = effects + 1\"/>");
    AdmissionHost host;
    autonomy::task::proto::TaskServerOptions options;
    Require(host.Initialize(options) && host.Launch(file.path()), "initialize typed native host");
    Require(Await([&] { return host.done(); }) && host.calls == 1 &&
            host.outcome() == BtRunState::kCanceled && host.effects() == 0,
            "domain gate prevents actual native script effects");
}

}  // namespace

/** @brief Run bounded native checks, with failures visible to CTest. */
int main()
{
    autonomy::task::AddBtNodeRegistrar(&RegisterProbe);
    try {
        DefaultAdmission(); std::cout << "PASS default_admission\n";
        DeniedBeforeFirstTick(); std::cout << "PASS denied_before_first_tick\n";
        RevokedRunningTree(); std::cout << "PASS revoked_running_tree\n";
        CancelDuringAdmissionAndReentry(); std::cout << "PASS cancel_during_admission_reentry\n";
        PauseDuringAdmission(); std::cout << "PASS pause_during_admission\n";
        CancelInterruptsWait(); std::cout << "PASS cancel_interrupts_wait\n";
        PauseResumeInterruptsWait(); std::cout << "PASS pause_resume_interrupts_wait\n";
        CancelWithFaultKeepsFailure(); std::cout << "PASS cancel_with_fault_keeps_failure\n";
        CallbackExceptionBoundaries(); std::cout << "PASS callback_exception_boundaries\n";
        HaltFailureIsRetained(); std::cout << "PASS halt_failure_is_retained\n";
        TypedHostAdmissionBridge(); std::cout << "PASS typed_host_admission_bridge\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL native BT admission: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
