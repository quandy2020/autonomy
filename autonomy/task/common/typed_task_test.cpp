/* Copyright 2026 The Openbot Authors. */
/** @file typed_task_test.cpp @brief Native goal acceptance and identity evidence. */
#include <chrono>
#include <condition_variable>
#include <functional>
#include <future>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>

#include "autonomy/task/common/typed_task.hpp"
#include <automsgs/task/navigation.pb.h>

namespace {
namespace proto = autonomy::task::proto;
using namespace std::chrono_literals;

/** @brief Fail an observable invariant without replacing native task state. */
void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

/** @brief Bound a handler's acceptance window using explicit synchronization. */
class Gate {
public:
    /// Publish handler entry and wait for an external release.
    void Enter() {
        std::unique_lock<std::mutex> lock(mutex_);
        entered_ = true;
        changed_.notify_all();
        if (!changed_.wait_for(lock, 3s, [this] { return released_; }))
            throw std::runtime_error("acceptance gate deadline");
    }
    /// Wait for a real pending OnGoal, rather than a guessed sleep.
    bool WaitEntered() {
        std::unique_lock<std::mutex> lock(mutex_);
        return changed_.wait_for(lock, 3s, [this] { return entered_; });
    }
    /// Always release before joining the submitted goal.
    void Release() {
        std::lock_guard<std::mutex> lock(mutex_);
        released_ = true;
        changed_.notify_all();
    }
private:
    /// Protects the handler-entry and release handshake.
    std::mutex mutex_;
    /// Signals both sides of the bounded handshake.
    std::condition_variable changed_;
    /// The real native handler is waiting.
    bool entered_{false};
    /// Its observer has finished examining the pending state.
    bool released_{false};
};

/** @brief Use the actual typed base with navigation protobufs and domain hooks. */
class Task final : public autonomy::task::TypedTaskAppBase<
    proto::NavigationGoal, proto::NavigationFeedback, proto::NavigationResult> {
public:
    /// Immutable callback policy, installed before concurrent calls.
    std::function<bool(const proto::NavigationGoal&)> handler;
    /// Return the same native task type as the navigation application.
    automsgs::msgs::vehicle_msgs::RobotTaskType GetTaskType() const override {
        return automsgs::msgs::vehicle_msgs::ROBOT_TASK_NAVIGATION;
    }
    /// Produce a real native terminal result for identity-query tests.
    void Complete() { SetLifecycle(autonomy::task::TaskLifecycle::kSucceeded); }
    /// Domain handlers own execution state and reset old progress on acceptance.
    void Start(const std::string& id) {
        SetLifecycle(autonomy::task::TaskLifecycle::kRunning);
        SetProgress(0.f, id);
    }
protected:
    /// New navigation goals replace identity; controls retain the active one.
    bool ShouldUpdateGoalIdentity(const proto::NavigationGoal& goal) const override {
        return goal.command() == proto::NAV_CMD_START || goal.command() == proto::NAV_CMD_REPLAN;
    }
    /// Control acceptance timing; the base lifecycle and identity are unchanged.
    bool OnGoal(const proto::NavigationGoal& goal) override {
        if (handler) return handler(goal);
        Start(goal.has_header() ? goal.header().task_id() : std::string{});
        return true;
    }
    /// Fill through the actual native protobuf interface.
    void FillFeedback(proto::NavigationFeedback* feedback) const override {
        feedback->set_status(proto::NAV_STATUS_SUCCEEDED);
    }
    /// Fill through the real terminal-result API, not a fixture return value.
    void FillResult(proto::NavigationResult* result) const override {
        *result->mutable_result() = MakeTaskResult();
        result->set_final_status(proto::NAV_STATUS_SUCCEEDED);
    }
};

/** @brief Build a caller-owned START with a distinct task/client identity. */
proto::NavigationGoal Goal(const std::string& id) {
    proto::NavigationGoal goal;
    goal.mutable_header()->set_task_id(id);
    goal.mutable_header()->set_client_id("client/" + id);
    goal.set_command(proto::NAV_CMD_START);
    return goal;
}

/** @brief A rejected goal cannot claim the previous task's successful result. */
void RejectionPreservesIdentity() {
    Task task;
    Require(task.SubmitGoal(Goal("old")), "accept old task");
    task.Complete();
    task.handler = [](const proto::NavigationGoal&) { return false; };
    Require(!task.SubmitGoal(Goal("rejected")), "reject new task");
    const auto snapshot = task.GetSnapshot();
    proto::NavigationResult result;
    Require(snapshot.task_id() == "old" && snapshot.client_id() == "client/old" &&
            task.GetResultForTask("old", &result) &&
            !task.GetResultForTask("rejected", &result),
            "rejected identity never inherits old terminal result");
}

/** @brief Terminal reads cannot expose an old result inside pending acceptance. */
void PendingAdmissionBlocksResults() {
    for (const bool accepted : {false, true}) {
        Task task;
        Require(task.SubmitGoal(Goal("old")), "accept old task");
        task.Complete();
        Gate gate;
        task.handler = [&](const proto::NavigationGoal& goal) {
            gate.Enter();
            if (accepted) task.Start(goal.header().task_id());
            return accepted;
        };
        auto submit = std::async(std::launch::async, [&] { return task.SubmitGoal(Goal("new")); });
        const bool entered = gate.WaitEntered();
        proto::NavigationResult result;
        result.set_final_status(proto::NAV_STATUS_FAILED);
        const auto original = result.SerializeAsString();
        const auto start = std::chrono::steady_clock::now();
        const bool reads_blocked = !task.GetResult(&result) &&
            !task.GetResultForTask("old", &result) && !task.GetResultForTask("new", &result);
        const bool unchanged = result.SerializeAsString() == original;
        const bool quick = std::chrono::steady_clock::now() - start < 1s;
        const bool old_identity = task.GetSnapshot().task_id() == "old";
        gate.Release();
        const bool returned = submit.get();
        Require(entered && reads_blocked && unchanged && quick && old_identity && returned == accepted,
                "pending handler publishes neither identity nor stale result and holds no read lock");
        Require(task.GetSnapshot().task_id() == (accepted ? "new" : "old"),
                "identity follows the actual acceptance decision");
        Require(accepted ? !task.GetResultForTask("new", &result) :
                task.GetResultForTask("old", &result),
                "accepted pending work does not reuse previous terminal completion");
    }
}

/** @brief Accepted anonymous requests clear previous task and client identity. */
void AnonymousAcceptanceClearsIdentity() {
    Task task;
    Require(task.SubmitGoal(Goal("old")), "accept named task");
    proto::NavigationGoal anonymous;
    anonymous.set_command(proto::NAV_CMD_START);
    Require(task.SubmitGoal(anonymous), "accept anonymous task");
    task.Complete();
    proto::NavigationResult result;
    Require(task.GetSnapshot().task_id().empty() && task.GetSnapshot().client_id().empty() &&
            !task.GetResultForTask("old", &result), "anonymous task cannot inherit old identity");
}

/** @brief Both exception classes release the terminal-read barrier. */
void ExceptionsReleaseAdmission() {
    for (const bool standard : {false, true}) {
        Task task;
        Require(task.SubmitGoal(Goal("old")), "accept named task");
        task.Complete();
        task.handler = [standard](const proto::NavigationGoal&) -> bool {
            if (standard) throw std::runtime_error("domain rejection fault");
            throw 29;
        };
        bool caught = false;
        try { task.SubmitGoal(Goal("throwing")); } catch (...) { caught = true; }
        proto::NavigationResult result;
        Require(caught && task.GetSnapshot().task_id() == "old" &&
                task.GetResultForTask("old", &result) && !task.GetResultForTask("throwing", &result),
                "exception cleanup restores reads without publishing rejected identity");
    }
}

/** @brief A later accepted goal wins; a later rejection does not erase acceptance. */
void ConcurrentAcceptanceOrdering() {
    for (const bool accept_newer : {false, true}) {
        Task task;
        Gate gate;
        task.handler = [&](const proto::NavigationGoal& goal) {
            if (goal.header().task_id() == "older") { gate.Enter(); return true; }
            return accept_newer;
        };
        auto older = std::async(std::launch::async, [&] { return task.SubmitGoal(Goal("older")); });
        const bool entered = gate.WaitEntered();
        const bool newer = task.SubmitGoal(Goal("newer"));
        gate.Release();
        const bool old_accepted = older.get();
        Require(entered && newer == accept_newer && old_accepted == !accept_newer &&
                task.GetSnapshot().task_id() == (accept_newer ? "newer" : "older"),
                "late acceptance cannot replace a newer accepted identity");
    }
}

/** @brief Closed and reinitialized hosts cannot publish pre-shutdown acceptance. */
void ShutdownInvalidatesPendingAcceptance() {
    Task task;
    Gate gate;
    task.handler = [&](const proto::NavigationGoal& goal) {
        if (goal.header().task_id() == "old") gate.Enter();
        return true;
    };
    auto old = std::async(std::launch::async, [&] { return task.SubmitGoal(Goal("old")); });
    const bool entered = gate.WaitEntered();
    task.Shutdown();
    const bool closed = !task.SubmitGoal(Goal("closed"));
    proto::TaskServerOptions options;
    const bool initialized = task.Initialize(options);
    const bool new_accepted = task.SubmitGoal(Goal("new"));
    gate.Release();
    Require(entered && closed && initialized && new_accepted && !old.get() &&
            task.GetSnapshot().task_id() == "new", "old handler cannot reopen shutdown identity");
}

/** @brief Callback reentry uses the same native acceptance ordering. */
void ReentrantAcceptanceOrdering() {
    Task task;
    task.handler = [&](const proto::NavigationGoal& goal) {
        return goal.header().task_id() != "outer" || task.SubmitGoal(Goal("inner"));
    };
    Require(!task.SubmitGoal(Goal("outer")) && task.GetSnapshot().task_id() == "inner",
            "reentrant newer acceptance preserves its published identity");
}

/** @brief A real domain completion inside OnGoal is published under the new ID. */
void ImmediateCompletionKeepsIdentity() {
    Task task;
    Require(task.SubmitGoal(Goal("old")), "accept old task");
    task.Complete();
    task.handler = [&](const proto::NavigationGoal& goal) {
        task.Start(goal.header().task_id());
        task.Complete();
        return true;
    };
    Require(task.SubmitGoal(Goal("immediate")), "accept immediate completed task");
    proto::NavigationResult result;
    Require(task.GetResultForTask("immediate", &result) &&
            result.result().message() == "immediate" &&
            !task.GetResultForTask("old", &result), "fast completion belongs to its accepted identity");
}

/** @brief An acknowledged control request is not a new task identity. */
void ControlAcceptanceKeepsIdentity() {
    Task task;
    Require(task.SubmitGoal(Goal("active")), "accept active task");
    task.handler = [](const proto::NavigationGoal&) { return true; };
    auto pause = Goal("pause-command");
    pause.set_command(proto::NAV_CMD_PAUSE);
    Require(task.SubmitGoal(pause) && task.GetSnapshot().task_id() == "active",
            "domain control request retains the active identity");
}
}  // namespace

/** @brief Exercise actual identity/result methods with bounded native handlers. */
int main() {
    try {
        RejectionPreservesIdentity(); std::cout << "PASS rejection_preserves_identity\n";
        PendingAdmissionBlocksResults(); std::cout << "PASS pending_admission_blocks_results\n";
        AnonymousAcceptanceClearsIdentity(); std::cout << "PASS anonymous_acceptance_clears_identity\n";
        ExceptionsReleaseAdmission(); std::cout << "PASS exceptions_release_admission\n";
        ConcurrentAcceptanceOrdering(); std::cout << "PASS concurrent_acceptance_ordering\n";
        ShutdownInvalidatesPendingAcceptance(); std::cout << "PASS shutdown_invalidates_pending_acceptance\n";
        ReentrantAcceptanceOrdering(); std::cout << "PASS reentrant_acceptance_ordering\n";
        ImmediateCompletionKeepsIdentity(); std::cout << "PASS immediate_completion_keeps_identity\n";
        ControlAcceptanceKeepsIdentity(); std::cout << "PASS control_acceptance_keeps_identity\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL typed native acceptance: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
