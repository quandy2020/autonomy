/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file motion_dispatch_test.cpp
 * @brief Native dispatch ownership, uncertain acceptance and cancellation tests.
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
#include <vector>

#include <automsgs/actions/nav_actions.pb.h>
#include "autolink/action/client.hpp"

/**
 * @brief Unique transport fixture using actual navigation protobuf payloads.
 */
struct DispatchProbeAction {
    using Goal = automsgs::actions::FollowPathAction::Goal;
    using Feedback = automsgs::actions::FollowPathAction::Feedback;
    using Result = automsgs::actions::FollowPathAction::Result;
};

namespace autolink::action {
/**
 * @brief Test-only friend factory for result-aware native goal handles.
 * Production action specializations and their middleware remain unchanged.
 */
template <>
class Client<DispatchProbeAction> {
public:
    using GoalHandle = ClientGoalHandle<DispatchProbeAction>;
    using WrappedResult = GoalHandle::WrappedResult;
    /**
     * @brief The same callback contract as the production asynchronous
              * client.
     */
    struct SendGoalOptions {
        std::function<void(std::shared_ptr<GoalHandle>)> goal_response_callback;
        GoalHandle::FeedbackCallback feedback_callback;
        GoalHandle::ResultCallback result_callback;
        std::function<void(const GoalUUID&)> goal_request_callback;
        std::function<void(const GoalUUID&)> goal_rejection_callback;
        std::function<void(const GoalUUID&, std::exception_ptr)> goal_error_callback;
    };
    /**
     * @brief Construct a genuine, result-aware native handle for a recorded
              * UUID.
     */
    static std::shared_ptr<GoalHandle> Handle(const GoalUUID& id,
                                            const SendGoalOptions& options) {
        GoalInfo info; info.goal_id = id; info.stamp = 1;
        auto handle = std::make_shared<GoalHandle>(info, options.feedback_callback,
                                                   options.result_callback);
        handle->SetResultAwareness(true);
        return handle;
    }
    /**
     * @brief Complete the native promise and its real result callback.
     */
    static void Finish(const std::shared_ptr<GoalHandle>& handle,
                       const WrappedResult& result) { handle->SetResult(result); }
    /**
     * @brief Simulate a local monitor losing awareness without remote
              * termination.
     */
    static void ForgetAwareness(const std::shared_ptr<GoalHandle>& handle) {
        handle->SetResultAwareness(false);
    }
    /**
     * @brief Exercise the production invalidation path, not a terminal
              * callback.
     */
    static void Invalidate(const std::shared_ptr<GoalHandle>& handle) {
        handle->Invalidate(UnawareGoalHandleError());
    }
    /**
     * @brief Deliver a delayed native status observation.
     */
    static void Status(const std::shared_ptr<GoalHandle>& handle, GoalStatus status) {
        handle->SetStatus(static_cast<int8_t>(status));
    }
};
}  // namespace autolink::action

#include "autonomy/task/behavior_tree/bt_node_registry.hpp"
#include "autonomy/task/navigation/action_session.hpp"
#include "autonomy/task/navigation/navigation.hpp"

namespace {
using namespace std::chrono_literals;
using Gate = autonomy::task::common::MotionDispatchGate;
using Native = autolink::action::Client<DispatchProbeAction>;
using Handle = Native::GoalHandle;
using Wrapped = Native::WrappedResult;
using UUID = autolink::action::GoalUUID;
using Code = autolink::action::ResultCode;
using Cancellation = autonomy::task::common::TaskActionClient<DispatchProbeAction>::GoalCancellation;
namespace proto = autonomy::task::proto;

/**
 * @brief Fail a contract observation with its exact missing requirement.
 */
void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
/**
 * @brief Await actual observable effects with a bounded deadline.
 */
template <class Predicate>
bool Await(Predicate predicate, std::chrono::milliseconds timeout = 1500ms) {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (!predicate() && std::chrono::steady_clock::now() < end)
        std::this_thread::sleep_for(1ms);
    return predicate();
}
/**
 * @brief Prepare a ready cancel acknowledgment; it is not a movement result.
 */
std::shared_future<bool> Ack(bool accepted) {
    std::promise<bool> promise; promise.set_value(accepted);
    return promise.get_future().share();
}

/**
 * @brief Evidence for a single transport request, independent of local
          * sessions.
 */
struct Record {
    UUID id{};                                  ///< Recorded send identity.
    Native::SendGoalOptions options;             ///< Immutable transport callbacks.
    std::promise<std::shared_ptr<Handle>> accepted; ///< Acceptance response channel.
    std::shared_ptr<Handle> handle;               ///< Real native handle, if accepted.
    std::mutex mutex;                            ///< Protects handle publication.
    std::atomic<bool> finished{false};            ///< Prevents duplicate promise completion.
};
/**
 * @brief Controlled RPC replies, including ambiguity and missing
          * acknowledgments.
 */
enum class CancelReply { kAccept, kReject, kException, kThrow, kInvalid, kPending };
/**
 * @brief Shared fixture effects; production cancellation captures this
          * weakly.
 */
struct TransportState {
    std::mutex mutex;                            ///< Protects recorded requests/effects.
    std::condition_variable wake;                ///< Send entry/release handshake.
    std::vector<std::shared_ptr<Record>> records; ///< Requests retained for late replies.
    std::vector<UUID> canceled;                   ///< Actual targeted cancel RPC identities.
    std::vector<std::shared_ptr<std::promise<bool>>> pending; ///< Missing ACK promises.
    std::size_t entered{0};                      ///< Sends with UUID notification completed.
    bool block_send{false};                      ///< Block inside AsyncSendGoal before return.
    bool released{false};                        ///< Explicit release of the blocking seam.
    bool terminate_on_cancel{false};             ///< Deliver a real CANCELED result after RPC.
    CancelReply reply{CancelReply::kAccept};      ///< Configured response, set before execution.
    std::function<void()> cancel_hook;           ///< Test synchronous reentry outside locks.
    std::weak_ptr<Gate> observed_gate;            ///< Native blackboard dispatch gate.
    bool block_tick{false};                      ///< Hold a real running BT tick for pause races.
    bool tick_released{false};                   ///< Release that tick without releasing a send.
    std::size_t tick_entered{0};                 ///< Running ticks which reached the test barrier.
    std::size_t tick_returned{0};                ///< Ticks which actually returned to the runner.
    BT::NodeStatus last_tick_status{BT::NodeStatus::IDLE}; ///< Result returned by the actual node.
};

/**
 * @brief Return a stable recorded request without holding fixture locks.
 */
std::shared_ptr<Record> Request(const std::shared_ptr<TransportState>& state,
                                std::size_t index = 0) {
    std::lock_guard<std::mutex> lock(state->mutex);
    return state->records.at(index);
}
/**
 * @brief Count actual sends, rather than local reservations or intentions.
 */
std::size_t Sends(const std::shared_ptr<TransportState>& state) {
    std::lock_guard<std::mutex> lock(state->mutex); return state->records.size();
}
/**
 * @brief Count UUID-targeted cancellation calls.
 */
std::size_t Cancels(const std::shared_ptr<TransportState>& state) {
    std::lock_guard<std::mutex> lock(state->mutex); return state->canceled.size();
}
/**
 * @brief Observe entry only after the native identity callback has run.
 */
bool Entered(const std::shared_ptr<TransportState>& state, std::size_t count = 1) {
    std::unique_lock<std::mutex> lock(state->mutex);
    return state->wake.wait_for(lock, 1500ms, [&] { return state->entered >= count; });
}
/**
 * @brief Release a potentially blocked sender, including failure cleanup.
 */
void Release(const std::shared_ptr<TransportState>& state) {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->released = true; state->tick_released = true; state->wake.notify_all();
}

/**
 * @brief Observe an executing native node after the runner's pre-tick checks.
 */
bool EnteredTick(const std::shared_ptr<TransportState>& state) {
    std::unique_lock<std::mutex> lock(state->mutex);
    return state->wake.wait_for(lock, 1500ms, [&] { return state->tick_entered != 0; });
}
/**
 * @brief Let the already executing node poll the newly suspended session.
 */
void ReleaseTick(const std::shared_ptr<TransportState>& state) {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->tick_released = true; state->wake.notify_all();
}
/**
 * @brief Deliver a terminal result through the actual native handle promise.
 */
void Finish(const std::shared_ptr<Record>& record, Code code = Code::CANCELED,
            const UUID* override_id = nullptr) {
    std::shared_ptr<Handle> handle;
    { std::lock_guard<std::mutex> lock(record->mutex); handle = record->handle; }
    Require(static_cast<bool>(handle), "terminal requires accepted native handle");
    if (record->finished.exchange(true)) return;
    Wrapped result; result.goal_id = override_id ? *override_id : record->id;
    result.code = code; result.result = std::make_shared<DispatchProbeAction::Result>();
    result.result->set_error_code(
        static_cast<decltype(result.result->error_code())>(0));
    Native::Finish(handle, result);
}
/**
 * @brief Publish matching acceptance; native result awareness precedes
          * callbacks.
 */
void Accept(const std::shared_ptr<Record>& record) {
    auto handle = Native::Handle(record->id, record->options);
    { std::lock_guard<std::mutex> lock(record->mutex); record->handle = handle; }
    record->accepted.set_value(handle);
    if (record->options.goal_response_callback) record->options.goal_response_callback(handle);
}

/**
 * @brief Control transport timing while reusing production GoalCancellation.
 */
class ControlledClient {
public:
    using Goal = DispatchProbeAction::Goal;
    using GoalHandle = Handle;
    using WrappedResult = Wrapped;
    using SendGoalOptions = Native::SendGoalOptions;
    using GoalCancellation = Cancellation;
    /**
     * @brief Bind recorded transport effects, not a BT node or task.
     */
    explicit ControlledClient(std::shared_ptr<TransportState> state) : state_(std::move(state)) {}
    /**
     * @brief Install a weak, UUID-targeted cancel transport boundary.
     */
    std::shared_ptr<GoalCancellation> MakeGoalCancellation() {
        const std::weak_ptr<TransportState> weak = state_;
        return std::make_shared<GoalCancellation>([weak](const UUID& id) {
            auto state = weak.lock();
            if (!state) throw std::runtime_error("controlled transport expired");
            CancelReply reply; bool terminal; std::function<void()> hook;
            std::shared_ptr<Record> record;
            {
                std::lock_guard<std::mutex> lock(state->mutex);
                state->canceled.push_back(id);
                reply = state->reply; terminal = state->terminate_on_cancel;
                hook = state->cancel_hook;
                for (const auto& request : state->records) if (request->id == id) record = request;
            }
            if (hook) hook();
            if (reply == CancelReply::kThrow) throw std::runtime_error("cancel transport throw");
            if (reply == CancelReply::kInvalid) return std::shared_future<bool>{};
            if (reply == CancelReply::kPending) {
                auto promise = std::make_shared<std::promise<bool>>();
                auto future = promise->get_future().share();
                std::lock_guard<std::mutex> lock(state->mutex);
                state->pending.push_back(promise); return future;
            }
            if (reply == CancelReply::kException) {
                std::promise<bool> promise;
                promise.set_exception(std::make_exception_ptr(std::runtime_error("cancel RPC exception")));
                return promise.get_future().share();
            }
            if (terminal && reply == CancelReply::kAccept && record) Finish(record);
            return Ack(reply == CancelReply::kAccept);
        });
    }
    /**
     * @brief Register UUID before optionally blocking the actual send call.
     */
    std::shared_future<std::shared_ptr<GoalHandle>> AsyncSendGoal(
        const Goal&, const SendGoalOptions& options) {
        auto record = std::make_shared<Record>(); record->options = options;
        auto future = record->accepted.get_future().share();
        {
            std::lock_guard<std::mutex> lock(state_->mutex);
            record->id[0] = static_cast<unsigned char>(state_->records.size() + 1);
            state_->records.push_back(record);
        }
        if (options.goal_request_callback) options.goal_request_callback(record->id);
        std::unique_lock<std::mutex> lock(state_->mutex);
        ++state_->entered; state_->wake.notify_all();
        if (state_->block_send && !state_->wake.wait_for(lock, 2500ms, [&] { return state_->released; }))
            throw std::runtime_error("controlled blocked send timed out");
        return future;
    }
private:
    std::shared_ptr<TransportState> state_;       ///< Transport fixture lifetime.
};
using Session = autonomy::task::navigation::ActionSession<DispatchProbeAction, ControlledClient>;

/**
 * @brief One independently enabled production gate with a controlled sender.
 */
struct Fixture {
    std::shared_ptr<TransportState> state = std::make_shared<TransportState>(); ///< RPC effects.
    ControlledClient client{state};             ///< Native-compatible sender boundary.
    std::shared_ptr<Gate> gate = std::make_shared<Gate>(); ///< Actual production ownership.
    Session session;                            ///< Actual production action session.
    Fixture() { Require(gate->Enable(), "enable fresh fixture gate"); }
    ~Fixture() { Release(state); }
    /**
     * @brief Commit one request with actual callback configuration.
     */
    void Begin() { Require(session.Begin(client, {}, gate), "commit fixture send"); }
    /**
     * @brief Poll real session completion, observing success separately.
     */
    BT::NodeStatus Tick(unsigned* success = nullptr) {
        return session.Tick({}, [success](const auto&) { if (success) ++*success; },
                            [](int, const std::string&) {});
    }
};

/**
 * @brief Closed admission cannot reach transport; unstarted tickets settle
          * locally.
 */
void Admission() {
    Fixture f; f.gate->Close();
    Require(!f.session.Begin(f.client, {}, f.gate) && Sends(f.state) == 0,
            "closed gate refuses network send");
    Require(f.gate->Enable(), "enable unused closed gate");
    unsigned cancels = 0;
    auto ticket = f.gate->Reserve([&] { ++cancels; }, [] { return false; });
    Require(static_cast<bool>(ticket), "reserve send before revocation");
    f.gate->Revoke();
    Require(!ticket->Start() && f.gate->Drained() && cancels == 0,
            "unstarted revoked reservation needs no remote terminal");
}
/**
 * @brief Cancellation must remain reachable while Begin holds its session
          * mutex.
 */
void BlockedLateAcceptance() {
    Fixture f; f.state->block_send = true;
    auto sending = std::async(std::launch::async, [&] { return f.session.Begin(f.client, {}, f.gate); });
    const bool entered = Entered(f.state);
    if (!entered) Release(f.state);
    Require(entered, "sender reached UUID notification boundary");
    f.gate->Revoke();
    Require(Cancels(f.state) == 0 && !f.gate->Drained(), "await uncertain acceptance before cancel");
    Accept(Request(f.state));
    const bool targeted = Cancels(f.state) == 1;
    const bool blocked = sending.wait_for(0ms) != std::future_status::ready;
    Release(f.state); Require(sending.get(), "pre-close committed send remains owned");
    Require(targeted && blocked, "late accept cancels without session mutex or send return");
    Require(!f.gate->Drained(), "cancel ACK alone cannot drain blocked send");
    Finish(Request(f.state));
    Require(f.gate->Drained(), "matching terminal drains revoked send");
}
/**
 * @brief ACK and terminal result are separate observable facts.
 */
void AckAndRevokedResult() {
    Fixture f; f.Begin(); Accept(Request(f.state));
    Require(f.Tick() == BT::NodeStatus::RUNNING, "accepted send executes");
    f.gate->Revoke();
    auto other_gate = std::make_shared<Gate>();
    Require(other_gate->Enable() && !f.session.NeedsResume(other_gate),
            "another task gate cannot resume retained action");
    Require(!f.gate->Enable() && !f.session.NeedsResume(f.gate),
            "cancel ACK cannot authorize resume before terminal");
    Require(Cancels(f.state) == 1 && !f.gate->Drained() && !f.gate->HasFault(),
            "positive ACK neither drains nor faults an awaiting goal");
    Finish(Request(f.state), Code::SUCCEEDED);
    unsigned success = 0;
    Require(f.gate->Drained() && f.Tick(&success) == BT::NodeStatus::FAILURE && success == 0,
            "revoked success cannot publish current action success");
}
/**
 * @brief Resume requires a matching terminal and a new owner revision.
 */
void RetainedResume() {
    Fixture f; f.Begin(); auto previous = Request(f.state); Accept(previous);
    Require(f.Tick() == BT::NodeStatus::RUNNING, "retained path running before pause");
    f.gate->Revoke(); Finish(previous);
    Require(!f.session.NeedsResume(f.gate), "closed drained gate is still paused");
    Require(f.gate->Enable() && f.session.NeedsResume(f.gate),
            "new revision after authoritative drain enables explicit continuation");
    f.Begin(); auto resumed = Request(f.state, 1); Accept(resumed);
    Require(!f.session.NeedsResume(f.gate) && f.Tick() == BT::NodeStatus::RUNNING,
            "fresh continuation is not repeatedly reissued");
    f.gate->Revoke(); Finish(resumed);
    Require(f.gate->Drained(), "resumed path retains independent cancellation ownership");
}

/**
 * @brief Pause during acceptance retains a real session through late
          * termination.
 */
void PausedTickRetainsIntent() {
    Fixture f; f.Begin(); auto old = Request(f.state);
    f.gate->Suspend(); f.gate->Revoke();
    Require(f.Tick() == BT::NodeStatus::RUNNING && f.session.is_busy(),
            "paused accepting tick retains intent without failing or resetting");
    Require(!f.session.NeedsResume(f.gate) && Sends(f.state) == 1,
            "paused acceptance cannot acquire fresh dispatch");
    Accept(old);
    Require(Cancels(f.state) == 1 && f.Tick() == BT::NodeStatus::RUNNING,
            "late acceptance is canceled while paused session remains retained");
    Finish(old);
    Require(f.gate->Drained() && f.gate->Enable() && f.session.NeedsResume(f.gate),
            "matching late terminal and owner resume preserve accepting intent");
    f.Begin(); auto resumed = Request(f.state, 1); Accept(resumed);
    Require(f.Tick() == BT::NodeStatus::RUNNING && Sends(f.state) == 2,
            "retained acceptance resumes with one new request");
    f.gate->Close(); f.gate->Revoke(); Finish(resumed);
}

/**
 * @brief Refused replacement admission cannot destroy a preceding running
          * action.
 */
void RefusedReplacementRetainsIntent() {
    Fixture f; f.Begin(); auto old = Request(f.state); Accept(old);
    Require(f.Tick() == BT::NodeStatus::RUNNING && f.session.has_goal_handle(),
            "preceding accepted action is held before replacement");
    f.gate->Suspend();
    Require(!f.session.Begin(f.client, {}, f.gate) && Sends(f.state) == 1 &&
                f.session.is_busy() && f.session.has_goal_handle(),
            "failed admission preserves preceding session and accepted handle");
    f.gate->Revoke(); unsigned success = 0;
    Finish(old, Code::SUCCEEDED);
    Require(f.Tick(&success) == BT::NodeStatus::RUNNING && success == 0,
            "terminal racing with pause retains intent without publishing success");
    Require(f.gate->Enable() && f.session.NeedsResume(f.gate),
            "refused replacement does not erase explicit resume eligibility");
    f.Begin(); auto resumed = Request(f.state, 1); Accept(resumed);
    Require(f.Tick() == BT::NodeStatus::RUNNING, "replacement retries after new revision");
    f.gate->Close(); f.gate->Revoke(); Finish(resumed);
}

/**
 * @brief Final cancellation and explicit halt supersede recoverable paused
          * state.
 */
void CancellationOverridesPause() {
    {
        Fixture f; f.Begin(); auto old = Request(f.state); Accept(old);
        Require(f.Tick() == BT::NodeStatus::RUNNING, "accepted action before paused cancel");
        f.gate->Suspend(); f.gate->Revoke();
        Require(f.Tick() == BT::NodeStatus::RUNNING, "pause initially retains intent");
        f.gate->Close();
        Require(f.Tick() == BT::NodeStatus::FAILURE && !f.session.is_busy() &&
                    !f.gate->Drained(),
                "cancel resets local intent without fabricating remote terminal");
        Finish(old); Require(f.gate->Drained(), "late terminal still settles canceled intent");
    }
    {
        Fixture f; f.Begin(); auto old = Request(f.state); Accept(old);
        f.gate->Suspend(); f.gate->Revoke();
        Require(f.session.Tick([] { return true; }, [](const auto&) {},
                    [](int, const std::string&) {}) == BT::NodeStatus::FAILURE &&
                    !f.session.is_busy(),
                "explicit halt checker takes precedence over suspension");
        Finish(old); Require(f.gate->Drained(), "explicit halt preserves goal cancellation debt");
    }
}
/**
 * @brief Cancel callbacks can synchronously query/reenter the gate.
 */
void ReentrantCancellation() {
    Fixture f; f.Begin(); Accept(Request(f.state));
    f.state->cancel_hook = [&] { f.gate->Close(); Require(!f.gate->Drained(), "reentrant pending drain"); };
    auto revoke = std::async(std::launch::async, [&] { f.gate->Revoke(); });
    Require(revoke.wait_for(1500ms) == std::future_status::ready, "cancel callback outside gate lock");
    revoke.get(); Finish(Request(f.state));
    Require(f.gate->Drained(), "reentrant cancellation preserves terminal ownership");
}
/**
 * @brief Observable cancel failure forbids another actual send.
 */
void CancellationFailures() {
    for (auto reply : {CancelReply::kReject, CancelReply::kException, CancelReply::kThrow,
                       CancelReply::kInvalid, CancelReply::kPending}) {
        Fixture f; f.state->reply = reply; f.Begin(); Accept(Request(f.state));
        f.gate->Revoke();
        if (reply == CancelReply::kPending) {
            Require(!f.gate->HasFault() && !f.gate->Drained(), "missing ACK is pending initially");
            Require(!f.gate->WaitForDrain(15ms), "missing ACK/terminal bounded by drain deadline");
        } else Require(f.gate->HasFault(), "cancel rejection/exception/invalid future is visible");
        Require(!f.session.Begin(f.client, {}, f.gate) && Sends(f.state) == 1,
                "cancel uncertainty prohibits new send");
        Finish(Request(f.state));
        Require(f.gate->Drained() && f.gate->HasFault() && !f.gate->Enable(),
                "terminal evidence does not silently clear sticky invocation fault");
    }
}
/**
 * @brief A null acceptance is not sufficient to prove rejection.
 */
void RejectionAndSendUncertainty() {
    {
        Fixture f; f.Begin(); auto record = Request(f.state);
        record->options.goal_rejection_callback(record->id);
        record->accepted.set_value(nullptr); record->options.goal_response_callback(nullptr);
        Require(f.gate->Drained() && f.Tick() == BT::NodeStatus::FAILURE,
                "explicit matching negative response settles rejected goal");
    }
    {
        Fixture f; f.Begin(); auto record = Request(f.state);
        auto error = std::make_exception_ptr(std::runtime_error("send response timeout"));
        record->options.goal_error_callback(record->id, error);
        record->accepted.set_exception(error); record->options.goal_response_callback(nullptr);
        f.gate->Revoke();
        Require(!f.gate->Drained() && f.gate->HasFault() && Cancels(f.state) == 1,
                "ambiguous null/timeout retains UUID and targeted cancel responsibility");
        // A later middleware result may still prove terminal despite lost acceptance.
        Wrapped terminal; terminal.goal_id = record->id; terminal.code = Code::CANCELED;
        record->options.result_callback(terminal);
        Require(f.gate->Drained(), "late authoritative terminal settles uncertain send");
    }
}
/**
 * @brief Wrong identities and nonterminal statuses cannot fabricate a drain.
 */
void ResultIdentity() {
    Fixture f; f.Begin(); auto record = Request(f.state); Accept(record);
    Wrapped wrong; wrong.goal_id = record->id; ++wrong.goal_id[1]; wrong.code = Code::CANCELED;
    record->options.result_callback(wrong);
    Require(!f.gate->Drained() && f.gate->HasFault(), "wrong UUID cannot settle owned movement");
    Wrapped terminal; terminal.goal_id = record->id; terminal.code = Code::CANCELED;
    record->options.result_callback(terminal);
    Require(f.gate->Drained(), "matching terminal remains usable after identity fault");
}
/**
 * @brief Terminal callbacks before accepted callbacks retain correct
          * ownership.
 */
void ResultBeforeAcceptance() {
    Fixture f; f.Begin(); auto record = Request(f.state);
    auto handle = Native::Handle(record->id, record->options);
    { std::lock_guard<std::mutex> lock(record->mutex); record->handle = handle; }
    Finish(record, Code::SUCCEEDED);
    record->accepted.set_value(handle); record->options.goal_response_callback(handle);
    unsigned success = 0;
    Require(f.gate->Drained() && f.Tick(&success) == BT::NodeStatus::SUCCESS && success == 1,
            "result-before-accept yields one matching success");
}
/**
 * @brief Replanning preserves server preemption while owning both pending
          * results.
 */
void RollingReplacement() {
    Fixture f; f.Begin(); auto old = Request(f.state); Accept(old);
    Require(f.Tick() == BT::NodeStatus::RUNNING, "first path accepted and running");
    f.Begin(); auto latest = Request(f.state, 1);
    Require(Cancels(f.state) == 0, "running same-gate replacement preserves server preemption");
    auto feedback = std::make_shared<DispatchProbeAction::Feedback>();
    old->options.feedback_callback(old->handle, feedback);
    Require(!f.session.has_feedback(), "old feedback cannot update replacement session");
    Accept(latest); Finish(old, Code::SUCCEEDED);
    Require(f.Tick() == BT::NodeStatus::RUNNING && !f.gate->Drained(),
            "old result cannot complete replacement or release current motion");
    f.gate->Revoke();
    { std::lock_guard<std::mutex> lock(f.state->mutex);
      Require(f.state->canceled.size() == 1 && f.state->canceled.front() == latest->id,
              "only unfinished current goal is canceled after old terminal"); }
    Finish(latest); Require(f.gate->Drained(), "all rolling movement obligations settle");
}
/**
 * @brief Dropping local session state does not drop late-accept cancellation.
 */
void DestroyedSession() {
    auto state = std::make_shared<TransportState>(); ControlledClient client(state);
    auto gate = std::make_shared<Gate>(); Require(gate->Enable(), "enable destruction fixture");
    { Session session; Require(session.Begin(client, {}, gate), "send before session destruction"); }
    gate->Revoke(); Accept(Request(state));
    Require(Cancels(state) == 1 && !gate->Drained(), "late acceptance after local destruction cancels");
    Finish(Request(state)); Require(gate->Drained(), "destroyed session retains terminal responsibility");
}
/**
 * @brief Revoked planning cannot block physical-motion drain or revive old
          * tickets.
 */
void PlanningAndRevision() {
    Fixture f; unsigned planning_cancels = 0;
    auto plan = f.gate->Reserve([&] { ++planning_cancels; }, [] { return true; }, Gate::Kind::kPlanning);
    Require(plan && plan->Start(), "commit planning request"); f.gate->Revoke();
    Require(planning_cancels == 1 && f.gate->Drained() && !f.gate->HasFault(),
            "planning cancellation independent of physical-motion drain/fault");
    Require(f.gate->Enable() && plan->Revoked() && !plan->Start(), "resume cannot revive old ticket");
    plan->Finish();
}

/**
 * @brief Local planning retirement keeps transport-owned late cancellation.
 */
void PlanningLateAcceptance() {
    auto state = std::make_shared<TransportState>(); ControlledClient client(state);
    auto gate = std::make_shared<Gate>(); Require(gate->Enable(), "enable planning fixture");
    auto cancellation = client.MakeGoalCancellation();
    auto ticket = gate->Reserve([cancellation] { cancellation->RequestCancel(); },
        [cancellation] { return cancellation->HasFailure(); }, Gate::Kind::kPlanning);
    Require(static_cast<bool>(ticket), "reserve planning transport");
    cancellation->SetTerminalCallback([ticket] { ticket->Finish(); });
    Native::SendGoalOptions options; cancellation->Configure(options);
    Require(ticket->Start(), "commit planning transport");
    auto accepted = client.AsyncSendGoal({}, options);
    cancellation->RequestCancel();
    Require(ticket->RetirePlanning(), "retire published local planning completion");
    ticket.reset(); cancellation.reset(); options = {};
    // Reserve prunes the retired registration and releases all gate captures.
    auto motion = gate->Reserve([] {}, [] { return false; });
    Require(motion && !motion->RetirePlanning(), "motion cannot retire without terminal");
    Accept(Request(state));
    Require(Cancels(state) == 1, "retired planning still cancels matching late acceptance");
    Finish(Request(state)); gate->Revoke();
    Require(gate->Drained(), "unstarted motion and retired planning leave no motion debt");
    Require(accepted.valid(), "planning transport response remains independently observable");
}

/**
 * @brief Local loss/destruction must never invent an ABORTED remote result.
 */
void NativeHandleLocalUncertainty() {
    UUID id{}; id[0] = 41;
    unsigned callbacks = 0; Native::SendGoalOptions options;
    options.result_callback = [&](const auto&) { ++callbacks; };
    {
        auto handle = Native::Handle(id, options);
        auto result = handle->AsyncGetResult();
        Native::ForgetAwareness(handle); handle.reset();
        Require(result.wait_for(0ms) == std::future_status::ready,
                "destroyed native handle settles local waiting future");
        bool uncertain = false; try { (void)result.get(); } catch (...) { uncertain = true; }
        Require(uncertain && callbacks == 0, "handle destruction supplies exception, not remote abort");
    }
    {
        auto handle = Native::Handle(id, options);
        auto result = handle->AsyncGetResult(); Native::Invalidate(handle);
        Require(result.wait_for(0ms) == std::future_status::ready && callbacks == 0 &&
                    !handle->IsCanceled() && !handle->IsAborted(),
                "invalidation is observable without fabricating terminal status");
        bool uncertain = false; try { (void)result.get(); } catch (...) { uncertain = true; }
        Require(uncertain, "native invalidation future reports local uncertainty");
        Wrapped terminal; terminal.goal_id = id; terminal.code = Code::CANCELED;
        terminal.result = std::make_shared<DispatchProbeAction::Result>();
        Native::Finish(handle, terminal); Native::Finish(handle, terminal);
        Require(callbacks == 1 && handle->IsCanceled(),
                "late matching remote terminal is delivered exactly once after invalidation");
    }
}

/**
 * @brief Terminal callbacks permit another thread to query the real handle.
 */
void NativeHandleTerminalDelivery() {
    UUID id{}; id[0] = 42;
    std::promise<void> callback_entered;
    auto entered = callback_entered.get_future().share();
    std::promise<void> query_finished;
    auto queried = query_finished.get_future();
    bool query_during_callback = false; unsigned callbacks = 0;
    Native::SendGoalOptions options;
    options.result_callback = [&](const auto&) {
        ++callbacks; callback_entered.set_value();
        query_during_callback = queried.wait_for(1500ms) == std::future_status::ready;
    };
    auto handle = Native::Handle(id, options);
    auto result = handle->AsyncGetResult();
    std::thread observer([&] {
        entered.wait(); (void)handle->GetGoalStatus(); query_finished.set_value();
    });
    Wrapped terminal; terminal.goal_id = id; terminal.code = Code::CANCELED;
    terminal.result = std::make_shared<DispatchProbeAction::Result>();
    Native::Finish(handle, terminal); observer.join();
    Require(query_during_callback && callbacks == 1 && result.get().code == Code::CANCELED,
            "actual terminal callback runs outside native handle mutex");
    Native::Status(handle, autolink::action::GoalStatus::EXECUTING);
    Native::Finish(handle, terminal);
    Require(handle->IsCanceled() && callbacks == 1,
            "stale status and duplicate result cannot regress real terminal evidence");
}

/**
 * @brief Genuine native worker node with the production action polling path.
 */
class DispatchNode final : public BT::StatefulActionNode {
public:
    DispatchNode(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config),
          state_(config.blackboard->get<std::shared_ptr<TransportState>>("dispatch_transport")),
          client_(state_) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus onStart() override {
        gate_ = config().blackboard->get<std::shared_ptr<Gate>>(
            autonomy::task::common::kMotionDispatchGateBlackboardKey);
        { std::lock_guard<std::mutex> lock(state_->mutex); state_->observed_gate = gate_; }
        return session_.Begin(client_, {}, gate_) ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
    }
    BT::NodeStatus onRunning() override {
        {
            std::unique_lock<std::mutex> lock(state_->mutex);
            if (state_->block_tick) {
                ++state_->tick_entered; state_->wake.notify_all();
                if (!state_->wake.wait_for(lock, 2500ms, [&] { return state_->tick_released; }))
                    throw std::runtime_error("controlled native tick timed out");
            }
        }
        BT::NodeStatus status;
        if (session_.NeedsResume(gate_))
            status = session_.Begin(client_, {}, gate_) ? BT::NodeStatus::RUNNING
                                                       : BT::NodeStatus::FAILURE;
        else status = session_.Tick({}, [](const auto&) {}, [](int, const std::string&) {});
        {
            std::lock_guard<std::mutex> lock(state_->mutex);
            if (state_->block_tick) {
                ++state_->tick_returned; state_->last_tick_status = status;
                state_->wake.notify_all();
            }
        }
        return status;
    }
    void onHalted() override { session_.Cancel(client_); }
private:
    std::shared_ptr<TransportState> state_;       ///< Observed effects outlive the worker node.
    ControlledClient client_;                    ///< Only RPC timing is controlled.
    std::shared_ptr<Gate> gate_;                  ///< Gate installed by production NavigationTask.
    Session session_;                            ///< Actual production acceptance/result polling.
};
/**
 * @brief Register the fixture node through the actual native factory
          * registry.
 */
void Register(BT::BehaviorTreeFactory& factory) { factory.registerNodeType<DispatchNode>("DispatchProbe"); }
/**
 * @brief Own a real XML tree until every task worker has joined.
 */
class Xml {
public:
    Xml() {
        path_ = std::filesystem::temp_directory_path() /
            ("autonomy-motion-dispatch-" + std::to_string(
                std::chrono::steady_clock::now().time_since_epoch().count()) + ".xml");
        std::ofstream stream(path_);
        stream << "<root BTCPP_format=\"4\" main_tree_to_execute=\"Test\"><BehaviorTree ID=\"Test\">"
                  "<DispatchProbe/></BehaviorTree></root>";
        Require(static_cast<bool>(stream), "write dispatch XML");
    }
    ~Xml() { std::error_code error; std::filesystem::remove(path_, error); }
    std::string path() const { return path_.string(); }
private:
    std::filesystem::path path_;                 ///< Unique fixture file only.
};
/**
 * @brief Native task with real default cancellation/drain; no cancel
          * override.
 */
class TaskHarness final : public autonomy::task::NavigationTask {
public:
    std::shared_ptr<TransportState> state = std::make_shared<TransportState>(); ///< RPC fixture.
    /// Observe completion only after the production terminal cleanup has returned.
    bool done() { return !runner()->IsRunning(); }
    /// Keep explicit terminal handshakes bounded without relying on a 100ms race.
    std::chrono::milliseconds cancel_timeout{100ms};
    ~TaskHarness() override { Release(state); try { Shutdown(); } catch (...) {} }
protected:
    bool EnsureNavigationClient() override { return true; }
    bool NavigationServersReady() const override { return true; }
    std::chrono::milliseconds NavigationReadyTimeout() const override { return 100ms; }
    std::chrono::milliseconds NavigationCancelTimeout() const override { return cancel_timeout; }
    void PopulateBlackboard(const BT::Blackboard::Ptr& board) override {
        NavigationTask::PopulateBlackboard(board); board->set("dispatch_transport", state);
    }
};
/**
 * @brief Construct a valid native navigation request and unique task
          * identity.
 */
proto::NavigationGoal Goal(const Xml& xml, const std::string& id = "dispatch-A") {
    proto::NavigationGoal goal; goal.set_command(proto::NAV_CMD_START);
    goal.mutable_header()->set_task_id(id); goal.mutable_header()->set_client_id("exploration-parent");
    goal.mutable_plugins()->set_behavior_tree(xml.path());
    auto* pose = goal.add_goals(); pose->mutable_header()->set_frame_id("map");
    pose->mutable_pose()->mutable_position()->set_x(2);
    pose->mutable_pose()->mutable_orientation()->set_w(1);
    return goal;
}
/**
 * @brief Initialize, accept and observe a production worker's actual
          * transport call.
 */
void Start(TaskHarness& task, const Xml& xml) {
    proto::TaskServerOptions options;
    Require(task.Initialize(options), "initialize native dispatch task");
    Require(task.SubmitGoal(Goal(xml)), "native dispatch task accepts request");
    Require(Entered(task.state), "native worker commits transport send");
}
/**
 * @brief Native Cancel reaches a blocked send before join and waits terminal
          * evidence.
 */
void NativeBlockedCancellation() {
    Xml xml; TaskHarness task; task.state->block_send = true;
    task.state->terminate_on_cancel = true; Start(task, xml);
    auto cancel = std::async(std::launch::async, [&] { return task.Cancel(); });
    auto gate = task.state->observed_gate.lock();
    const bool revoked = Await([&] { return gate && !gate->Reserve([] {}, [] { return false; }); });
    if (!revoked) Release(task.state);
    Require(revoked, "native cancel closes admission while worker send remains blocked");
    Accept(Request(task.state));
    const bool targeted = Cancels(task.state) == 1;
    Release(task.state);
    Require(cancel.wait_for(1500ms) == std::future_status::ready && cancel.get() && targeted,
            "native cancel uses late UUID acceptance without blocked session lock");
    Require(gate->Drained(), "native cancel observed matching terminal");
}
/**
 * @brief Pause cancels execution and resume must issue a fresh owned request.
 */
void NativePauseResume() {
    Xml xml; TaskHarness task; task.state->terminate_on_cancel = true; Start(task, xml);
    Accept(Request(task.state));
    Require(task.Pause(), "native pause drains actual action");
    Require(Cancels(task.state) == 1, "native pause targeted one action");
    Require(task.Resume(), "native resume accepts new ownership revision");
    Require(Await([&] { return Sends(task.state) == 2; }), "resume issues fresh action after old ticket revoked");
    Accept(Request(task.state, 1));
    Require(task.Cancel(), "resumed task cancels with genuine terminal result");
}

/**
 * @brief Pause after entry into an actual node must not lose its resumable
          * session.
 */
void NativePauseDuringTick() {
    Xml xml; TaskHarness task;
    task.state->terminate_on_cancel = true; task.state->block_tick = true;
    Start(task, xml); Accept(Request(task.state));
    Require(EnteredTick(task.state), "native node executing after worker admission checks");
    Require(task.Pause(), "pause can drain motion while a native node is still executing");
    auto gate = task.state->observed_gate.lock();
    Require(gate && gate->Suspended() && gate->Drained() && Sends(task.state) == 1,
            "executing paused node receives no fresh send before release");
    ReleaseTick(task.state);
    Require(Await([&] {
        std::lock_guard<std::mutex> lock(task.state->mutex);
        return task.state->tick_returned != 0;
    }), "already executing native tick returned after pause");
    {
        std::lock_guard<std::mutex> lock(task.state->mutex);
        Require(task.state->last_tick_status == BT::NodeStatus::RUNNING,
                "paused executing tick cannot report failure or success");
    }
    Require(task.Resume() && Await([&] { return Sends(task.state) == 2; }),
            "actual node retains session and resends after native resume");
    Accept(Request(task.state, 1)); Require(task.Cancel(), "resumed native node retains cancellation");
}
/**
 * @brief Failed cancellation blocks new START/Resume until explicit drained
          * reinit.
 */
void NativeCancellationFault() {
    Xml xml; TaskHarness task; task.state->reply = CancelReply::kReject; Start(task, xml);
    Accept(Request(task.state));
    bool failure = false; try { task.Cancel(); } catch (...) { failure = true; }
    Require(failure && !task.Resume() && !task.SubmitGoal(Goal(xml, "dispatch-B")),
            "native cancel fault prohibits resume and replacement sends");
    proto::TaskServerOptions options;
    Require(!task.Initialize(options), "reinitialize cannot discard unresolved motion");
    Finish(Request(task.state));
    task.state->reply = CancelReply::kAccept; task.state->terminate_on_cancel = true;
    Require(task.Initialize(options), "explicit initialization after real terminal can establish new owner");
    Require(task.SubmitGoal(Goal(xml, "dispatch-C")) && Entered(task.state, 2),
            "fresh owner sends only after prior goal terminal");
    Accept(Request(task.state, 1)); Require(task.Cancel(), "fresh owner cancellation remains functional");
}

/**
 * @brief A guarded native send still requires its actual terminal after
          * cancel ACK.
 */
void NativePermitCancellationDrain() {
    Xml xml; TaskHarness task; task.cancel_timeout=500ms;
    auto allowed=std::make_shared<std::atomic<bool>>(true);
    proto::TaskServerOptions options;
    Require(task.Initialize(options) && task.SubmitGoalWithMotionPermit(
        Goal(xml),[allowed]{return allowed->load();}) && Entered(task.state),
        "actual guarded task commits one owned transport request");
    auto request=Request(task.state); Accept(request);
    auto gate=task.state->observed_gate.lock();
    allowed->store(false);
    Require(Await([&]{return Cancels(task.state)==1;}) && gate &&
            !gate->Drained() && !task.done() && Sends(task.state)==1,
            "external revocation targets one UUID and cancel ACK cannot drain motion");
    Finish(request);
    // Revocation before a tick cancels the runner; revocation after admission
    // may make the actual action node fail. Neither may publish success.
    Require(Await([&]{return task.done();}) && gate->Drained() &&
            (task.GetLifecycle()==autonomy::task::TaskLifecycle::kCanceled ||
             task.GetLifecycle()==autonomy::task::TaskLifecycle::kFailed),
            "matching native terminal releases guarded invocation cleanup");
    allowed->store(true);
    Require(!gate->Enable(), "restored parent query cannot revive completed denied gate");
    proto::NavigationResult result;
    Require(task.GetResultForTask("dispatch-A",&result) &&
            !task.GetResultForTask("dispatch-stale",&result), "drained result keeps native identity");
}

/**
 * @brief Owner loss during an admitted node tick is checked by its action
          * session.
 */
void NativePermitDuringTick() {
    Xml xml; TaskHarness task; task.state->terminate_on_cancel=true;
    task.state->block_tick=true;
    auto allowed=std::make_shared<std::atomic<bool>>(true);
    proto::TaskServerOptions options;
    Require(task.Initialize(options) && task.SubmitGoalWithMotionPermit(
        Goal(xml),[allowed]{return allowed->load();}) && Entered(task.state),
        "start actual guarded node for admitted tick");
    Accept(Request(task.state));
    const bool entered=EnteredTick(task.state);
    allowed->store(false); ReleaseTick(task.state);
    Require(entered && Await([&]{return task.done();}), "admitted native tick ends after owner loss");
    auto gate=task.state->observed_gate.lock();
    Require(gate && gate->Drained() && Cancels(task.state)==1 && Sends(task.state)==1 &&
            task.GetLifecycle()!=autonomy::task::TaskLifecycle::kSucceeded,
            "action-session boundary honors parent guard without fresh sends or success");
}
}  // namespace

/**
 * @brief Run actual ownership scenarios; emit one named result per contract.
 */
int main() {
    autonomy::task::AddBtNodeRegistrar(&Register);
    try {
        const std::vector<std::pair<const char*, std::function<void()>>> tests = {
            {"closed and unstarted admission", Admission},
            {"blocked send and late acceptance", BlockedLateAcceptance},
            {"ACK versus terminal and revoked success", AckAndRevokedResult},
            {"retained action fresh resume", RetainedResume},
            {"paused acceptance retains session", PausedTickRetainsIntent},
            {"refused replacement retains session", RefusedReplacementRetainsIntent},
            {"final cancellation supersedes pause", CancellationOverridesPause},
            {"cancel callback reentry", ReentrantCancellation},
            {"cancel rejection exception invalid pending", CancellationFailures},
            {"explicit rejection versus send uncertainty", RejectionAndSendUncertainty},
            {"authoritative result identity", ResultIdentity},
            {"result before acceptance", ResultBeforeAcceptance},
            {"rolling replacement and stale callbacks", RollingReplacement},
            {"destroyed session and late acceptance", DestroyedSession},
            {"planning drain and fresh revision", PlanningAndRevision},
            {"retired planning late acceptance", PlanningLateAcceptance},
            {"native handle local uncertainty", NativeHandleLocalUncertainty},
            {"native handle terminal callback and stale status", NativeHandleTerminalDelivery},
            {"native cancel during blocked send", NativeBlockedCancellation},
            {"native pause resume fresh send", NativePauseResume},
            {"native pause during executing tick", NativePauseDuringTick},
            {"native fault and drained reinitialization", NativeCancellationFault},
            {"native guarded cancellation ACK versus terminal", NativePermitCancellationDrain},
            {"native guarded owner loss during admitted tick", NativePermitDuringTick}
        };
        for (const auto& test : tests) { test.second(); std::cout << "PASS " << test.first << std::endl; }
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << std::endl; return 1;
    }
    return 0;
}
