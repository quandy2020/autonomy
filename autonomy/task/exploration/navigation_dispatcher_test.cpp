/* Copyright 2026 The Openbot Authors. */
/** @file navigation_dispatcher_test.cpp @brief Native acceptance/cancel regressions. */
#include <chrono>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>

#include "autonomy/task/exploration/integration/navigation_dispatcher.hpp"

namespace {
namespace e = autonomy::task::exploration;
using Clock = std::chrono::steady_clock;

/** @brief Fail a check without depending on release-disabled assert macros. */
void Require(bool condition, const char* reason) {
    if (!condition) throw std::runtime_error(reason);
}

/** @brief A bounded wait keeps failures distinct from a permanent test hang. */
void Until(const std::function<bool()>& condition, const char* reason) {
    const auto deadline = Clock::now() + std::chrono::seconds(3);
    while (!condition()) {
        if (Clock::now() >= deadline) throw std::runtime_error(reason);
        std::this_thread::yield();
    }
}

/** @brief Real dispatcher input with different native and algorithm identities. */
e::RequestVersion Version(std::uint64_t request) {
    e::RequestVersion result;
    result.session_id = 7;
    result.request_id = request;
    result.goal_version = request;
    result.map_epoch = 2;
    return result;
}

/** @brief Locked backend records native acceptance separately from owner state. */
class Backend {
public:
    /** @brief Deliberately permit a late native accept to exercise cancel fallback. */
    bool Submit(const e::NavigationDispatchRequest& request,
                const e::MotionGate::Permit&) {
        std::unique_lock<std::mutex> lock(mutex);
        ++submissions;
        ids.insert(request.task_id);
        condition.notify_all();
        condition.wait(lock, [this] { return !blocked; });
        if (throw_submit) {
            outstanding.insert(request.task_id);
            throw std::runtime_error("native accepted then threw");
        }
        if (reject_submit) return false;
        outstanding.insert(request.task_id);
        return true;
    }

    /** @brief Return only the named native result; no active-bool inference. */
    std::optional<bool> Result(const std::string& id) {
        std::lock_guard<std::mutex> lock(mutex);
        if (throw_result) throw std::runtime_error("result read unavailable");
        const auto found = replies.find(id);
        if (found == replies.end()) return {};
        outstanding.erase(id);
        return found->second;
    }

    /** @brief Conditional cancellation records each ID and optional failure. */
    void Cancel(const std::string& id) {
        std::lock_guard<std::mutex> lock(mutex);
        ++cancellations;
        ++canceled_ids[id];
        condition.notify_all();
        if (throw_cancel) throw std::runtime_error("native cancellation unconfirmed");
        outstanding.erase(id);
    }

    /** @brief Wait for a native submission or cancellation on its actual event. */
    void Wait(std::size_t submitted, std::size_t canceled = 0) {
        std::unique_lock<std::mutex> lock(mutex);
        const bool reached = condition.wait_for(lock, std::chrono::seconds(3), [=] {
            return submissions >= submitted && cancellations >= canceled;
        });
        if (!reached) throw std::runtime_error("native event missing: expected submit/cancel " +
            std::to_string(submitted) + "/" + std::to_string(canceled) +
            ", actual " + std::to_string(submissions) + "/" + std::to_string(cancellations) +
            ", outstanding " + std::to_string(outstanding.size()));
    }

    /** @brief Release a blocked submit even while an exception unwinds a test. */
    void Release() {
        std::lock_guard<std::mutex> lock(mutex);
        blocked = false;
        condition.notify_all();
    }

    /// All callback/test shared values have one explicit synchronization domain.
    std::mutex mutex;
    /// Signals real backend events rather than scheduling assumptions.
    std::condition_variable condition;
    /// Accept callback can wait until the owner demonstrates nonblocking cancel.
    bool blocked = false;
    /// Native service may reject a request without accepting motion.
    bool reject_submit = false;
    /// Exception occurs after acceptance, requiring identity-aware cancellation.
    bool throw_submit = false;
    /// Result transport failure leaves motion ambiguous until cancellation.
    bool throw_result = false;
    /// Failure must permanently inhibit further native submissions.
    bool throw_cancel = false;
    /// Callback counters are never read without mutex.
    std::size_t submissions = 0, cancellations = 0;
    /// Accepted native motions; shutdown must leave this empty on success.
    std::set<std::string> outstanding, ids;
    /// Matching terminal replies, including deliberately stale IDs.
    std::map<std::string, bool> replies;
    /// Each accepted/canceled native ID is canceled no more than once.
    std::map<std::string, std::size_t> canceled_ids;
};

/** @brief Release backend waits before the dispatcher destructor joins. */
struct ReleaseOnExit {
    /// Referenced backend outlives this guard and the dispatcher.
    Backend& backend;
    ~ReleaseOnExit() { backend.Release(); }
};

/** @brief Common backend closures leave dispatcher locking visible to tests. */
std::unique_ptr<e::NavigationDispatcher> Make(
    Backend& backend, const std::shared_ptr<e::MotionGate>& gate) {
    return std::make_unique<e::NavigationDispatcher>(gate, "boot/task/",
        [&backend](const auto& request, const auto& permit) { return backend.Submit(request, permit); },
        [&backend](const auto& id) { return backend.Result(id); },
        [&backend](const auto& id) { backend.Cancel(id); });
}

/** @brief Resume issues a new lease without resurrecting paused work. */
void LeaseRevocation() {
    auto gate = std::make_shared<e::MotionGate>();
    auto disabled = gate->Lease();
    Require(!disabled(), "a new gate is disabled");
    gate->SetEnabled(true);
    auto initial = gate->Lease();
    Require(initial() && !disabled(), "enable cannot revive a disabled lease");
    gate->SetEnabled(false);
    Require(!initial(), "pause revokes existing leases");
    gate->SetEnabled(true);
    auto resumed = gate->Lease();
    Require(resumed() && !initial(), "resume does not revive the old generation revision");
    gate->Retire();
    gate->SetEnabled(true);
    Require(!resumed() && !gate->Lease()(), "retirement is permanent");
}

/** @brief Cancel returns while native accept blocks; late accept is canceled once. */
void CancelBlockedAccept() {
    Backend backend;
    backend.blocked = true;
    auto gate = std::make_shared<e::MotionGate>(); gate->SetEnabled(true);
    auto dispatcher = Make(backend, gate);
    ReleaseOnExit cleanup{backend};
    const auto request = Version(1);
    Require(dispatcher->Enqueue({1, 2, .3}, request), "request queues");
    backend.Wait(1);
    Require(!dispatcher->Poll(), "reserved submission is not completion");
    Require(dispatcher->Enqueue({1, 2, .3}, request), "same payload deduplicates");
    Require(!dispatcher->Enqueue({9, 2, .3}, request), "same ID cannot change the payload");
    dispatcher->CancelRequest(request);
    backend.Release();
    backend.Wait(1, 1);
    dispatcher->Shutdown();
    std::lock_guard<std::mutex> lock(backend.mutex);
    Require(backend.submissions == 1 && backend.cancellations == 1 && backend.outstanding.empty(),
            "late native acceptance receives exactly one cancellation");
    Require(!dispatcher->Poll(), "canceled arrival cannot finish another request");
}

/** @brief Accept/cancel handoff is exercised repeatedly without artificial hooks. */
void AcceptCancelHandoff() {
    Backend backend;
    auto gate = std::make_shared<e::MotionGate>(); gate->SetEnabled(true);
    std::atomic<std::uint64_t> accepted{0};
    e::NavigationDispatcher dispatcher(gate, "boot/task/",
        [&](const auto& request, const auto& permit) {
            const auto result = backend.Submit(request, permit);
            // A native acceptance event is published immediately before its
            // callback returns, rather than waiting for a sleeping CV reader.
            accepted.store(request.version.request_id, std::memory_order_release);
            return result;
        }, [&](const auto& id) { return backend.Result(id); },
        [&](const auto& id) { backend.Cancel(id); });
    for (std::uint64_t i = 1; i <= 5000; ++i) {
        const auto request = Version(i);
        Require(dispatcher.Enqueue({1, 2, 0}, request), "handoff request queues");
        const auto deadline = Clock::now() + std::chrono::seconds(3);
        while (accepted.load(std::memory_order_acquire) < i)
            Require(Clock::now() < deadline, "native acceptance was not published");
        dispatcher.CancelRequest(request);
        backend.Wait(i, i);
    }
    dispatcher.Shutdown();
    std::lock_guard<std::mutex> lock(backend.mutex);
    Require(backend.outstanding.empty() && backend.cancellations == 5000,
            "accept/cancel handoff never leaves native motion outstanding");
    for (const auto& entry : backend.canceled_ids)
        Require(entry.second == 1, "each native cancellation has one owner");
}

/** @brief Results and rejection are read only for the currently owned identity. */
void MatchingResultsAndRejection() {
    Backend backend;
    auto gate = std::make_shared<e::MotionGate>(); gate->SetEnabled(true);
    auto dispatcher = Make(backend, gate);
    auto first = Version(1);
    Require(dispatcher->Enqueue({1, 1, 0}, first), "first queues"); backend.Wait(1);
    dispatcher->CancelRequest(first); backend.Wait(1, 1);
    auto second = Version(2);
    Require(dispatcher->Enqueue({1, 1, 0}, second), "second identity queues at the same pose");
    backend.Wait(2);
    {
        std::lock_guard<std::mutex> lock(backend.mutex);
        backend.replies["boot/task/7/1"] = true;
    }
    Require(!dispatcher->Poll(), "old native result does not complete current identity");
    {
        std::lock_guard<std::mutex> lock(backend.mutex);
        backend.replies["boot/task/7/2"] = true;
    }
    std::optional<e::NavigationReply> reply;
    Until([&] { reply = dispatcher->Poll(); return reply.has_value(); }, "matching result not delivered");
    Require(reply->success && reply->version.SameRequest(second), "result preserves the owner version");
    {
        std::lock_guard<std::mutex> lock(backend.mutex);
        backend.reject_submit = true;
    }
    Require(dispatcher->Enqueue({1, 1, 0}, Version(3)), "rejected native request still queues");
    Until([&] { reply = dispatcher->Poll(); return reply.has_value(); }, "native rejection not delivered");
    Require(!reply->success && reply->version.SameRequest(Version(3)), "native rejection is not arrival");
    dispatcher->Shutdown();
}

/** @brief Exceptions after native accept and during result reads cancel by ID. */
void AmbiguousCallbacks() {
    for (bool submit_exception : {true, false}) {
        Backend backend;
        backend.throw_submit = submit_exception;
        backend.throw_result = !submit_exception;
        auto gate = std::make_shared<e::MotionGate>(); gate->SetEnabled(true);
        auto dispatcher = Make(backend, gate);
        Require(dispatcher->Enqueue({0, 0, 0}, Version(1)), "exception scenario queues");
        std::optional<e::NavigationReply> reply;
        Until([&] { reply = dispatcher->Poll(); return reply.has_value(); }, "exception failure reply missing");
        Require(!reply->success, "callback exception cannot certify arrival");
        backend.Wait(1, 1);
        dispatcher->Shutdown();
        std::lock_guard<std::mutex> lock(backend.mutex);
        Require(backend.outstanding.empty() && backend.cancellations == 1,
                "ambiguous accepted motion canceled exactly once");
    }
}

/** @brief Cancellation failure retires the gate and prevents any further START. */
void CancelFailureRetiresGeneration() {
    Backend backend; backend.throw_cancel = true;
    auto gate = std::make_shared<e::MotionGate>(); gate->SetEnabled(true);
    auto dispatcher = Make(backend, gate);
    Require(dispatcher->Enqueue({0, 0, 0}, Version(1)), "cancel-failure request queues");
    backend.Wait(1);
    dispatcher->CancelRequest(Version(1)); backend.Wait(1, 1);
    Until([&] { return !gate->Enabled(); }, "cancel failure did not retire motion gate");
    Require(dispatcher->Failed(), "unconfirmed cancellation is observable by the owner");
    gate->SetEnabled(true);
    Require(!dispatcher->Enqueue({0, 0, 0}, Version(2)), "cancel failure forbids further native starts");
    dispatcher->Shutdown();
}

/** @brief Worker shutdown reentry cannot wait on an external join of itself. */
void ShutdownReentryAndLateAcceptance() {
    Backend backend; backend.blocked = true;
    auto gate = std::make_shared<e::MotionGate>(); gate->SetEnabled(true);
    e::NavigationDispatcher* callback_owner = nullptr;
    e::NavigationDispatcher dispatcher(gate, "boot/task/",
        [&](const auto& request, const auto& permit) {
            const auto accepted = backend.Submit(request, permit);
            callback_owner->Shutdown();
            return accepted;
        }, [&](const auto& id) { return backend.Result(id); },
        [&](const auto& id) { backend.Cancel(id); });
    callback_owner = &dispatcher;
    ReleaseOnExit cleanup{backend};
    Require(dispatcher.Enqueue({0, 0, 0}, Version(1)), "shutdown case queues");
    backend.Wait(1);
    std::thread external([&] { dispatcher.Shutdown(); });
    backend.Release();
    external.join();
    dispatcher.Shutdown();
    std::lock_guard<std::mutex> lock(backend.mutex);
    Require(backend.cancellations == 1 && backend.outstanding.empty(),
            "reentrant shutdown drains late accepted native motion before join");
}

/** @brief Pausing a submit revokes its captured lease even after resume. */
void PauseDuringSubmission() {
    Backend backend; backend.blocked = true;
    auto gate = std::make_shared<e::MotionGate>(); gate->SetEnabled(true);
    auto dispatcher = Make(backend, gate);
    ReleaseOnExit cleanup{backend};
    Require(dispatcher->Enqueue({0, 0, 0}, Version(1)), "pause case queues"); backend.Wait(1);
    gate->SetEnabled(false); gate->SetEnabled(true);
    backend.Release(); backend.Wait(1, 1);
    Require(dispatcher->Enqueue({0, 0, 0}, Version(2)), "resume issues a new request lease");
    backend.Wait(2);
    dispatcher->CancelRequest(Version(2)); backend.Wait(2, 2);
    dispatcher->Shutdown();
    std::lock_guard<std::mutex> lock(backend.mutex);
    Require(backend.outstanding.empty(), "pause/resume cannot resurrect a late native acceptance");
}
}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--handoff-only") {
            AcceptCancelHandoff();
            std::cout << "PASS 5000_accept_cancel_handoffs\n";
            return 0;
        }
        LeaseRevocation(); std::cout << "PASS lease_pause_resume_retirement\n";
        CancelBlockedAccept(); std::cout << "PASS blocked_accept_cancel_dedup\n";
        AcceptCancelHandoff(); std::cout << "PASS 5000_accept_cancel_handoffs\n";
        MatchingResultsAndRejection(); std::cout << "PASS identity_results_rejection\n";
        AmbiguousCallbacks(); std::cout << "PASS ambiguous_submit_result_exceptions\n";
        CancelFailureRetiresGeneration(); std::cout << "PASS cancel_failure_stops_new_motion\n";
        ShutdownReentryAndLateAcceptance(); std::cout << "PASS shutdown_reentry_late_accept\n";
        PauseDuringSubmission(); std::cout << "PASS pause_resume_submission\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL navigation_dispatcher: " << error.what() << '\n';
        return 1;
    }
}
