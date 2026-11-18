/* Copyright 2026 The Openbot Authors. */
/**
 * @file motion_dispatch_gate_test.cpp
 * @brief Concurrent admission, terminal ownership and bounded planning lifetime.
 *
 * Exercises the production gate with real threads. No transport, action session
 * or robot is modeled here; those require the separate native dispatch suite.
 */
#include "autonomy/task/common/motion_dispatch_gate.hpp"

#include <atomic>
#include <chrono>
#include <future>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>

namespace {
using Gate = autonomy::task::common::MotionDispatchGate;
using namespace std::chrono_literals;

/** @brief Report the exact gate invariant which failed. */
void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

/** @brief Verify that only a genuine terminal can release movement ownership. */
void TerminalAndResume() {
    Gate gate;
    Require(!gate.Reserve([] {}, [] { return false; }), "gate initially disabled");
    Require(gate.Enable(), "enable fresh gate");
    unsigned cancels = 0;
    auto old = gate.Reserve([&] { ++cancels; }, [] { return false; });
    Require(old && old->Start() && !old->Start(), "commit exactly once");
    Require(!old->RetirePlanning(), "motion cannot retire locally");
    gate.Revoke();
    Require(cancels == 1 && !gate.Drained() && !gate.Enable(),
            "cancel initiation is not terminal evidence");
    Require(!old->HasResumedRevision(), "no resumed revision before drain");
    old->Finish();
    Require(gate.Drained() && gate.Enable() && old->HasResumedRevision(),
            "new revision enabled only after terminal");
    Require(old->Revoked() && !old->Start(), "old ticket never regains dispatch");
    gate.Close();
    Require(!old->HasResumedRevision(), "closed resumed gate grants no continuation");
}

/** @brief Pause preserves intent without granting sends; final close supersedes it. */
void SuspensionAndCancellation() {
    Gate gate;
    Require(gate.Enable(), "enable suspension fixture");
    unsigned cancels = 0;
    auto running = gate.Reserve([&] { ++cancels; }, [] { return false; });
    auto unstarted = gate.Reserve([&] { ++cancels; }, [] { return false; });
    Require(running && running->Start() && unstarted, "reserve active and pending sends");
    gate.Suspend();
    Require(gate.Suspended() && running->Suspended() && running->Revoked(),
            "pause retains intent but revokes old dispatch");
    Require(!gate.Reserve([] {}, [] { return false; }) && !unstarted->Start(),
            "paused gate refuses reservations and prior uncommitted sends");
    gate.Revoke();
    Require(cancels == 1 && gate.Suspended() && !gate.Drained() && !gate.Enable(),
            "pause cancellation retains marker and awaits real terminal");
    running->Finish();
    Require(gate.Enable() && !gate.Suspended() && running->HasResumedRevision(),
            "resumed revision follows terminal and clears suspension");
    gate.Suspend();
    Require(running->Suspended(), "a second pause retains not-yet-resent intent");
    gate.Close(); gate.Revoke();
    Require(!gate.Suspended() && !running->Suspended(),
            "cancel close overrides a paused retained action");

    Require(gate.Enable(), "enable fault suspension fixture");
    auto failed = gate.Reserve([] {}, [] { return true; });
    Require(failed && failed->Start(), "commit fault suspension action");
    gate.Suspend(); gate.Revoke();
    Require(gate.HasFault() && !gate.Suspended() && !failed->Suspended(),
            "transport uncertainty supersedes paused retention");
    failed->Finish();
    Require(!gate.Enable(), "terminal cannot erase a paused cancellation fault");
}

/** @brief Racing pause and send commitment retains exactly one cancellation owner. */
void ConcurrentPauseAndCommit() {
    for (unsigned i = 0; i < 256; ++i) {
        Gate gate;
        Require(gate.Enable(), "enable pause race fixture");
        std::atomic<unsigned> cancels{0};
        auto ticket = gate.Reserve([&] { ++cancels; }, [] { return false; });
        Require(static_cast<bool>(ticket), "reserve pause race send");
        std::promise<void> release;
        auto barrier = release.get_future().share();
        bool committed = false;
        std::thread sender([&] { barrier.wait(); committed = ticket->Start(); });
        std::thread pauser([&] { barrier.wait(); gate.Suspend(); gate.Revoke(); });
        release.set_value(); sender.join(); pauser.join();
        Require(gate.Suspended() && ticket->Suspended() && !ticket->Start(),
                "concurrent pause preserves intent without recommitting old send");
        Require(cancels.load() == (committed ? 1u : 0u),
                "only committed pause-racing sends require cancellation");
        Require(gate.Drained() == !committed,
                "committed send still needs terminal evidence while paused");
        if (committed) ticket->Finish();
        Require(gate.Enable() && ticket->HasResumedRevision(),
                "racing pause resumes only after owned motion settles");
        gate.Close();
        Require(!ticket->Suspended(), "later cancel cannot remain a recoverable pause");
    }
}

/** @brief Start and revoke racing on real threads cannot lose cancel ownership. */
void ConcurrentCommitAndRevoke() {
    for (unsigned i = 0; i < 256; ++i) {
        Gate gate;
        Require(gate.Enable(), "enable race fixture");
        std::atomic<unsigned> cancels{0};
        auto ticket = gate.Reserve([&] { ++cancels; }, [] { return false; });
        std::promise<void> release;
        auto barrier = release.get_future().share();
        bool committed = false;
        std::thread sender([&] { barrier.wait(); committed = ticket->Start(); });
        std::thread closer([&] { barrier.wait(); gate.Revoke(); });
        release.set_value();
        sender.join(); closer.join();
        Require(cancels.load() == (committed ? 1u : 0u),
                "every committed send has exactly one cancellation initiation");
        Require(gate.Drained() == !committed,
                "only uncommitted reservations settle without terminal");
        if (committed) ticket->Finish();
        Require(gate.Drained(), "race fixture retains terminal responsibility");
    }
}

/** @brief Cancellation can reenter state queries without holding the gate lock. */
void ReentrantCancellationAndFault() {
    Gate gate;
    Require(gate.Enable(), "enable reentry fixture");
    auto ticket = gate.Reserve([&] {
        gate.Close();
        Require(!gate.Drained(), "reentrant callback sees pending movement");
    }, [] { return false; });
    Require(ticket && ticket->Start(), "start reentry movement");
    gate.Revoke(); ticket->Finish();
    Require(gate.WaitForDrain(20ms), "reentry preserves drain");

    Require(gate.Enable(), "enable next revision");
    auto uncertain = gate.Reserve([] {}, [] { return true; });
    Require(uncertain && uncertain->Start(), "commit uncertain movement");
    gate.Revoke();
    Require(gate.HasFault() && !gate.WaitForDrain(20ms), "failure observable and sticky");
    uncertain->Finish();
    Require(gate.Drained() && !gate.Enable(), "terminal does not clear latched fault");
}

/** @brief Retired planning captures are released outside locks on the next reserve. */
void BoundedPlanningLifetime() {
    auto gate = std::make_shared<Gate>();
    Require(gate->Enable(), "enable planning lifetime fixture");
    std::weak_ptr<unsigned> previous;
    for (unsigned i = 0; i < 512; ++i) {
        auto capture = std::make_shared<unsigned>(i);
        auto ticket = gate->Reserve([capture] {}, [] { return true; }, Gate::Kind::kPlanning);
        Require(previous.expired(), "previous retired capture released on next reserve");
        Require(ticket && ticket->Start() && ticket->RetirePlanning(),
                "planning can retire without asserting a remote result");
        previous = capture;
        Require(gate->Drained() && !gate->HasFault(),
                "planning retirement/failure independent of physical drain");
    }
    auto unused_motion = gate->Reserve([] {}, [] { return false; });
    Require(unused_motion && previous.expired(), "final retired planning capture reclaimed");
    gate->Revoke();
    Require(gate->Drained(), "unstarted motion reservation settles locally");
}

/** @brief External denial settles only unsent work and cannot revive on resume. */
void ExternalPermitAndTerminal() {
    std::atomic<bool> allowed{true};
    Gate gate([&] { return allowed.load(); });
    Require(gate.HasExternalPermit() && gate.PermitValid() && gate.Enable(),
            "fresh invocation retains its external owner");
    unsigned cancels = 0;
    auto active = gate.Reserve([&] { ++cancels; }, [] { return false; });
    auto pending = gate.Reserve([] {}, [] { return false; });
    Require(active && active->Start() && pending, "own active and pending guarded sends");
    allowed = false;
    Require(!pending->Start() && active->Revoked() && !gate.PermitValid() &&
            !gate.Reserve([] {}, [] { return false; }), "external denial blocks send boundaries");
    Require(!gate.Drained() && cancels == 0, "query denial is not cancellation or terminal");
    gate.Suspend(); gate.Revoke();
    Require(cancels == 1 && !gate.Drained() && !gate.Suspended() && !active->Suspended(),
            "external denial supersedes retained pause without faking terminal");
    active->Finish(); allowed = true;
    Require(gate.Drained() && !gate.Enable() && !active->HasResumedRevision() &&
            !gate.PermitValid() && !gate.HasFault(), "terminal and restored query cannot revive denial");
    Gate next([&] { return allowed.load(); });
    Require(next.Enable(), "explicit new invocation gets independent permission state");
}

/** @brief Exceptions deny permission without escaping into transport or task code. */
void ExternalPermitException() {
    bool throws = true;
    Gate gate([&] {
        if (throws) throw std::runtime_error("unavailable parent lease");
        return true;
    });
    Require(!gate.Enable() && !gate.PermitValid() && !gate.HasFault(),
            "permission exception is sticky denial, not fabricated transport uncertainty");
    throws = false;
    Require(!gate.Enable(), "recovered throwing callback cannot revive this invocation");
    Gate ordinary;
    Require(!ordinary.HasExternalPermit() && ordinary.Enable(), "ordinary admission stays compatible");
}

/** @brief Concurrent owner loss preserves cancellation ownership and sticky denial. */
void ConcurrentExternalPermit() {
    for (unsigned i = 0; i < 128; ++i) {
        std::atomic<bool> allowed{true};
        std::atomic<unsigned> cancels{0};
        Gate gate([&] { return allowed.load(); });
        Require(gate.Enable(), "enable guarded race");
        auto ticket = gate.Reserve([&] { ++cancels; }, [] { return false; });
        Require(static_cast<bool>(ticket), "reserve guarded race");
        std::promise<void> release;
        auto barrier = release.get_future().share();
        bool sent = false;
        std::thread sender([&] { barrier.wait(); sent = ticket->Start(); });
        std::thread owner([&] { barrier.wait(); allowed = false; gate.PermitValid(); gate.Revoke(); });
        release.set_value(); sender.join(); owner.join();
        Require(cancels.load() == (sent ? 1u : 0u) && gate.Drained() == !sent,
                "guarded racing commitment keeps exactly one cancel owner");
        if (sent) ticket->Finish();
        allowed = true;
        Require(!gate.Enable() && !ticket->Start(), "raced denied invocation cannot revive");
    }
}
}  // namespace

/** @brief Emit separate results for native and cooperative ownership contracts. */
int main() {
    try {
        TerminalAndResume(); std::cout << "PASS terminal ownership and resumed revision\n";
        SuspensionAndCancellation(); std::cout << "PASS suspension, cancellation and fault precedence\n";
        ConcurrentPauseAndCommit(); std::cout << "PASS 256 concurrent pause/commit races\n";
        ConcurrentCommitAndRevoke(); std::cout << "PASS 256 concurrent commitment/revocation races\n";
        ReentrantCancellationAndFault(); std::cout << "PASS cancellation reentry and sticky fault\n";
        BoundedPlanningLifetime(); std::cout << "PASS 512 planning retirements release captures\n";
        ExternalPermitAndTerminal(); std::cout << "PASS external permission and real terminal drain\n";
        ExternalPermitException(); std::cout << "PASS sticky external query exception\n";
        ConcurrentExternalPermit(); std::cout << "PASS 128 external permission/commit races\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
    return 0;
}
