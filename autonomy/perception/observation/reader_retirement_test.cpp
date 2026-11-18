/* Copyright 2026 The Openbot Authors. */
/** @file reader_retirement_test.cpp
 * @brief Exercise the production retirement worker and ownership protocol.
 * @details Native Node/Reader behavior has a separate integration test. These
 * tests do not replace Reader with a mock or claim native scheduler validation.
 */
#include <atomic>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

#include "autonomy/perception/observation/reader_retirement.hpp"

namespace {
using namespace std::chrono_literals;
namespace p = autonomy::perception;
/** @brief Fail a scenario with its actual unmet invariant. */
void Require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
/** @brief Deterministic worker entry and release without guessed delays. */
struct Barrier {
    std::promise<void> entered;  ///< Worker entry publication.
    std::promise<void> release;  ///< External owner release.
    std::shared_future<void> released = release.get_future().share();  ///< Worker wait handle.
    std::future<void> entry = entered.get_future();  ///< External owner wait handle.
    void Enter() { entered.set_value(); released.wait(); }
    void Await() { Require(entry.wait_for(3s) == std::future_status::ready, "worker reached cleanup"); }
    void Release() noexcept { try { release.set_value(); } catch (...) {} }
};

/** @brief External release survives early failure even while cleanup pins Barrier. */
struct ReleaseOnExit {
    std::shared_ptr<Barrier> barrier;  ///< Worker barrier to release on every exit path.
    ~ReleaseOnExit() { barrier->Release(); }
};

/** @brief Request never waits for an outstanding callback/resource lock. */
void DeferredCleanupAndDeadline() {
    auto barrier = std::make_shared<Barrier>();
    ReleaseOnExit release{barrier};
    const auto caller = std::this_thread::get_id();
    auto other_thread = std::make_shared<std::atomic<bool>>(false);
    p::ReaderRetirement retirement([barrier, caller, other_thread] {
        *other_thread = std::this_thread::get_id() != caller;
        barrier->Enter(); return true;
    });
    Require(!retirement.Snapshot().requested, "construction does not retire subscriptions");
    retirement.Request(); barrier->Await();
    Require(*other_thread && !retirement.WaitFor(0ms), "request defers teardown and timeout stays outstanding");
    Require(!retirement.Snapshot().finished, "blocked cleanup cannot publish success");
    barrier->Release();
    Require(retirement.WaitFor(3s) && retirement.Snapshot().succeeded, "external owner observes real completion");
}

/** @brief Source destruction cannot release captured resources on its callback. */
void CaptureSurvivesRequesterDestruction() {
    auto barrier = std::make_shared<Barrier>();
    ReleaseOnExit release{barrier};
    auto destroyed = std::make_shared<std::atomic<int>>(0);
    struct Resource {
        explicit Resource(std::shared_ptr<std::atomic<int>> value) : count(std::move(value)) {}
        std::shared_ptr<std::atomic<int>> count;  ///< Destructor evidence outlives failed assertions.
        ~Resource() { ++*count; }
    };
    auto resource = std::make_shared<Resource>(destroyed);
    std::weak_ptr<Resource> weak = resource;
    auto retirement = std::make_unique<p::ReaderRetirement>([resource, barrier] {
        barrier->Enter(); return true;
    });
    const auto completion = retirement->Completion();
    resource.reset(); retirement.reset(); barrier->Await();
    Require(!weak.expired() && *destroyed == 0, "worker pins resources after requester destruction");
    barrier->Release();
    Require(completion.wait_for(3s) == std::future_status::ready && completion.get(), "stable completion survives owner");
    Require(weak.expired() && *destroyed == 1, "completion follows capture destruction");
}

/** @brief Exception unwinding requests retirement and releases an outstanding callback. */
void ExceptionalRequesterExit() {
    auto barrier = std::make_shared<Barrier>();
    auto pin = std::make_shared<int>(7);
    std::weak_ptr<int> weak = pin;
    std::shared_future<bool> completion;
    bool unwound = false;
    try {
        ReleaseOnExit release{barrier};
        p::ReaderRetirement retirement([barrier, pin] { barrier->Enter(); return *pin == 7; });
        completion = retirement.Completion();
        pin.reset();
        retirement.Request(); barrier->Await();
        throw std::runtime_error("controlled requester exit");
    } catch (const std::runtime_error& error) {
        unwound = std::string(error.what()) == "controlled requester exit";
    }
    Require(unwound && completion.valid() && completion.wait_for(3s) == std::future_status::ready &&
        completion.get() && weak.expired(), "exceptional owner exit drains and releases pinned resources");
}

/** @brief Concurrent close requests execute the actual cleanup only once. */
void ConcurrentRequests() {
    auto calls = std::make_shared<std::atomic<int>>(0);
    p::ReaderRetirement retirement([calls] { ++*calls; return true; });
    std::vector<std::thread> requesters;
    try {
        for (int i = 0; i < 16; ++i) requesters.emplace_back([&] { retirement.Request(); });
    } catch (...) {
        for (auto& thread : requesters) thread.join();
        throw;
    }
    for (auto& thread : requesters) thread.join();
    Require(retirement.WaitFor(3s) && *calls == 1, "concurrent requests retire exactly once");
}

/** @brief Exceptions and explicit failure stay failures after draining. */
void CleanupFailure() {
    for (bool throwing : {false, true}) {
        p::ReaderRetirement retirement([throwing]() -> bool {
            if (throwing) throw 7;
            return false;
        });
        retirement.Request();
        const auto completion = retirement.Completion();
        Require(completion.wait_for(3s) == std::future_status::ready && !completion.get(), "failure result retained");
        Require(retirement.Snapshot().finished && !retirement.WaitFor(0ms), "finished failure is not successful drain");
    }
}

/** @brief Cleanup reentry cannot wait on its own completion or rerun cleanup. */
void WorkerReentry() {
    struct Context {
        std::weak_ptr<p::ReaderRetirement> retirement;  ///< No cycle between worker and requester.
        std::atomic<bool> rejected{false};  ///< Worker reentry result survives failed assertions.
    };
    auto context = std::make_shared<Context>();
    auto retirement = std::make_shared<p::ReaderRetirement>([context] {
        const auto requester = context->retirement.lock();
        if (!requester) return false;
        requester->Request(); context->rejected = !requester->WaitFor(3s); return true;
    });
    context->retirement = retirement;
    retirement->Request();
    Require(retirement->WaitFor(3s) && context->rejected, "worker wait-on-self rejected without deadlock");
    Require(!retirement->WaitFor(-1ms), "negative deadline rejected");
}
}  // namespace

/** @brief Run worker protocol scenarios separately from native Reader integration. */
int main() {
    const std::vector<std::pair<const char*, void(*)()>> cases = {
        {"deferred cleanup deadline", DeferredCleanupAndDeadline},
        {"requester destruction pin", CaptureSurvivesRequesterDestruction},
        {"exceptional requester exit", ExceptionalRequesterExit},
        {"concurrent request once", ConcurrentRequests},
        {"cleanup failure", CleanupFailure}, {"worker reentry", WorkerReentry}};
    int failures = 0;
    for (const auto& test : cases) {
        try { test.second(); std::cout << "PASS " << test.first << std::endl; }
        catch (const std::exception& error) { ++failures; std::cerr << "FAIL " << test.first << ": " << error.what() << std::endl; }
    }
    return failures ? 1 : 0;
}
