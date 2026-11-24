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
 * @file result_response_wait_test.cpp
 * @brief Verify the production result waiter with real asynchronous promises.
 */

#include <atomic>
#include <chrono>
#include <future>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "autolink/action/result_response_wait.hpp"

namespace {

using autolink::action::internal::WaitForResultResponse;
using namespace std::chrono_literals;

/**
 * @brief Fail with a named lifecycle contract instead of hanging the test.
 */
void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

/**
 * @brief A ready real response remains available after local monitor release.
 */
void ReadyEvidenceWins() {
    std::promise<int> promise;
    const auto response = promise.get_future().share();
    promise.set_value(17);
    Require(WaitForResultResponse(response, [] { return false; }),
            "local release concealed an already ready response");
    Require(response.get() == 17, "waiter changed the transport response");
}

/**
 * @brief Local release does not fabricate a result or consume a pending
          * future.
 */
void PendingRelease() {
    std::promise<int> promise;
    const auto response = promise.get_future().share();
    Require(!WaitForResultResponse(response, [] { return false; }),
            "a released pending response was reported ready");
    Require(response.wait_for(0ms) == std::future_status::timeout,
            "local release fabricated a transport result");
    promise.set_value(29);
    Require(response.get() == 29, "released future no longer receives its real response");
}

/**
 * @brief Release wakes a worker whose native response never arrives.
 */
void ReleaseWhileWaiting() {
    std::promise<int> promise;
    const auto response = promise.get_future().share();
    std::promise<void> entered;
    auto started = entered.get_future();
    std::atomic<bool> monitoring{true};
    std::atomic<bool> announced{false};
    auto worker = std::async(std::launch::async, [&] {
        return WaitForResultResponse(response, [&] {
            if (!announced.exchange(true)) entered.set_value();
            return monitoring.load();
        }, 2ms);
    });
    const bool did_start = started.wait_for(1s) == std::future_status::ready;
    monitoring.store(false);
    const bool stopped = worker.wait_for(1s) == std::future_status::ready;
    // An implementation regression must fail rather than strand std::async's
    // joining destructor; releasing the real promise permits cleanup on failure.
    if (!stopped) promise.set_value(41);
    const bool ready = worker.get();
    Require(did_start && stopped, "local release did not stop the pending result waiter");
    Require(!ready, "release fabricated result readiness");
    Require(response.wait_for(0ms) == std::future_status::timeout,
            "test unexpectedly needed a real response to stop");
}

/**
 * @brief Retained actions do not acquire an arbitrary overall execution
          * timeout.
 */
void RetainedLongAction() {
    std::promise<int> promise;
    const auto response = promise.get_future().share();
    std::promise<void> monitoring_established;
    auto established = monitoring_established.get_future();
    std::atomic<unsigned> checks{0};
    auto worker = std::async(std::launch::async, [&] {
        return WaitForResultResponse(response, [&] {
            if (checks.fetch_add(1) == 2) monitoring_established.set_value();
            return true;
        }, 2ms);
    });
    const bool polled = established.wait_for(1s) == std::future_status::ready;
    const bool remained_pending = worker.wait_for(0ms) == std::future_status::timeout;
    promise.set_value(53);
    const bool ready = worker.get();
    Require(polled && remained_pending && ready,
            "a retained action was released before its real response");
    Require(response.get() == 53, "real response was consumed by the waiter");
}

/**
 * @brief A transport exception stays an exception, including after local
          * release.
 */
void PreserveResponseException() {
    std::promise<int> promise;
    const auto response = promise.get_future().share();
    promise.set_exception(std::make_exception_ptr(std::runtime_error("native transport failure")));
    Require(WaitForResultResponse(response, [] { return false; }),
            "ready transport exception was hidden by local release");
    bool failed = false;
    try { (void)response.get(); }
    catch (const std::runtime_error& error) {
        failed = std::string(error.what()) == "native transport failure";
    }
    Require(failed, "transport exception was replaced with a fake response");
}

/**
 * @brief Invalid or deferred work cannot make get() execute a blocking task.
 */
void RejectNonAsyncInput() {
    bool invalid = false;
    try { WaitForResultResponse(std::shared_future<int>{}, [] { return true; }); }
    catch (const std::invalid_argument&) { invalid = true; }
    Require(invalid, "invalid response future was accepted");

    std::atomic<unsigned> executed{0};
    const auto deferred = std::async(std::launch::deferred, [&] {
        ++executed;
        return 67;
    }).share();
    bool rejected = false;
    try { WaitForResultResponse(deferred, [] { return true; }); }
    catch (const std::runtime_error&) { rejected = true; }
    Require(rejected && executed.load() == 0,
            "result waiter executed deferred transport work");

    std::promise<int> promise;
    const auto response = promise.get_future().share();
    for (auto cadence : {0ms, -1ms}) {
        bool bad_cadence = false;
        try { WaitForResultResponse(response, [] { return true; }, cadence); }
        catch (const std::invalid_argument&) { bad_cadence = true; }
        Require(bad_cadence, "nonpositive result polling cadence was accepted");
    }
}

}  // namespace

/**
 * @brief Run named contracts against the exact production result-wait helper.
 */
int main() {
    const std::vector<std::pair<const char*, void (*)()>> tests = {
        {"ready_terminal_evidence", ReadyEvidenceWins},
        {"pending_local_release", PendingRelease},
        {"release_while_waiting", ReleaseWhileWaiting},
        {"retained_long_action", RetainedLongAction},
        {"preserve_transport_exception", PreserveResponseException},
        {"reject_invalid_and_deferred", RejectNonAsyncInput},
    };
    try {
        for (const auto& test : tests) {
            test.second();
            std::cout << "PASS " << test.first << '\n';
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
