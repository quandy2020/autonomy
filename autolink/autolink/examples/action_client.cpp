/**
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autolink/action/action.hpp"
#include "autolink/autolink.hpp"
#include "autolink/common/log.hpp"
#include "autolink/common/state.hpp"
#include "autolink/examples/proto/examples.pb.h"

#include <chrono>
#include <iomanip>
#include <thread>

using autolink::action::ResultCode;
using autolink::examples::proto::ExampleAction;

int main(int argc, char* argv[]) {
    autolink::Init(argv[0]);

    const char* node_name_env = std::getenv("AUTOLINK_NODE_NAME");
    std::string node_name =
        node_name_env ? node_name_env : "action_client_test";
    auto node = autolink::CreateNode(node_name);

    AINFO << "Creating action client for ExampleAction...";

    // Convert unique_ptr to shared_ptr
    std::shared_ptr<autolink::Node> node_shared(node.release());
    auto action_client = autolink::action::CreateClient<ExampleAction>(
        node_shared, "example_action");

    if (!action_client) {
        AERROR << "Failed to create action client!";
        return 1;
    }

    // Wait for server
    int wait_count = 0;
    int max_wait = 200;
    while (!action_client->ActionServerIsReady() && wait_count < max_wait) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        wait_count++;
    }

    if (!action_client->ActionServerIsReady()) {
        AERROR << "Action server not ready after " << (max_wait * 100) << "ms!";
        return 1;
    }

    // Additional wait for service discovery to fully propagate
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Create goal
    ExampleAction::Goal goal;
    int32_t target = (argc > 1) ? std::atoi(argv[1]) : 10;
    if (target <= 0 || target > 100) {
        AWARN << "Invalid target_number: " << target << ". Using default: 10";
        target = 10;
    }
    goal.set_target_number(target);
    goal.set_task_name("Count to " + std::to_string(target));

    AINFO << "Sending goal: " << goal.task_name() << " (target: " << target
          << ")";

    // Configure callbacks
    autolink::action::Client<ExampleAction>::SendGoalOptions options;

    // Callback 1: Goal response callback (called when server accepts/rejects
    // goal)
    options.goal_response_callback = [](auto goal_handle) {
        if (goal_handle) {
            AINFO << "Goal ACCEPTED";
        } else {
            AINFO << "Goal REJECTED";
        }
    };

    // Callback 2: Feedback callback (called when server publishes feedback)
    options.feedback_callback = [](auto goal_handle, auto feedback) {
        AINFO << "Progress: " << feedback->current_progress() << " ("
              << std::fixed << std::setprecision(1)
              << (feedback->percentage() * 100.0) << "%)";
    };

    // Callback 3: Result callback (called when result is available)
    options.result_callback = [](const auto& wrapped_result) {
        std::string status_str;
        switch (wrapped_result.code) {
            case ResultCode::SUCCEEDED:
                status_str = "SUCCEEDED";
                break;
            case ResultCode::CANCELED:
                status_str = "CANCELED";
                break;
            case ResultCode::ABORTED:
                status_str = "ABORTED";
                break;
            default:
                status_str = "UNKNOWN";
                break;
        }
        AINFO << "Result: " << status_str
              << ", count: " << wrapped_result.result->final_count()
              << ", time: " << wrapped_result.result->execution_time_ms()
              << " ms";
    };

    // Send goal
    auto goal_future = action_client->AsyncSendGoal(goal, options);
    auto goal_handle = goal_future.get();

    if (!goal_handle) {
        AERROR << "Failed to send goal or goal was rejected!";
        return 1;
    }

    // Optional: Cancel goal after specified seconds
    bool should_cancel = false;
    int cancel_after_seconds = 0;
    if (argc > 2) {
        cancel_after_seconds = std::atoi(argv[2]);
        if (cancel_after_seconds > 0) {
            should_cancel = true;
        }
    }

    // Monitor thread for cancellation
    std::thread monitor_thread(
        [action_client, goal_handle, should_cancel, cancel_after_seconds]() {
            if (should_cancel) {
                std::this_thread::sleep_for(
                    std::chrono::seconds(cancel_after_seconds));
                auto status = static_cast<autolink::action::GoalStatus>(
                    goal_handle->GetStatus());
                if (status != autolink::action::GoalStatus::SUCCEEDED &&
                    status != autolink::action::GoalStatus::CANCELED &&
                    status != autolink::action::GoalStatus::ABORTED) {
                    try {
                        auto cancel_future =
                            action_client->AsyncCancelGoal(goal_handle);
                        cancel_future.get();
                    } catch (const std::exception& e) {
                        AERROR << "Error canceling goal: " << e.what();
                    }
                }
            }
        });

    // Wait for result
    ResultCode final_code = ResultCode::UNKNOWN;
    try {
        auto result_future = action_client->AsyncGetResult(goal_handle);
        auto status = result_future.wait_for(std::chrono::seconds(30));

        if (status == std::future_status::timeout) {
            AERROR << "Timeout waiting for result!";
            monitor_thread.detach();
            return 1;
        }

        auto wrapped_result = result_future.get();
        final_code = wrapped_result.code;

        std::string status_str;
        switch (final_code) {
            case ResultCode::SUCCEEDED:
                status_str = "SUCCEEDED";
                break;
            case ResultCode::CANCELED:
                status_str = "CANCELED";
                break;
            case ResultCode::ABORTED:
                status_str = "ABORTED";
                break;
            default:
                status_str = "UNKNOWN";
                break;
        }
        AINFO << "Final result: " << status_str
              << ", count: " << wrapped_result.result->final_count()
              << ", time: " << wrapped_result.result->execution_time_ms()
              << " ms";
    } catch (const std::exception& e) {
        AERROR << "Error getting result: " << e.what();
    }

    if (monitor_thread.joinable()) {
        monitor_thread.join();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    return (final_code == ResultCode::SUCCEEDED) ? 0 : 1;
}
