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

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <unordered_map>

using autolink::examples::proto::ExampleAction;

namespace {
// Helper: Create result message
auto CreateResult(bool success, int32_t count, const std::string& message,
                  int64_t duration_ms) {
    auto result = std::make_shared<ExampleAction::Result>();
    result->set_success(success);
    result->set_final_count(count);
    result->set_message(message);
    result->set_execution_time_ms(duration_ms);
    return result;
}

// Helper: Calculate duration in milliseconds
int64_t DurationMs(const std::chrono::steady_clock::time_point& start) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - start)
        .count();
}

// Helper: Validate goal
bool IsValidGoal(const ExampleAction::Goal* goal) {
    return goal && goal->target_number() > 0 && goal->target_number() <= 100;
}
}  // namespace

// Execution state for each goal
struct GoalExecutionState {
    std::atomic<bool> running{false};
    std::atomic<bool> canceled{false};
    std::shared_ptr<autolink::action::ServerGoalHandle<ExampleAction>>
        goal_handle;
    std::thread execution_thread;
};

int main(int argc, char* argv[]) {
    autolink::Init(argv[0]);

    const char* node_name_env = std::getenv("AUTOLINK_NODE_NAME");
    std::string node_name =
        node_name_env ? node_name_env : "action_server_test";
    auto node = autolink::CreateNode(node_name);

    AINFO << "Creating action server for ExampleAction...";

    // Active goal executions
    std::unordered_map<autolink::action::GoalUUID,
                       std::shared_ptr<GoalExecutionState>>
        active_goals;
    std::mutex goals_mutex;

    // ========================================================================
    // Callback 1: handle_goal - Decide whether to accept or reject a goal
    // ========================================================================
    auto handle_goal = [](const autolink::action::GoalUUID& goal_id,
                          std::shared_ptr<const ExampleAction::Goal> goal)
        -> autolink::action::GoalResponse {
        if (!goal) {
            AERROR << "Received null goal - REJECTING";
            return autolink::action::GoalResponse::REJECT;
        }

        if (!IsValidGoal(goal.get())) {
            AERROR << "Invalid goal (target_number=" << goal->target_number()
                   << " must be 1-100) - REJECTING";
            return autolink::action::GoalResponse::REJECT;
        }

        AINFO << "Goal ACCEPTED: " << goal->task_name()
              << " (target: " << goal->target_number() << ")";
        return autolink::action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    // ========================================================================
    // Callback 2: handle_cancel - Decide whether to accept a cancel request
    // ========================================================================
    auto handle_cancel =
        [&active_goals, &goals_mutex](
            std::shared_ptr<autolink::action::ServerGoalHandle<ExampleAction>>
                goal_handle) -> autolink::action::CancelResponse {
        auto goal_id = goal_handle->GetGoalId();
        std::lock_guard<std::mutex> lock(goals_mutex);

        if (active_goals.count(goal_id)) {
            active_goals[goal_id]->canceled = true;
            AINFO << "Cancel request ACCEPTED";
            return autolink::action::CancelResponse::ACCEPT;
        }

        AWARN << "Cancel request for unknown goal - REJECTING";
        return autolink::action::CancelResponse::REJECT;
    };

    // ========================================================================
    // Callback 3: handle_accepted - Called when goal is accepted, start
    // execution
    // ========================================================================
    auto handle_accepted =
        [&active_goals, &goals_mutex](
            std::shared_ptr<autolink::action::ServerGoalHandle<ExampleAction>>
                goal_handle) {
            auto goal_id = goal_handle->GetGoalId();
            auto goal = goal_handle->GetGoal();
            if (!goal) {
                AERROR << "Received null goal in accepted callback - aborting";
                auto result = CreateResult(
                    false, 0, "Invalid goal in accepted callback", 0);
                goal_handle->Abort(result);
                return;
            }

            AINFO << "Starting execution: " << goal->task_name()
                  << " (target: " << goal->target_number() << ")";

            // Transition goal to EXECUTING state (must be done in this
            // callback)
            try {
                goal_handle->Execute();
            } catch (const std::exception& e) {
                AERROR << "Failed to execute goal: " << e.what();
                auto result = CreateResult(
                    false, 0,
                    "Failed to start execution: " + std::string(e.what()), 0);
                goal_handle->Abort(result);
                return;
            }

            // Create execution state and start thread
            auto exec_state = std::make_shared<GoalExecutionState>();
            exec_state->goal_handle = goal_handle;
            exec_state->running = true;

            {
                std::lock_guard<std::mutex> lock(goals_mutex);
                active_goals[goal_id] = exec_state;
            }

            // Execute goal in separate thread
            exec_state->execution_thread = std::thread([exec_state, goal,
                                                        goal_id, &active_goals,
                                                        &goals_mutex]() {
                int32_t target = goal->target_number();
                std::string task_name = goal->task_name();
                auto start_time = std::chrono::steady_clock::now();
                int32_t current = 0;

                // Execution loop - iterate until we reach the target
                while (current < target && autolink::OK()) {
                    // Check cancellation
                    if (exec_state->canceled ||
                        exec_state->goal_handle->IsCanceling()) {
                        AINFO << "Goal canceled at progress: " << current << "/"
                              << target;
                        auto result = CreateResult(false, current,
                                                   "Action canceled by client",
                                                   DurationMs(start_time));
                        try {
                            exec_state->goal_handle->Canceled(result);
                        } catch (const std::exception& e) {
                            AERROR << "Exception in Canceled: " << e.what();
                        }
                        exec_state->running = false;
                        return;
                    }

                    current++;

                    // Publish feedback every step or every 10% progress
                    if (target <= 10 ||
                        current % std::max(1, target / 10) == 0 ||
                        current == target) {
                        auto feedback =
                            std::make_shared<ExampleAction::Feedback>();
                        feedback->set_current_progress(current);
                        feedback->set_percentage(static_cast<double>(current) /
                                                 target);
                        feedback->set_status_message(
                            "Processing: " + std::to_string(current) + "/" +
                            std::to_string(target) + " (" +
                            std::to_string(static_cast<int>(
                                feedback->percentage() * 100)) +
                            "%)");

                        try {
                            exec_state->goal_handle->PublishFeedback(feedback);
                        } catch (const std::exception& e) {
                            AERROR << "Failed to publish feedback: "
                                   << e.what();
                        }
                    }

                    // Small delay between iterations
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }

                // Complete goal
                exec_state->running = false;
                auto duration = DurationMs(start_time);

                if (current >= target) {
                    auto result = CreateResult(true, current,
                                               "Action completed: " + task_name,
                                               duration);
                    try {
                        exec_state->goal_handle->Succeed(result);
                        AINFO << "Goal succeeded: " << task_name << " ("
                              << current << "/" << target << ", " << duration
                              << " ms)";
                    } catch (const std::exception& e) {
                        AERROR << "Exception in Succeed: " << e.what();
                    }
                } else if (!autolink::OK()) {
                    auto result = CreateResult(
                        false, current, "Action aborted: system shutdown",
                        duration);
                    try {
                        exec_state->goal_handle->Abort(result);
                        AINFO << "Goal aborted: system shutdown";
                    } catch (const std::exception& e) {
                        AERROR << "Exception in Abort: " << e.what();
                    }
                } else {
                    auto result = CreateResult(
                        false, current, "Action did not complete properly",
                        duration);
                    try {
                        exec_state->goal_handle->Abort(result);
                        AINFO << "Goal aborted: did not complete";
                    } catch (const std::exception& e) {
                        AERROR << "Exception in Abort: " << e.what();
                    }
                }
            });
        };

    // Create action server with the three callbacks
    // Convert unique_ptr to shared_ptr
    std::shared_ptr<autolink::Node> node_shared(node.release());
    auto server = autolink::action::CreateServer<ExampleAction>(
        node_shared, "example_action", handle_goal, handle_cancel,
        handle_accepted);

    if (!server) {
        AERROR << "Failed to create action server!";
        return 1;
    }

    AINFO << "Action server ready. Waiting for goals (target_number: 1-100)...";

    autolink::WaitForShutdown();

    // Cleanup: cancel all active goals
    {
        std::lock_guard<std::mutex> lock(goals_mutex);
        for (auto& [goal_id, exec_state] : active_goals) {
            exec_state->canceled = true;
        }
    }

    // Wait for threads to complete (with timeout)
    auto start_cleanup = std::chrono::steady_clock::now();
    const auto cleanup_timeout = std::chrono::seconds(5);

    while (true) {
        {
            std::lock_guard<std::mutex> lock(goals_mutex);
            if (active_goals.empty()) {
                break;
            }

            if (std::chrono::steady_clock::now() - start_cleanup >
                cleanup_timeout) {
                AWARN << "Cleanup timeout: some goals may still be executing";
                break;
            }

            // Remove completed threads
            for (auto it = active_goals.begin(); it != active_goals.end();) {
                if (!it->second->running) {
                    if (it->second->execution_thread.joinable()) {
                        it->second->execution_thread.join();
                    }
                    it = active_goals.erase(it);
                } else {
                    ++it;
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Final cleanup: detach any remaining threads
    {
        std::lock_guard<std::mutex> lock(goals_mutex);
        for (auto& [goal_id, exec_state] : active_goals) {
            if (exec_state->execution_thread.joinable()) {
                exec_state->execution_thread.detach();
            }
        }
        active_goals.clear();
    }

    return 0;
}
