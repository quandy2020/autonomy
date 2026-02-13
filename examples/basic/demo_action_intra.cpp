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
 *
 * Demo: Intra-process Action (server + client in same process).
 * Runs both ExampleAction server and client in one process, demonstrating
 * intra-process action communication without network.
 * Usage: demo_action_intra [target_number]
 *   target_number: 1-100 (default 5)
 */

#include "autolink/action/action.hpp"
#include "autolink/autolink.hpp"
#include "autolink/common/log.hpp"
#include "autolink/common/state.hpp"
#include "proto/examples.pb.h"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <mutex>
#include <thread>
#include <unordered_map>

using autolink::action::ResultCode;
using autolink::examples::proto::ExampleAction;

namespace {
auto CreateResult(bool success, int32_t count, const std::string& message,
                  int64_t duration_ms) {
  auto result = std::make_shared<ExampleAction::Result>();
  result->set_success(success);
  result->set_final_count(count);
  result->set_message(message);
  result->set_execution_time_ms(duration_ms);
  return result;
}

int64_t DurationMs(const std::chrono::steady_clock::time_point& start) {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now() - start)
      .count();
}

bool IsValidGoal(const ExampleAction::Goal* goal) {
  return goal && goal->target_number() > 0 && goal->target_number() <= 100;
}

struct GoalExecutionState {
  std::atomic<bool> running{false};
  std::atomic<bool> canceled{false};
  std::shared_ptr<autolink::action::ServerGoalHandle<ExampleAction>> goal_handle;
  std::thread execution_thread;
};
}  // namespace

int main(int argc, char* argv[]) {
  autolink::Init(argv[0]);

  const char* node_name_env = std::getenv("AUTOLINK_NODE_NAME");
  std::string node_name =
      node_name_env ? node_name_env : "demo_action_intra";
  auto node = autolink::CreateNode(node_name);

  AINFO << "Demo: Intra-process Action (server + client in same process)";

  // ---- Action Server ----
  std::unordered_map<autolink::action::GoalUUID,
                     std::shared_ptr<GoalExecutionState>>
      active_goals;
  std::mutex goals_mutex;

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
    AINFO << "[Server] Goal ACCEPTED: " << goal->task_name()
          << " (target: " << goal->target_number() << ")";
    return autolink::action::GoalResponse::ACCEPT_AND_EXECUTE;
  };

  auto handle_cancel =
      [&active_goals, &goals_mutex](
          std::shared_ptr<autolink::action::ServerGoalHandle<ExampleAction>>
              goal_handle) -> autolink::action::CancelResponse {
    auto goal_id = goal_handle->GetGoalId();
    std::lock_guard<std::mutex> lock(goals_mutex);
    if (active_goals.count(goal_id)) {
      active_goals[goal_id]->canceled = true;
      AINFO << "[Server] Cancel request ACCEPTED";
      return autolink::action::CancelResponse::ACCEPT;
    }
    AWARN << "[Server] Cancel request for unknown goal - REJECTING";
    return autolink::action::CancelResponse::REJECT;
  };

  auto handle_accepted =
      [&active_goals, &goals_mutex](
          std::shared_ptr<autolink::action::ServerGoalHandle<ExampleAction>>
              goal_handle) {
        auto goal_id = goal_handle->GetGoalId();
        auto goal = goal_handle->GetGoal();
        if (!goal) {
          AERROR << "Null goal in accepted callback";
          goal_handle->Abort(CreateResult(false, 0, "Invalid goal", 0));
          return;
        }
        AINFO << "[Server] Starting execution: " << goal->task_name()
              << " (target: " << goal->target_number() << ")";
        try {
          goal_handle->Execute();
        } catch (const std::exception& e) {
          AERROR << "Execute failed: " << e.what();
          goal_handle->Abort(
              CreateResult(false, 0, std::string("Execute: ") + e.what(), 0));
          return;
        }

        auto exec_state = std::make_shared<GoalExecutionState>();
        exec_state->goal_handle = goal_handle;
        exec_state->running = true;
        {
          std::lock_guard<std::mutex> lock(goals_mutex);
          active_goals[goal_id] = exec_state;
        }

        exec_state->execution_thread = std::thread(
            [exec_state, goal, goal_id, &active_goals, &goals_mutex]() {
              int32_t target = goal->target_number();
              std::string task_name = goal->task_name();
              auto start_time = std::chrono::steady_clock::now();
              int32_t current = 0;

              while (current < target && autolink::OK()) {
                if (exec_state->canceled ||
                    exec_state->goal_handle->IsCanceling()) {
                  AINFO << "[Server] Goal canceled at " << current << "/"
                        << target;
                  exec_state->goal_handle->Canceled(
                      CreateResult(false, current, "Canceled by client",
                                   DurationMs(start_time)));
                  exec_state->running = false;
                  return;
                }
                current++;
                if (target <= 10 || current % std::max(1, target / 10) == 0 ||
                    current == target) {
                  auto feedback = std::make_shared<ExampleAction::Feedback>();
                  feedback->set_current_progress(current);
                  feedback->set_percentage(
                      static_cast<double>(current) / target);
                  feedback->set_status_message(
                      "Processing: " + std::to_string(current) + "/" +
                      std::to_string(target));
                  try {
                    exec_state->goal_handle->PublishFeedback(feedback);
                  } catch (const std::exception& e) {
                    AERROR << "Feedback: " << e.what();
                  }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
              }

              exec_state->running = false;
              auto duration = DurationMs(start_time);
              if (current >= target) {
                exec_state->goal_handle->Succeed(
                    CreateResult(true, current,
                                 "Action completed: " + task_name, duration));
                AINFO << "[Server] Goal succeeded: " << task_name << " ("
                      << current << "/" << target << ", " << duration << " ms)";
              } else if (!autolink::OK()) {
                exec_state->goal_handle->Abort(CreateResult(
                    false, current, "System shutdown", duration));
              } else {
                exec_state->goal_handle->Abort(
                    CreateResult(false, current, "Incomplete", duration));
              }
            });
      };

  auto server = autolink::action::CreateServer<ExampleAction>(
      node, "example_action", handle_goal, handle_cancel, handle_accepted);
  if (!server) {
    AERROR << "Failed to create action server!";
    return 1;
  }
  AINFO << "[Server] Ready on example_action";

  // ---- Action Client ----
  auto action_client =
      autolink::action::CreateClient<ExampleAction>(node, "example_action");
  if (!action_client) {
    AERROR << "Failed to create action client!";
    return 1;
  }

  // Intra-process: server is in same process, brief wait for registration
  int wait_count = 0;
  while (!action_client->ActionServerIsReady() && wait_count < 100) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    wait_count++;
  }
  if (!action_client->ActionServerIsReady()) {
    AERROR << "Action server not ready!";
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Send goal
  ExampleAction::Goal goal;
  int32_t target = (argc > 1) ? std::atoi(argv[1]) : 5;
  if (target <= 0 || target > 100) {
    AWARN << "Invalid target_number " << target << ", using 5";
    target = 5;
  }
  goal.set_target_number(target);
  goal.set_task_name("Intra-process count to " + std::to_string(target));

  AINFO << "[Client] Sending goal: " << goal.task_name();

  autolink::action::Client<ExampleAction>::SendGoalOptions options;
  options.goal_response_callback = [](auto goal_handle) {
    AINFO << "[Client] " << (goal_handle ? "Goal ACCEPTED" : "Goal REJECTED");
  };
  options.feedback_callback = [](auto, auto feedback) {
    AINFO << "[Client] Progress: " << feedback->current_progress() << " ("
          << std::fixed << std::setprecision(1)
          << (feedback->percentage() * 100.0) << "%)";
  };
  options.result_callback = [](const auto& wrapped_result) {
    const char* status = "UNKNOWN";
    switch (wrapped_result.code) {
      case ResultCode::SUCCEEDED:
        status = "SUCCEEDED";
        break;
      case ResultCode::CANCELED:
        status = "CANCELED";
        break;
      case ResultCode::ABORTED:
        status = "ABORTED";
        break;
      default:
        break;
    }
    AINFO << "[Client] Result: " << status
          << " count=" << wrapped_result.result->final_count()
          << " time=" << wrapped_result.result->execution_time_ms() << " ms";
  };

  auto goal_future = action_client->AsyncSendGoal(goal, options);
  auto goal_handle = goal_future.get();
  if (!goal_handle) {
    AERROR << "Failed to send goal or goal rejected!";
    return 1;
  }

  ResultCode final_code = ResultCode::UNKNOWN;
  try {
    auto result_future = action_client->AsyncGetResult(goal_handle);
    int wait_count = 0;
    std::future_status status;
    while (wait_count < 300 && autolink::OK()) {
      status = result_future.wait_for(std::chrono::milliseconds(100));
      if (status != std::future_status::timeout) break;
      wait_count++;
    }
    if (status == std::future_status::timeout) {
      AERROR << "Timeout waiting for result";
      return 1;
    }
    auto wrapped_result = result_future.get();
    final_code = wrapped_result.code;
    AINFO << "[Client] Final: count=" << wrapped_result.result->final_count()
          << " time=" << wrapped_result.result->execution_time_ms() << " ms";
  } catch (const std::exception& e) {
    AERROR << "Get result error: " << e.what();
  }

  // Cleanup: wait for server execution thread to finish
  {
    std::lock_guard<std::mutex> lock(goals_mutex);
    for (auto& p : active_goals) p.second->canceled = true;
  }
  auto start_cleanup = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_cleanup <
         std::chrono::seconds(2)) {
    std::lock_guard<std::mutex> lock(goals_mutex);
    for (auto it = active_goals.begin(); it != active_goals.end();) {
      if (!it->second->running) {
        if (it->second->execution_thread.joinable())
          it->second->execution_thread.join();
        it = active_goals.erase(it);
      } else {
        ++it;
      }
    }
    if (active_goals.empty()) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  AINFO << "Demo completed (intra-process).";
  return (final_code == ResultCode::SUCCEEDED) ? 0 : 1;
}
