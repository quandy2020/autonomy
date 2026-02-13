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
 * Demo: Action client (like autolink/examples/cpp/action_client.cpp).
 * Sends ExampleAction goals to "example_action". Run demo_action_server first.
 * Usage: demo_action_client [target_number] [cancel_after_seconds]
 *   target_number: 1-100 (default 10)
 *   cancel_after_seconds: if >0, cancel goal after N seconds (default 0)
 */

#include "autolink/action/action.hpp"
#include "autolink/autolink.hpp"
#include "autolink/common/log.hpp"
#include "autolink/common/state.hpp"
#include "proto/examples.pb.h"

#include <chrono>
#include <iomanip>
#include <thread>

using autolink::action::ResultCode;
using autolink::examples::proto::ExampleAction;

int main(int argc, char* argv[]) {
  autolink::Init(argv[0]);

  const char* node_name_env = std::getenv("AUTOLINK_NODE_NAME");
  std::string node_name =
      node_name_env ? node_name_env : "demo_action_client";
  auto node = autolink::CreateNode(node_name);

  AINFO << "Creating demo action client for ExampleAction...";

  auto action_client =
      autolink::action::CreateClient<ExampleAction>(node, "example_action");
  if (!action_client) {
    AERROR << "Failed to create action client!";
    return 1;
  }

  int wait_count = 0;
  while (!action_client->ActionServerIsReady() && wait_count < 200) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    wait_count++;
  }
  if (!action_client->ActionServerIsReady()) {
    AERROR << "Action server not ready after 20s!";
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::seconds(2));

  ExampleAction::Goal goal;
  int32_t target = (argc > 1) ? std::atoi(argv[1]) : 10;
  if (target <= 0 || target > 100) {
    AWARN << "Invalid target_number " << target << ", using 10";
    target = 10;
  }
  goal.set_target_number(target);
  goal.set_task_name("Count to " + std::to_string(target));

  AINFO << "Sending goal: " << goal.task_name();

  autolink::action::Client<ExampleAction>::SendGoalOptions options;
  options.goal_response_callback = [](auto goal_handle) {
    AINFO << (goal_handle ? "Goal ACCEPTED" : "Goal REJECTED");
  };
  options.feedback_callback = [](auto, auto feedback) {
    AINFO << "Progress: " << feedback->current_progress() << " ("
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
    AINFO << "Result: " << status
          << " count=" << wrapped_result.result->final_count()
          << " time=" << wrapped_result.result->execution_time_ms() << " ms";
  };

  auto goal_future = action_client->AsyncSendGoal(goal, options);
  auto goal_handle = goal_future.get();
  if (!goal_handle) {
    AERROR << "Failed to send goal or goal rejected!";
    return 1;
  }

  int cancel_after_seconds = (argc > 2) ? std::atoi(argv[2]) : 0;
  std::thread monitor_thread(
      [action_client, goal_handle, cancel_after_seconds]() {
        if (cancel_after_seconds > 0) {
          std::this_thread::sleep_for(
              std::chrono::seconds(cancel_after_seconds));
          auto status = static_cast<autolink::action::GoalStatus>(
              goal_handle->GetStatus());
          if (status != autolink::action::GoalStatus::SUCCEEDED &&
              status != autolink::action::GoalStatus::CANCELED &&
              status != autolink::action::GoalStatus::ABORTED) {
            try {
              action_client->AsyncCancelGoal(goal_handle).get();
            } catch (const std::exception& e) {
              AERROR << "Cancel error: " << e.what();
            }
          }
        }
      });

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
      if (autolink::IsShutdown()) {
        try {
          action_client->AsyncCancelGoal(goal_handle).get();
        } catch (...) {}
      }
      monitor_thread.detach();
      return 1;
    }
    auto wrapped_result = result_future.get();
    final_code = wrapped_result.code;
    AINFO << "Final: count=" << wrapped_result.result->final_count()
          << " time=" << wrapped_result.result->execution_time_ms() << " ms";
  } catch (const std::exception& e) {
    AERROR << "Get result error: " << e.what();
  }

  if (monitor_thread.joinable()) monitor_thread.join();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  return (final_code == ResultCode::SUCCEEDED) ? 0 : 1;
}
