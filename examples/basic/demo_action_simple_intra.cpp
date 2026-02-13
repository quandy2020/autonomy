/*
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
 * Demo: 使用 autonomy::common::SimpleActionServer 的进程内 Action 示例。
 * 同一进程中启动 ExampleAction 的 server（SimpleActionServer）和 client，演示进程内通信。
 * Usage: demo_action_simple_intra [target_number]
 *   target_number: 1-100，默认 5
 */

#include "autonomy/common/simple_action_server.hpp"
#include "autolink/action/action.hpp"
#include "autolink/autolink.hpp"
#include "autolink/common/log.hpp"
#include "autolink/common/state.hpp"
#include "proto/examples.pb.h"

#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <memory>
#include <thread>

using autolink::action::ResultCode;
using autolink::examples::proto::ExampleAction;

namespace {

int64_t DurationMs(const std::chrono::steady_clock::time_point& start) {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now() - start)
      .count();
}

bool IsValidGoal(const ExampleAction::Goal* goal) {
  return goal && goal->target_number() > 0 && goal->target_number() <= 100;
}

}  // namespace

int main(int argc, char* argv[]) {
  autolink::Init(argv[0]);

  const char* node_name_env = std::getenv("AUTOLINK_NODE_NAME");
  std::string node_name =
      node_name_env ? node_name_env : "demo_action_simple_intra";
  auto node = autolink::CreateNode(node_name);

  AINFO << "Demo: SimpleActionServer + client (intra-process)";

  // 执行状态：在 execute_callback 的多次调用之间保持
  struct ExecState {
    int32_t current = 0;
    int32_t target = 0;
    std::string task_name;
    std::chrono::steady_clock::time_point start_time;
    bool initialized = false;
  };
  auto exec_state = std::make_shared<ExecState>();

  // 先声明 server 指针，再在 lambda 中捕获，以便在 execute 里调用 GetCurrentGoal/PublishFeedback/SucceededCurrent
  std::shared_ptr<autonomy::common::SimpleActionServer<ExampleAction>> server;

  server = std::make_shared<autonomy::common::SimpleActionServer<ExampleAction>>(
      node,
      "example_action",
      [&server, exec_state]() {
        if (!server) return;

        auto goal = server->GetCurrentGoal();
        if (!goal) return;

        if (!exec_state->initialized) {
          if (!IsValidGoal(goal.get())) {
            auto result = std::make_shared<ExampleAction::Result>();
            result->set_success(false);
            result->set_final_count(0);
            result->set_message("Invalid target_number (must be 1-100)");
            result->set_execution_time_ms(0);
            server->TerminateCurrent(result);
            return;
          }
          exec_state->target = goal->target_number();
          exec_state->task_name = goal->task_name();
          exec_state->start_time = std::chrono::steady_clock::now();
          exec_state->current = 0;
          exec_state->initialized = true;
        }

        if (server->IsCancelRequested()) {
          auto result = std::make_shared<ExampleAction::Result>();
          result->set_success(false);
          result->set_final_count(exec_state->current);
          result->set_message("Canceled by client");
          result->set_execution_time_ms(DurationMs(exec_state->start_time));
          server->TerminateCurrent(result);
          exec_state->initialized = false;
          return;
        }

        exec_state->current++;
        int32_t t = exec_state->target;

        if (t <= 10 || exec_state->current % std::max(1, t / 10) == 0 ||
            exec_state->current >= t) {
          auto feedback = std::make_shared<ExampleAction::Feedback>();
          feedback->set_current_progress(exec_state->current);
          feedback->set_percentage(
              static_cast<double>(exec_state->current) / static_cast<double>(t));
          feedback->set_status_message(
              "Processing: " + std::to_string(exec_state->current) + "/" +
              std::to_string(t));
          server->PublishFeedback(feedback);
        }

        if (exec_state->current >= t) {
          auto result = std::make_shared<ExampleAction::Result>();
          result->set_success(true);
          result->set_final_count(exec_state->current);
          result->set_message("Action completed: " + exec_state->task_name);
          result->set_execution_time_ms(DurationMs(exec_state->start_time));
          server->SucceededCurrent(result);
          exec_state->initialized = false;
          AINFO << "[SimpleActionServer] Goal succeeded: " << exec_state->task_name
                << " (" << exec_state->current << "/" << t << ", "
                << result->execution_time_ms() << " ms)";
        } else {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
      },
      nullptr,
      std::chrono::milliseconds(500),
      false);

  server->SetGoalCallback(
      [](const autolink::action::GoalUUID&,
         std::shared_ptr<const ExampleAction::Goal> goal)
          -> autolink::action::GoalResponse {
        if (!goal) return autolink::action::GoalResponse::REJECT;
        if (!IsValidGoal(goal.get())) {
          AERROR << "Invalid goal (target_number must be 1-100) - REJECTING";
          return autolink::action::GoalResponse::REJECT;
        }
        AINFO << "[SimpleActionServer] Goal ACCEPTED: " << goal->task_name()
              << " (target: " << goal->target_number() << ")";
        return autolink::action::GoalResponse::ACCEPT_AND_EXECUTE;
      });

  server->Activate();
  AINFO << "[SimpleActionServer] Ready on example_action";

  // ---- 进程内 Client ----
  auto action_client =
      autolink::action::CreateClient<ExampleAction>(node, "example_action");

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

  ExampleAction::Goal goal;
  int32_t target = (argc > 1) ? std::atoi(argv[1]) : 5;
  if (target <= 0 || target > 100) {
    AWARN << "Invalid target_number " << target << ", using 5";
    target = 5;
  }
  goal.set_target_number(target);
  goal.set_task_name("Simple intra-process count to " + std::to_string(target));

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
    int wait_count_result = 0;
    std::future_status status;
    while (wait_count_result < 300 && autolink::OK()) {
      status = result_future.wait_for(std::chrono::milliseconds(100));
      if (status != std::future_status::timeout) break;
      wait_count_result++;
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

  server->Deactivate();
  AINFO << "Demo completed (SimpleActionServer intra-process).";
  return (final_code == ResultCode::SUCCEEDED) ? 0 : 1;
}
