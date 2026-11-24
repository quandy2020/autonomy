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
 * @file runtime.cpp
 * @brief Native complete-coverage runtime implementation.
 */

#include "autonomy/task/navigation/coverage/runtime.hpp"
namespace autonomy::task::coverage {
CoverageRuntime::CoverageRuntime(std::shared_ptr<autolink::Node> node,
                                 TaskScheduler::SharedPtr scheduler,
                                 navigation::NavigationClient::Ptr nav,
                                 Configuration config, std::string prefix) {
  if (!node || !scheduler || !nav)
    throw RequestException(
        "Coverage runtime requires node, scheduler and navigation client");
  visualizer_ = std::make_shared<CoverageVisualizer>(node, prefix);
  planner_ = std::make_shared<CoverageServer>(config);
  navigator_ = std::make_shared<CoverageNavigator>(config, planner_, nav);
  scheduler->RegisterTask(navigator_);
  if (scheduler->GetTask(navigator_->GetTaskType()) != navigator_)
    throw RequestException("Coverage scheduler registration failed");
  tool_state_ = std::make_shared<ToolState>();
  tool_writer_ =
      node->CreateWriter<proto::ToolCommand>(prefix + "/coverage/tool_command");
  const auto state = tool_state_;
  tool_reader_ = node->CreateReader<proto::ToolAcknowledgment>(
      prefix + "/coverage/tool_ack",
      [state](const std::shared_ptr<proto::ToolAcknowledgment>& message) {
        if (!message) return;
        {
          std::lock_guard<std::mutex> lock(state->mutex);
          state->ack = *message;
        }
        state->wake.notify_all();
      });
  debug_writer_ =
      node->CreateWriter<proto::CoverageDebug>(prefix + "/coverage/debug");
  plan_writer_ = node->CreateWriter<NavPath>(prefix + "/coverage/plan");
  planning_action_ = std::make_unique<ActionEndpoint<ComputeAction>>(
      node, prefix + "/compute_coverage_path",
      [this](auto handle, const auto& cancel) {
        auto result = std::make_shared<ComputeAction::Result>(planner_->Compute(
            handle->GetGoal()->request(), cancel,
            [handle](const auto& stage, double progress) {
              auto feedback = std::make_shared<ComputeAction::Feedback>();
              feedback->set_stage(stage);
              feedback->set_progress(progress);
              handle->PublishFeedback(feedback);
            }));
        if (result->error_code() == proto::NONE) {
          visualizer_->Publish(*result);
          proto::CoverageDebug debug;
          debug.set_frame_id(result->nav_path().header().frame_id());
          debug.set_stage("complete");
          debug.set_field_wkt(result->field_wkt());
          debug.set_core_wkt(result->core_wkt());
          *debug.mutable_path() = result->nav_path();
          debug_writer_->Write(debug);
        }
        if (handle->IsCanceling())
          handle->Canceled(result);
        else if (result->error_code() == proto::NONE)
          handle->Succeed(result);
        else
          handle->Abort(result);
      });
  navigation_action_ = std::make_unique<ActionEndpoint<NavigateAction>>(
      node, prefix + "/navigate_complete_coverage",
      [this](auto handle, const auto& cancel) {
        const auto id = handle->GetGoal()->task_id();
        const bool required = handle->GetGoal()->tool_required();
        if (id.empty()) {
          auto r = std::make_shared<NavigateAction::Result>();
          r->set_error_code(proto::INVALID_REQUEST);
          r->set_error_msg("Coverage navigation requires a task_id");
          handle->Abort(r);
          return;
        }
        const auto state = tool_state_;
        const auto writer = tool_writer_;
        auto tool = [state, writer, id, required, cancel](bool enabled,
                                                          double speed) {
          proto::ToolCommand command;
          command.set_task_id(id);
          command.set_enabled(enabled);
          command.set_speed(speed);
          {
            std::lock_guard<std::mutex> lock(state->mutex);
            command.set_sequence(++state->sequence);
          }
          writer->Write(command);
          if (!required) return true;
          std::unique_lock<std::mutex> lock(state->mutex);
          const bool replied =
              state->wake.wait_for(lock, std::chrono::milliseconds(500), [&] {
                return (state->ack.task_id() == id &&
                        state->ack.sequence() == command.sequence()) ||
                       (cancel && cancel());
              });
          return replied && state->ack.task_id() == id &&
                 state->ack.sequence() == command.sequence() &&
                 state->ack.accepted() && state->ack.enabled() == enabled;
        };
        const auto writer_path = plan_writer_;
        navigator_->Execute(handle, cancel, std::move(tool),
                            [writer_path, visualizer = visualizer_](const ComputeAction::Result& result) {
          visualizer->Publish(result);
                              writer_path->Write(result.nav_path());
                            });
      });
}
void CoverageRuntime::Shutdown() {
  if (navigator_) navigator_->Cancel();
  navigation_action_.reset();
  planning_action_.reset();
  if (navigator_) navigator_->Shutdown();
  tool_reader_.reset();
  tool_writer_.reset();
  debug_writer_.reset();
}
}  // namespace autonomy::task::coverage
