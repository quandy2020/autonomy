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
 * @file coverage_endpoint_test.cpp
 * @brief Verify actual native coverage Action acceptance and terminal delivery.
 */

#include "autolink/autolink.hpp"
#include "autolink/action/client.hpp"
#include "autonomy/task/navigation/coverage/action_endpoint.hpp"
#include "autonomy/task/navigation/coverage/coverage_server.hpp"
#include <iostream>
#include <thread>

namespace coverage = autonomy::task::coverage;
using Action = coverage::ComputeAction;
using Client = autolink::action::Client<Action>;
void Require(bool value, const std::string& message) {
  if (!value) throw std::runtime_error(message);
}
Action::Goal Rectangle() {
  Action::Goal goal;
  auto* request = goal.mutable_request();
  request->set_frame_id("map");
  auto* ring = request->add_polygons();
  for (const auto [x, y] : {std::pair{0., 0.}, {40., 0.}, {40., 30.}, {0., 30.}}) {
    auto* p = ring->add_coordinates(); p->set_axis1(x); p->set_axis2(y);
  }
  return goal;
}
int main(int argc, char** argv) {
  if (!autolink::Init(argv[0])) return 1;
  try {
    auto node = autolink::CreateNode("coverage_endpoint_test");
    coverage::CoverageServer planner;
    {
      coverage::ActionEndpoint<Action> endpoint(node, "/test/coverage/compute",
        [&](const auto& handle, const auto& canceled) {
          auto result = std::make_shared<Action::Result>(planner.Compute(handle->GetGoal()->request(), canceled));
          if (handle->IsCanceling()) handle->Canceled(result);
          else if (result->error_code() == 0) handle->Succeed(result);
          else handle->Abort(result);
        });
      auto client = std::make_shared<Client>(node, "/test/coverage/compute");
      Require(client->WaitForActionServer(std::chrono::seconds(3)), "Native endpoint unavailable");
      for (bool valid : {true, false}) {
        auto acceptance = client->AsyncSendGoal(valid ? Rectangle() : Action::Goal());
        Require(acceptance.wait_for(std::chrono::seconds(5)) == std::future_status::ready, "Goal acceptance not delivered");
        auto handle = acceptance.get(); Require(static_cast<bool>(handle), "Expected accepted goal");
        auto result = client->AsyncGetResult(handle);
        Require(result.wait_for(std::chrono::seconds(5)) == std::future_status::ready, "Goal terminal result not delivered");
        auto wrapped = result.get(); Require(static_cast<bool>(wrapped.result), "Missing native result");
        Require(valid ? wrapped.code == autolink::action::ResultCode::SUCCEEDED
                      : wrapped.code == autolink::action::ResultCode::ABORTED,
                "Wrong native terminal status");
        Require(valid ? wrapped.result->error_code() == coverage::proto::NONE
                      : wrapped.result->error_code() == coverage::proto::INVALID_REQUEST,
                "Domain result was not preserved");
        std::cout << "PASS " << (valid ? "native_success_delivery" : "native_failure_delivery") << '\n';
      }
    }
    {
      coverage::ActionEndpoint<Action> endpoint(node, "/test/coverage/cancel",
        [](const auto& handle, const auto& canceled) {
          while (!canceled()) std::this_thread::sleep_for(std::chrono::milliseconds(5));
          auto result = std::make_shared<Action::Result>(); result->set_error_code(coverage::proto::CANCELED);
          if (handle->IsCanceling()) handle->Canceled(result); else handle->Abort(result);
        });
      auto client = std::make_shared<Client>(node, "/test/coverage/cancel");
      Require(client->WaitForActionServer(std::chrono::seconds(3)), "Cancel endpoint unavailable");
      auto acceptance = client->AsyncSendGoal(Rectangle());
      Require(acceptance.wait_for(std::chrono::seconds(5)) == std::future_status::ready, "Cancel goal acceptance not delivered");
      auto handle = acceptance.get(); Require(static_cast<bool>(handle), "Cancel goal rejected");
      auto cancel = client->AsyncCancelGoal(handle);
      Require(cancel.wait_for(std::chrono::seconds(5)) == std::future_status::ready && cancel.get(), "Cancellation was not accepted");
      auto result = client->AsyncGetResult(handle);
      Require(result.wait_for(std::chrono::seconds(5)) == std::future_status::ready, "Cancellation terminal receipt missing");
      auto wrapped = result.get();
      Require(wrapped.code == autolink::action::ResultCode::CANCELED && wrapped.result && wrapped.result->error_code() == coverage::proto::CANCELED, "Canceled result lost");
      std::cout << "PASS native_targeted_cancel_delivery\n";
    }
    std::cout << "3/3 native endpoint checks passed\n";
    autolink::Clear();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL native endpoint: " << error.what() << '\n';
    autolink::Clear();
    return 1;
  }
}
