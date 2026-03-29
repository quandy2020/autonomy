/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan). All Rights Reserved.
 *
 * Action client example: send a goal, optional feedback stream, wait for
 * terminal result. Start action_listener first.
 *****************************************************************************/

#include <chrono>
#include <memory>
#include <thread>

#include "autolink/action/action.hpp"
#include "autolink/action/types.hpp"
#include "autolink/autolink.hpp"
#include "examples.pb.h"

namespace {

namespace ae = autolink::examples;

struct SimpleMessageActionTraits {
    using Goal = ae::SimpleMessageAction_Goal;
    using Feedback = ae::SimpleMessageAction_Feedback;
    using Result = ae::SimpleMessageAction_Result;
};

using ActionClient = autolink::action::Client<SimpleMessageActionTraits>;
using GoalHandle =
    autolink::action::ClientGoalHandle<SimpleMessageActionTraits>;

constexpr char kActionName[] = "examples/simple_message_action";
constexpr auto kWaitServerReady = std::chrono::milliseconds(100);
constexpr auto kAcceptTimeout = std::chrono::seconds(30);
constexpr auto kResultTimeout = std::chrono::seconds(60);

void LogActionOutcome(const std::shared_ptr<GoalHandle>& handle,
                      const GoalHandle::WrappedResult& wr) {
    AINFO << "Action terminal: " << autolink::action::ToString(wr.code)
          << " (GoalStatus=" << static_cast<int>(wr.code) << ")  handle_ok="
          << " succ=" << handle->IsSucceeded()
          << " canceled=" << handle->IsCanceled()
          << " aborted=" << handle->IsAborted();
    if (wr.result) {
        AINFO << "Result payload: success="
              << (wr.result->success() ? "true" : "false");
    }
}

}  // namespace

int main(int argc, char* argv[]) {
    if (!autolink::Init(argv[0])) {
        return 1;
    }

    auto node = autolink::CreateNode("simple_action_client2");
    auto client = autolink::action::CreateClient<SimpleMessageActionTraits>(
        node, kActionName);

    AINFO << "Waiting for action server \"" << kActionName << "\"...";
    while (autolink::OK() && !client->ActionServerIsReady()) {
        std::this_thread::sleep_for(kWaitServerReady);
    }
    if (!autolink::OK()) {
        return 1;
    }

    SimpleMessageActionTraits::Goal goal;
    goal.set_text("hello from action_talker2");

    ActionClient::SendGoalOptions opts;
    opts.goal_response_callback = [](std::shared_ptr<GoalHandle> gh) {
        if (!gh) {
            AWARN << "Goal rejected by server";
        }
    };
    opts.feedback_callback =
        [](std::shared_ptr<GoalHandle> gh,
           std::shared_ptr<const SimpleMessageActionTraits::Feedback> fb) {
            (void)gh;
            if (fb) {
                AINFO << "Feedback index=" << fb->index();
            }
        };
    // Result is delivered once via AsyncGetResult().wait(); avoid duplicate
    // logging with a second result_callback.

    const auto accepted_future = client->AsyncSendGoal(goal, opts);
    if (accepted_future.wait_for(kAcceptTimeout) != std::future_status::ready) {
        AERROR << "Timeout waiting for goal acceptance";
        return 1;
    }

    std::shared_ptr<GoalHandle> handle = accepted_future.get();
    if (!handle) {
        AERROR << "Goal handle null after send";
        return 1;
    }

    AINFO << "Goal accepted, id="
          << autolink::action::ToString(handle->GetGoalId())
          << " — waiting for result...";

    auto result_future = handle->AsyncGetResult();
    if (result_future.wait_for(kResultTimeout) != std::future_status::ready) {
        AERROR << "Timeout waiting for action result";
        return 1;
    }

    try {
        LogActionOutcome(handle, result_future.get());
    } catch (const std::exception& e) {
        AERROR << "Result error: " << e.what();
        return 1;
    }

    return 0;
}
