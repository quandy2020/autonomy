/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan). All Rights Reserved.
 *
 * Action client example: sends a SimpleMessageAction goal and prints feedback
 * and result. Start action_listener first.
 *****************************************************************************/

#include <chrono>
#include <memory>
#include <thread>

#include "autolink/action/action.hpp"
#include "autolink/action/types.hpp"
#include "autolink/autolink.hpp"
#include "examples.pb.h"

namespace ae = autolink::examples;

struct SimpleMessageActionTraits {
    using Goal = ae::SimpleMessageAction_Goal;
    using Feedback = ae::SimpleMessageAction_Feedback;
    using Result = ae::SimpleMessageAction_Result;
};

static constexpr char kActionName[] = "examples/simple_message_action";

int main(int argc, char* argv[]) {
    if (!autolink::Init(argv[0])) {
        return 1;
    }

    auto node = autolink::CreateNode("simple_action_client");
    auto client = autolink::action::CreateClient<SimpleMessageActionTraits>(
        node, kActionName);

    AINFO << "Waiting for action server...";
    while (autolink::OK() && !client->ActionServerIsReady()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!autolink::OK()) {
        return 1;
    }

    ae::SimpleMessageAction_Goal goal;
    goal.set_text("hello from action_talker");

    autolink::action::Client<SimpleMessageActionTraits>::SendGoalOptions opts;

    opts.goal_response_callback =
        [](std::shared_ptr<
            autolink::action::ClientGoalHandle<SimpleMessageActionTraits>>
               gh) {
            if (gh) {
                AINFO << "Goal accepted by server, id="
                      << autolink::action::ToString(gh->GetGoalId());
            } else {
                AWARN << "Goal rejected by server";
            }
        };

    // 反馈回调
    opts.feedback_callback =
        [](std::shared_ptr<
               autolink::action::ClientGoalHandle<SimpleMessageActionTraits>>,
           std::shared_ptr<const SimpleMessageActionTraits::Feedback> fb) {
            if (fb) {
                AINFO << "Client feedback: index=" << fb->index();
            }
        };
    // 结果回调
    opts.result_callback = [](const autolink::action::ClientGoalHandle<
                               SimpleMessageActionTraits>::WrappedResult& wr) {
        AINFO << "Result code=" << static_cast<int>(wr.code);
        if (wr.result) {
            AINFO << "Result success="
                  << (wr.result->success() ? "true" : "false");
        }
    };

    auto accepted_future = client->AsyncSendGoal(goal, opts);
    auto status = accepted_future.wait_for(std::chrono::seconds(30));
    if (status != std::future_status::ready) {
        AERROR << "Timeout waiting for goal acceptance";
        return 1;
    }
    auto handle = accepted_future.get();
    if (!handle) {
        AERROR << "Goal rejected or failed to get handle";
        return 1;
    }
    AINFO << "Goal accepted, waiting for terminal result...";

    auto result_future = handle->AsyncGetResult();
    auto rs = result_future.wait_for(std::chrono::seconds(60));
    if (rs == std::future_status::timeout) {
        AERROR << "Timeout waiting for action result";
        return 1;
    }

    try {
        const auto& wr = result_future.get();
        AINFO << "Final: code=" << static_cast<int>(wr.code)
              << " (see result_callback logs above)";
    } catch (const std::exception& e) {
        AERROR << "Result future error: " << e.what();
        return 1;
    }

    return 0;
}
