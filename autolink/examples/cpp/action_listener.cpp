/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan). All Rights Reserved.
 *
 * Action server example: accept goals, stream feedback, succeed. Pair with
 * action_talker.cpp (client).
 *****************************************************************************/

#include <chrono>
#include <memory>
#include <thread>

#include "autolink/action/simple_action_server.hpp"
#include "autolink/autolink.hpp"
#include "examples.pb.h"

namespace {

namespace ae = autolink::examples;

struct SimpleMessageActionTraits {
    using Goal = ae::SimpleMessageAction_Goal;
    using Feedback = ae::SimpleMessageAction_Feedback;
    using Result = ae::SimpleMessageAction_Result;
};

using ActionServer =
    autolink::action::SimpleActionServer<SimpleMessageActionTraits>;
using GoalPtr = std::shared_ptr<const SimpleMessageActionTraits::Goal>;
using FeedbackPtr = std::shared_ptr<SimpleMessageActionTraits::Feedback>;
using ResultPtr = std::shared_ptr<SimpleMessageActionTraits::Result>;

constexpr char kActionName[] = "examples/simple_message_action";
constexpr int kFeedbackSteps = 30;
constexpr auto kFeedbackPeriod = std::chrono::milliseconds(1000);
constexpr char kNodeName[] = "simple_action_server";

void RunAcceptedGoal(const std::shared_ptr<ActionServer>& server) {
    if (!server) {
        return;
    }

    while (autolink::OK()) {
        GoalPtr goal = server->GetCurrentGoal();
        if (!goal) {
            AERROR << "No current goal in execute callback";
            return;
        }

        AINFO << "Executing goal, text=\"" << goal->text() << "\"";

        bool preempted = false;
        for (int i = 0; i < kFeedbackSteps; ++i) {
            if (server->IsCancelRequested()) {
                ResultPtr aborted =
                    std::make_shared<SimpleMessageActionTraits::Result>();
                aborted->set_success(false);
                server->TerminateCurrent(aborted);
                AINFO << "Goal terminated (cancel)";
                return;
            }

            if (server->IsPreemptRequested()) {
                AINFO << "Goal preempted; handing off to pending goal";
                server->AcceptPendingGoal();
                preempted = true;
                break;
            }

            FeedbackPtr fb =
                std::make_shared<SimpleMessageActionTraits::Feedback>();
            fb->set_index(i);
            server->PublishFeedback(fb);
            AINFO << "Server sent feedback index=" << i;
            std::this_thread::sleep_for(kFeedbackPeriod);
        }

        if (preempted) {
            continue;
        }

        ResultPtr ok = std::make_shared<SimpleMessageActionTraits::Result>();
        ok->set_success(true);
        server->SucceededCurrent(ok);
        AINFO << "Goal succeeded (terminal: SUCCEEDED)";
        return;
    }
}

}  // namespace

int main(int argc, char* argv[]) {
    if (!autolink::Init(argv[0])) {
        return 1;
    }

    auto node = autolink::CreateNode(kNodeName);

    std::shared_ptr<ActionServer> server;
    server = std::make_shared<ActionServer>(
        node, kActionName, [&server]() { RunAcceptedGoal(server); });

    AINFO << "Action server ready: \"" << kActionName
          << "\" (ROS 2–style "
             "send_goal / feedback / get_result under this prefix)";

    autolink::WaitForShutdown();
    return 0;
}
