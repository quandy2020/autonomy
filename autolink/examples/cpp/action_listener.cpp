/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan). All Rights Reserved.
 *
 * Action server example: receives SimpleMessageAction goals, publishes
 * feedback, then returns success. Pair with action_talker.cpp (client).
 *****************************************************************************/

#include <chrono>
#include <memory>
#include <thread>

#include "autolink/action/simple_action_server.hpp"
#include "autolink/autolink.hpp"
#include "examples.pb.h"

namespace ae = autolink::examples;

/// Maps protobuf nested messages to the Goal / Feedback / Result types the
/// action stack expects.
struct SimpleMessageActionTraits {
    using Goal = ae::SimpleMessageAction_Goal;
    using Feedback = ae::SimpleMessageAction_Feedback;
    using Result = ae::SimpleMessageAction_Result;
};

using ActionServer =
    autolink::action::SimpleActionServer<SimpleMessageActionTraits>;

static constexpr char kActionName[] = "examples/simple_message_action";

int main(int argc, char* argv[]) {
    if (!autolink::Init(argv[0])) {
        return 1;
    }

    auto node = autolink::CreateNode("simple_action_server");
    std::shared_ptr<ActionServer> server;

    server = std::make_shared<ActionServer>(node, kActionName, [&server]() {
        if (!server) {
            return;
        }
        auto goal = server->GetCurrentGoal();
        if (!goal) {
            AERROR << "Execute callback: no current goal";
            return;
        }
        AINFO << "Server executing goal, text=" << goal->text();

        for (int i = 0; i < 30; ++i) {
            if (server->IsCancelRequested()) {
                auto aborted =
                    std::make_shared<SimpleMessageActionTraits::Result>();
                aborted->set_success(false);
                server->TerminateCurrent(aborted);
                return;
            }
            auto fb = std::make_shared<SimpleMessageActionTraits::Feedback>();
            fb->set_index(i);
            server->PublishFeedback(fb);
            AINFO << "Published feedback index=" << i;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }

        auto res_msg = std::make_shared<SimpleMessageActionTraits::Result>();
        res_msg->set_success(true);
        server->SucceededCurrent(res_msg);
        AINFO << "Goal succeeded";
    });

    server->Activate();
    AINFO << "SimpleMessageAction server ready on \"" << kActionName << "\"";
    autolink::WaitForShutdown();
    server->Deactivate();
    return 0;
}
