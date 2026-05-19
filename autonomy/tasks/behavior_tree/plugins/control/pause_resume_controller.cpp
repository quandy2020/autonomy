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
 */

#include "autonomy/tasks/behavior_tree/plugins/control/pause_resume_controller.hpp"

#include "autonomy/common/log.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace control {

PauseResumeController::PauseResumeController(const std::string& xml_tag_name,
                                             const BT::NodeConfiguration& conf)
    : BT::ControlNode(xml_tag_name, conf), state_(RESUMED) {
    auto node =
        config().blackboard->get<std::shared_ptr<::autolink::Node>>("node");
    std::string pause_service_name;
    getInput("pause_service_name", pause_service_name);
    pause_srv_ = node->CreateService<Trigger::Request, Trigger::Response>(
        pause_service_name,
        [this](const std::shared_ptr<Trigger::Request>& request,
               std::shared_ptr<Trigger::Response>& response) {
            this->pauseServiceCallback(request, response);
        });

    std::string resume_service_name;
    getInput("resume_service_name", resume_service_name);
    resume_srv_ = node->CreateService<Trigger::Request, Trigger::Response>(
        resume_service_name,
        [this](const std::shared_ptr<Trigger::Request>& request,
               std::shared_ptr<Trigger::Response>& response) {
            this->resumeServiceCallback(request, response);
        });
}

BT::NodeStatus PauseResumeController::tick() {
    unsigned int children_count = children_nodes_.size();
    if (children_count < 1 || children_count > 4) {
        throw std::runtime_error(
            "PauseNode must have at least one and at most four children "
            "(currently has " +
            std::to_string(children_count) + ")");
    }

    if (status() == BT::NodeStatus::IDLE) {
        state_ = RESUMED;
    }
    setStatus(BT::NodeStatus::RUNNING);

    // If pause / resume requested, reset children and switch to transient
    // child
    if (state_ == PAUSE_REQUESTED || state_ == RESUME_REQUESTED) {
        resetChildren();
        switchToNextState();
    }

    // Return RUNNING and do nothing if specific child is not used
    const uint16_t child_idx = child_indices.at(state_);
    if (children_nodes_.size() <= child_idx) {
        switchToNextState();
        return BT::NodeStatus::RUNNING;
    }

    // If child is used, tick it
    const BT::NodeStatus child_status =
        children_nodes_[child_indices.at(state_)]->executeTick();

    switch (child_status) {
        case BT::NodeStatus::RUNNING:
            return BT::NodeStatus::RUNNING;
        case BT::NodeStatus::SUCCESS:
        case BT::NodeStatus::SKIPPED:
            if (state_ == RESUMED) {
                // Resumed child returned SUCCESS, we're done
                return BT::NodeStatus::SUCCESS;
            }
            switchToNextState();
            // If any branch other than RESUMED returned SUCCESS, keep ticking
            return BT::NodeStatus::RUNNING;
        case BT::NodeStatus::FAILURE:
            AERROR << state_names.at(state_).c_str()
                   << " child returned FAILURE";
            return BT::NodeStatus::FAILURE;
        default:
            throw std::runtime_error("A child node must never return IDLE");
    }
}

void PauseResumeController::halt() {
    BT::ControlNode::halt();
    state_ = RESUMED;
}

void PauseResumeController::switchToNextState() {
    static const std::map<state_t, state_t> next_states = {
        {PAUSE_REQUESTED, ON_PAUSE},
        {ON_PAUSE, PAUSED},
        {RESUME_REQUESTED, ON_RESUME},
        {ON_RESUME, RESUMED}};

    if (state_ == PAUSED || state_ == RESUMED) {
        // No next state, do nothing
        return;
    }

    state_ = next_states.at(state_);
    AINFO << state_names.at(state_).c_str() << " switched to state";
}

void PauseResumeController::pauseServiceCallback(
    const std::shared_ptr<Trigger::Request>& /*request*/,
    std::shared_ptr<Trigger::Response>& response) {
    if (state_ != PAUSED) {
        AINFO << "Received pause request";
        response->set_success(true);
        state_ = PAUSE_REQUESTED;
        return;
    }

    std::string warn_message =
        "PauseResumeController BT node already in state PAUSED ";

    AWARN << warn_message.c_str();
    response->set_success(false);
    response->set_message(warn_message);
}

void PauseResumeController::resumeServiceCallback(
    const std::shared_ptr<Trigger::Request>& /*request*/,
    std::shared_ptr<Trigger::Response>& response) {
    if (state_ == PAUSED) {
        AINFO << "Received resume request";
        response->set_success(true);
        state_ = RESUME_REQUESTED;
        return;
    }

    std::string warn_message =
        "PauseResumeController BT node not in state PAUSED ";
    AWARN << warn_message.c_str();
    response->set_success(false);
    response->set_message(warn_message);
}

}  // namespace control
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

BT_REGISTER_NODES(factory) {
    BT::NodeBuilder builder = [](const std::string& name,
                                 const BT::NodeConfiguration& config) {
        return std::make_unique<autonomy::tasks::behavior_tree::plugins::
                                    control::PauseResumeController>(name,
                                                                    config);
    };

    factory.registerBuilder<autonomy::tasks::behavior_tree::plugins::control::
                                PauseResumeController>("PauseResumeController",
                                                       builder);
}