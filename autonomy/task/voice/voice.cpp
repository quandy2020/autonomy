/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/voice/voice.hpp"

#include "autonomy/common/logging.hpp"
#include <automsgs/msgs/vehicle_msgs/robot_task_type.pb.h>

namespace autonomy {
namespace task {
namespace {

namespace tp = ::autonomy::task::proto;
using RobotTaskType = ::automsgs::msgs::vehicle_msgs::RobotTaskType;

}  // namespace

RobotTaskType VoiceTask::GetTaskType() const {
    return RobotTaskType::ROBOT_TASK_VOICE;
}

void VoiceTask::SetNode(std::shared_ptr<autolink::Node> node) {
    node_ = std::move(node);
}

void VoiceTask::SetSubmitNavigation(SubmitNavigation submit) {
    submit_navigation_ = std::move(submit);
}

void VoiceTask::SetSubmitTracking(SubmitTracking submit) {
    submit_tracking_ = std::move(submit);
}

void VoiceTask::SetSubmitCharging(SubmitCharging submit) {
    submit_charging_ = std::move(submit);
}

void VoiceTask::SetSubmitExploration(SubmitExploration submit) {
    submit_exploration_ = std::move(submit);
}

void VoiceTask::SetCancelDomains(CancelDomains cancel) {
    cancel_domains_ = std::move(cancel);
}

bool VoiceTask::OnInitialize(const tp::TaskServerOptions& /*options*/) {
    return true;
}

bool VoiceTask::Cancel() {
    StopAll();
    SetLifecycle(TaskLifecycle::kCanceled);
    status_ = tp::VOICE_STATUS_CANCELED;
    detail_ = "voice cancelled";
    return true;
}

void VoiceTask::StopAll() {
    if (cancel_domains_) {
        cancel_domains_();
    }
}

bool VoiceTask::DispatchStart(const tp::VoiceGoal& goal) {
    intent_ = goal.intent();
    status_ = tp::VOICE_STATUS_DISPATCHING;
    detail_ = "dispatching";

    switch (goal.intent()) {
        case tp::VOICE_INTENT_NAVIGATE: {
            if (!goal.has_navigate() || !submit_navigation_) {
                detail_ = "navigate payload / dispatcher unavailable";
                return false;
            }
            StopAll();
            if (!submit_navigation_(goal.navigate())) {
                detail_ = "navigation submit failed";
                return false;
            }
            break;
        }
        case tp::VOICE_INTENT_FOLLOW: {
            if (!goal.has_follow() || !submit_tracking_) {
                detail_ = "follow payload / dispatcher unavailable";
                return false;
            }
            StopAll();
            if (!submit_tracking_(goal.follow())) {
                detail_ = "tracking submit failed";
                return false;
            }
            break;
        }
        case tp::VOICE_INTENT_DOCK:
        case tp::VOICE_INTENT_UNDOCK: {
            if (!goal.has_charge() || !submit_charging_) {
                detail_ = "charge payload / dispatcher unavailable";
                return false;
            }
            StopAll();
            if (!submit_charging_(goal.charge())) {
                detail_ = "charging submit failed";
                return false;
            }
            break;
        }
        case tp::VOICE_INTENT_EXPLORE: {
            if (!goal.has_explore() || !submit_exploration_) {
                detail_ = "explore payload / dispatcher unavailable";
                return false;
            }
            StopAll();
            if (!submit_exploration_(goal.explore())) {
                detail_ = "exploration submit failed";
                return false;
            }
            break;
        }
        case tp::VOICE_INTENT_STOP:
        case tp::VOICE_INTENT_CANCEL_ALL: {
            StopAll();
            status_ = tp::VOICE_STATUS_SUCCEEDED;
            detail_ = "cancelled all tasks";
            SetLifecycle(TaskLifecycle::kSucceeded);
            SetProgress(1.f, detail_);
            return true;
        }
        default:
            detail_ = "unsupported voice intent";
            return false;
    }

    // Hand-off complete: domain task owns execution; terminate Voice stream.
    status_ = tp::VOICE_STATUS_SUCCEEDED;
    detail_ = "dispatched";
    SetLifecycle(TaskLifecycle::kSucceeded);
    SetProgress(1.f, detail_);
    return true;
}

bool VoiceTask::OnGoal(const tp::VoiceGoal& goal) {
    if (goal.command() == tp::VOICE_CMD_CANCEL ||
        goal.command() == tp::VOICE_CMD_STOP_ALL ||
        goal.intent() == tp::VOICE_INTENT_STOP ||
        goal.intent() == tp::VOICE_INTENT_CANCEL_ALL) {
        StopAll();
        intent_ = goal.intent();
        status_ = tp::VOICE_STATUS_SUCCEEDED;
        detail_ = "cancelled all tasks";
        SetLifecycle(TaskLifecycle::kSucceeded);
        SetProgress(1.f, detail_);
        return true;
    }

    if (!DispatchStart(goal)) {
        status_ = tp::VOICE_STATUS_FAILED;
        SetLifecycle(TaskLifecycle::kFailed);
        SetProgress(0.f, detail_);
        AWARN << "VoiceTask: " << detail_;
        return false;
    }
    return true;
}

void VoiceTask::FillFeedback(tp::VoiceFeedback* feedback) const {
    feedback->set_status(status_);
    feedback->set_intent(intent_);
    feedback->set_detail(detail_);
    *feedback->mutable_progress() = progress_;
}

void VoiceTask::FillResult(tp::VoiceResult* result) const {
    *result->mutable_result() = MakeTaskResult();
    result->set_final_status(status_);
    result->set_intent(intent_);
}

}  // namespace task
}  // namespace autonomy
