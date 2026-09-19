/*
 * Copyright 2026 The Openbot Authors
 *
 * Voice task: single GoalChannel ingress; dispatches to domain tasks.
 */

#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "autolink/node/node.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/task/common/typed_task.hpp"
#include <automsgs/task/charging.pb.h>
#include <automsgs/task/exploration.pb.h>
#include <automsgs/task/navigation.pb.h>
#include <automsgs/task/tracker.pb.h>
#include <automsgs/task/voice.pb.h>

namespace autonomy {
namespace task {

/**
 * Bridge VoiceService → VoiceGoal; orchestration lives here, not in Bridge.
 */
class VoiceTask
    : public TypedTaskAppBase<::autonomy::task::proto::VoiceGoal,
                              ::autonomy::task::proto::VoiceFeedback,
                              ::autonomy::task::proto::VoiceResult>
{
public:
    static constexpr bool kUsesNavigationClient = false;

    using SubmitNavigation =
        std::function<bool(const ::autonomy::task::proto::NavigationGoal&)>;
    using SubmitTracking =
        std::function<bool(const ::autonomy::task::proto::TrackerGoal&)>;
    using SubmitCharging =
        std::function<bool(const ::autonomy::task::proto::ChargingGoal&)>;
    using SubmitExploration =
        std::function<bool(const ::autonomy::task::proto::ExplorationGoal&)>;
    using CancelDomains = std::function<void()>;

    AUTONOMY_SMART_PTR_DEFINITIONS(VoiceTask)

    ::automsgs::msgs::vehicle_msgs::RobotTaskType GetTaskType()
        const override;

    void SetNode(std::shared_ptr<autolink::Node> node);
    void SetSubmitNavigation(SubmitNavigation submit);
    void SetSubmitTracking(SubmitTracking submit);
    void SetSubmitCharging(SubmitCharging submit);
    void SetSubmitExploration(SubmitExploration submit);
    void SetCancelDomains(CancelDomains cancel);

    bool Cancel() override;

protected:
    bool OnInitialize(
        const ::autonomy::task::proto::TaskServerOptions& options) override;
    bool OnGoal(const ::autonomy::task::proto::VoiceGoal& goal) override;
    void FillFeedback(
        ::autonomy::task::proto::VoiceFeedback* feedback) const override;
    void FillResult(
        ::autonomy::task::proto::VoiceResult* result) const override;

private:
    bool DispatchStart(const ::autonomy::task::proto::VoiceGoal& goal);
    void StopAll();

    std::shared_ptr<autolink::Node> node_;
    SubmitNavigation submit_navigation_;
    SubmitTracking submit_tracking_;
    SubmitCharging submit_charging_;
    SubmitExploration submit_exploration_;
    CancelDomains cancel_domains_;

    ::autonomy::task::proto::VoiceStatus status_{
        ::autonomy::task::proto::VOICE_STATUS_IDLE};
    ::autonomy::task::proto::VoiceIntent intent_{
        ::autonomy::task::proto::VOICE_INTENT_UNKNOWN};
    std::string detail_;
};

}  // namespace task
}  // namespace autonomy
