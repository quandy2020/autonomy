/* Copyright 2026 The Navigation Authors (navigation) */
#pragma once

#include <functional>
#include "navigation/planning/interface/navigation_session.hpp"

namespace navigation::behavior {

/** @brief Snapshot-backed BT session; navigation retains goal ownership. */
class BehaviorSession final : public planning::NavigationSession
{
public:
    using Sender = std::function<bool(const BehaviorRequest&)>;
    explicit BehaviorSession(Sender sender) : sender_(std::move(sender)) {}
    void Update(const NavigationStatus& status, std::int64_t now_ns) {
        status_ = status;
        now_ns_ = now_ns;
    }
    bool SetGoal(const GoalSpec& goal) override {
        return HasActiveGoal() &&
               goal.SerializeAsString() == status_.goal().SerializeAsString();
    }
    void ClearGoal() override {
        (void)FailGoal("behavior_cancelled");
    }
    bool HasActiveGoal() const override {
        return status_.has_active_goal();
    }
    bool IsGoalReached() const override {
        return status_.goal_reached();
    }
    bool RequestRecovery(std::string_view reason) override {
        if (!HasActiveGoal() || (RecoveryStatus() != RECOVERY_REQUIRED &&
                                 RecoveryStatus() != RECOVERY_FAILED))
            return false;
        return Send(BehaviorRequest::RECOVER, RecoveryCount() + 1, reason);
    }
    bool FailGoal(std::string_view reason) {
        return HasActiveGoal() &&
               Send(BehaviorRequest::FAIL_GOAL, RecoveryCount(), reason);
    }
    lifecycle::ModuleState State() const override {
        return RecoveryStatus() == RECOVERY_REQUIRED ||
                       RecoveryStatus() == RECOVERY_FAILED
                   ? lifecycle::MODULE_STATE_FAILED
                   : lifecycle::MODULE_STATE_READY;
    }
    RecoveryState RecoveryStatus() const override {
        return status_.recovery_state();
    }
    std::uint32_t RecoveryCount() const override {
        return status_.recovery_count();
    }

private:
    bool Send(BehaviorRequest::Kind kind, std::uint32_t count,
              std::string_view reason) {
        BehaviorRequest request;
        request.set_kind(kind);
        request.set_session_id(status_.session_id());
        request.set_goal_epoch(status_.goal_epoch());
        request.set_expected_recovery_count(count);
        request.set_reason(std::string(reason));
        request.set_steady_stamp_ns(now_ns_);
        return sender_ && sender_(request);
    }
    Sender sender_;
    NavigationStatus status_;
    std::int64_t now_ns_{0};
};

/** @brief Reject cached requests after cancellation, preemption or restart. */
inline bool MatchesBehaviorRequest(const BehaviorRequest& request,
                                   const NavigationStatus& status,
                                   std::int64_t now_ns) {
    if (!status.has_active_goal() || request.session_id().empty() ||
        request.session_id() != status.session_id() ||
        request.goal_epoch() != status.goal_epoch() ||
        request.steady_stamp_ns() <= 0 || now_ns < request.steady_stamp_ns() ||
        now_ns - request.steady_stamp_ns() > 500000000LL)
        return false;
    if (request.kind() == BehaviorRequest::RECOVER)
        return request.expected_recovery_count() == status.recovery_count() + 1;
    return request.kind() == BehaviorRequest::FAIL_GOAL &&
           request.expected_recovery_count() == status.recovery_count();
}
}  // namespace navigation::behavior
