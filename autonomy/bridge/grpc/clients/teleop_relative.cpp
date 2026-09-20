/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/clients/teleop_relative.hpp"

#include <algorithm>

#include "autonomy/bridge/constants.hpp"
#include "autonomy/bridge/grpc/task_types.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {
namespace teleop {
namespace {

namespace teleop_rpc = ::automsgs::rpcs::teleop;

}  // namespace

DriveOnHeadingTraits::Goal DriveOnHeadingTraits::ConvertToGoal(
    const Request& request) const {
    Goal goal;
    goal.mutable_target()->set_x(request.distance_meters());
    goal.mutable_target()->set_y(0.f);
    goal.mutable_target()->set_z(0.f);
    float speed = 0.2f;
    if (request.has_options() &&
        request.options().has_maximum_linear_speed()) {
        speed = request.options().maximum_linear_speed();
    }
    goal.set_speed(speed);
    if (request.has_options() && request.options().has_timeout_seconds()) {
        goal.mutable_time_allowance()->set_sec(
            static_cast<int32_t>(request.options().timeout_seconds()));
    }
    if (request.has_options()) {
        goal.set_disable_collision_checks(
            request.options().disable_collision_checks());
    }
    return goal;
}

DriveOnHeadingTraits::Response DriveOnHeadingTraits::MakeFeedback(
    const Request& request, const Feedback& feedback) const {
    auto response = detail::MakeTeleopResponse(
        request.goal_id(), detail::teleop_rpc::TELEOP_STATE_ACTIVE, true);
    response.set_remaining_distance_meters(std::max(
        0.f, request.distance_meters() - feedback.distance_traveled()));
    return response;
}

BackUpTraits::Goal BackUpTraits::ConvertToGoal(const Request& request) const {
    Goal goal;
    goal.mutable_target()->set_x(-request.distance_meters());
    float speed = 0.1f;
    if (request.has_options() &&
        request.options().has_maximum_linear_speed()) {
        speed = request.options().maximum_linear_speed();
    }
    goal.set_speed(speed);
    return goal;
}

BackUpTraits::Response BackUpTraits::MakeFeedback(
    const Request& request, const Feedback& feedback) const {
    auto response = detail::MakeTeleopResponse(
        request.goal_id(), detail::teleop_rpc::TELEOP_STATE_ACTIVE, true);
    response.set_remaining_distance_meters(std::max(
        0.f, request.distance_meters() - feedback.distance_traveled()));
    return response;
}

SpinTraits::Goal SpinTraits::ConvertToGoal(const Request& request) const {
    Goal goal;
    goal.set_target_yaw(request.target_yaw_radians());
    if (request.has_options() && request.options().has_timeout_seconds()) {
        goal.mutable_time_allowance()->set_sec(
            static_cast<int32_t>(request.options().timeout_seconds()));
    }
    return goal;
}

SpinTraits::Response SpinTraits::MakeFeedback(const Request& request,
                                              const Feedback& feedback) const {
    auto response = detail::MakeTeleopResponse(
        request.goal_id(), detail::teleop_rpc::TELEOP_STATE_ACTIVE, true);
    response.set_remaining_yaw_radians(request.target_yaw_radians() -
                                       feedback.angular_distance_traveled());
    return response;
}

TeleopRelativeBackend::TeleopRelativeBackend(
    std::shared_ptr<autolink::Node> node, TaskMuxer::SharedPtr muxer,
    WorkScheduler* scheduler, CommandIdempotencyCache* idempotency)
    : drive_client_(DriveOnHeadingTraits::Client::make_shared(
          node, kDriveOnHeadingActionName)),
      backup_client_(
          BackUpTraits::Client::make_shared(node, kBackUpActionName)),
      spin_client_(
          SpinTraits::Client::make_shared(node, kSpinActionName)),
      muxer_(std::move(muxer)),
      session_(scheduler, muxer_, TASK_TYPE_TELEOP, idempotency) {}

bool TeleopRelativeBackend::IsBusy() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return mode_ != Mode::kNone;
}

void TeleopRelativeBackend::ClearLocked() {
    mode_ = Mode::kNone;
    paused_ = false;
    state_ = teleop_rpc::TELEOP_STATE_IDLE;
    drive_handle_.reset();
    backup_handle_.reset();
    spin_handle_.reset();
    if (muxer_) {
        muxer_->Release(TASK_TYPE_TELEOP);
    }
}

teleop_rpc::TeleopResponse TeleopRelativeBackend::MakeResponse(
    const std::string& goal_id, const teleop_rpc::TeleopState state,
    const bool ok, const std::string& detail) const {
    return detail::MakeTeleopResponse(goal_id, state, ok, detail);
}

teleop_rpc::TeleopResponse TeleopRelativeBackend::GetSnapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return MakeResponse(goal_id_, state_, true);
}

bool TeleopRelativeBackend::CancelGoal(const std::string&) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode_ == Mode::kDrive && drive_handle_) {
        drive_client_->CancelGoal(drive_handle_);
    }
    if (mode_ == Mode::kBackUp && backup_handle_) {
        backup_client_->CancelGoal(backup_handle_);
    }
    if (mode_ == Mode::kSpin && spin_handle_) {
        spin_client_->CancelGoal(spin_handle_);
    }
    state_ = teleop_rpc::TELEOP_STATE_CANCELLED;
    ClearLocked();
    return true;
}

bool TeleopRelativeBackend::PauseGoal(const std::string&) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode_ == Mode::kNone) {
        return false;
    }
    paused_ = true;
    state_ = teleop_rpc::TELEOP_STATE_PAUSED;
    return true;
}

bool TeleopRelativeBackend::ResumeGoal(const std::string&) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode_ == Mode::kNone || !paused_) {
        return false;
    }
    paused_ = false;
    state_ = teleop_rpc::TELEOP_STATE_ACTIVE;
    return true;
}

template <typename Traits>
bool TeleopRelativeBackend::StartRelativeAction(
    typename Traits::Client& client, const typename Traits::Request& request,
    StreamCallback callback, Mode mode,
    std::shared_ptr<typename Traits::Client::GoalHandle>* handle_slot) {
    ActionBackgroundHooks<Traits> hooks;
    hooks.sync_reject = [this]() -> std::optional<std::string> {
        std::lock_guard<std::mutex> lock(mutex_);
        if (mode_ != Mode::kNone) {
            return std::string("another relative teleop goal is active");
        }
        if (velocity_busy_ && velocity_busy_()) {
            return std::string("velocity teleop session is active");
        }
        return std::nullopt;
    };
    hooks.skip_feedback = [this]() {
        std::lock_guard<std::mutex> lock(mutex_);
        return paused_;
    };
    hooks.on_accepted =
        [this, mode, handle_slot, goal_id = Traits{}.CmdId(request)](
            std::shared_ptr<void> erased) {
            auto handle =
                std::static_pointer_cast<typename Traits::Client::GoalHandle>(
                    std::move(erased));
            std::lock_guard<std::mutex> lock(mutex_);
            mode_ = mode;
            goal_id_ = goal_id;
            state_ = teleop_rpc::TELEOP_STATE_ACTIVE;
            *handle_slot = std::move(handle);
        };
    hooks.on_result_clear = [this]() {
        std::lock_guard<std::mutex> lock(mutex_);
        ClearLocked();
    };
    return RunAction<Traits>(session_, client, request, std::move(callback),
                             std::move(hooks));
}

bool TeleopRelativeBackend::HandleDriveOnHeading(
    const teleop_rpc::DriveOnHeadingRequest& request, StreamCallback callback) {
    return StartRelativeAction<DriveOnHeadingTraits>(
        *drive_client_, request, std::move(callback), Mode::kDrive,
        &drive_handle_);
}

bool TeleopRelativeBackend::HandleBackUp(
    const teleop_rpc::BackUpRequest& request, StreamCallback callback) {
    if (request.distance_meters() <= 0.f) {
        if (callback) {
            callback(MakeResponse(request.goal_id(),
                                  teleop_rpc::TELEOP_STATE_REJECTED, false,
                                  "distance_meters must be > 0"));
        }
        return false;
    }
    return StartRelativeAction<BackUpTraits>(*backup_client_, request,
                                             std::move(callback), Mode::kBackUp,
                                             &backup_handle_);
}

bool TeleopRelativeBackend::HandleSpin(const teleop_rpc::SpinRequest& request,
                                       StreamCallback callback) {
    return StartRelativeAction<SpinTraits>(*spin_client_, request,
                                           std::move(callback), Mode::kSpin,
                                           &spin_handle_);
}

}  // namespace teleop
}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
