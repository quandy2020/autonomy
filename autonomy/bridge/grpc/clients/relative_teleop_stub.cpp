/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/clients/relative_teleop_stub.hpp"

#include "autonomy/bridge/constants.hpp"
#include "autonomy/bridge/grpc/rpc_convert.hpp"
#include "autonomy/common/logging.hpp"
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {
namespace {

using StatusCode = ::automsgs::msgs::status_msgs::StatusCode;
namespace nav_actions = ::automsgs::actions;
namespace teleop_rpc = ::automsgs::rpcs::teleop;

}  // namespace

RelativeTeleopStub::RelativeTeleopStub(std::shared_ptr<autolink::Node> node,
                                       std::shared_ptr<TaskMuxer> muxer)
    : drive_client_(std::make_shared<DriveClient>(node, kDriveOnHeadingActionName)),
      backup_client_(std::make_shared<BackUpClient>(node, kBackUpActionName)),
      spin_client_(std::make_shared<SpinClient>(node, kSpinActionName)),
      muxer_(std::move(muxer)) {}

void RelativeTeleopStub::ClearLockedState() {
    mode_ = Mode::kNone;
    paused_ = false;
    state_ = teleop_rpc::TELEOP_STATE_IDLE;
    drive_handle_.reset();
    backup_handle_.reset();
    spin_handle_.reset();
    if (muxer_) {
        muxer_->Release(proto::TASK_TYPE_TELEOP);
    }
}

teleop_rpc::TeleopResponse RelativeTeleopStub::MakeResponse(
    const std::string& goal_id, const teleop_rpc::TeleopState state, const bool ok,
    const std::string& detail) const {
    teleop_rpc::TeleopResponse response;
    response.set_goal_id(goal_id);
    response.set_state(state);
    response.set_detail(detail);
    *response.mutable_status() =
        ok ? MakeOkStatus(detail)
           : MakeRpcStatus(StatusCode::TELEOP_BUSY, detail);
    return response;
}

teleop_rpc::TeleopResponse RelativeTeleopStub::GetSnapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return MakeResponse(goal_id_, state_, true);
}

bool RelativeTeleopStub::Cancel(const std::string&) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode_ == Mode::kDrive && drive_handle_) {
        drive_client_->AsyncCancelGoal(drive_handle_);
    } else if (mode_ == Mode::kBackUp && backup_handle_) {
        backup_client_->AsyncCancelGoal(backup_handle_);
    } else if (mode_ == Mode::kSpin && spin_handle_) {
        spin_client_->AsyncCancelGoal(spin_handle_);
    }
    state_ = teleop_rpc::TELEOP_STATE_CANCELLED;
    ClearLockedState();
    return true;
}

bool RelativeTeleopStub::Pause(const std::string&) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode_ == Mode::kNone) {
        return false;
    }
    paused_ = true;
    state_ = teleop_rpc::TELEOP_STATE_PAUSED;
    return true;
}

bool RelativeTeleopStub::Resume(const std::string&) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode_ == Mode::kNone || !paused_) {
        return false;
    }
    paused_ = false;
    state_ = teleop_rpc::TELEOP_STATE_ACTIVE;
    return true;
}

bool RelativeTeleopStub::DriveOnHeading(
    const teleop_rpc::DriveOnHeadingRequest& request, StreamCallback callback) {
    if (!callback) {
        return false;
    }
    if (muxer_ && muxer_->CheckEstopActive()) {
        callback(MakeResponse(request.goal_id(), teleop_rpc::TELEOP_STATE_REJECTED, false,
                              "emergency stop active"));
        return false;
    }
    if (muxer_ &&
        !muxer_->TryAcquire(proto::TASK_TYPE_TELEOP, request.goal_id(), "")) {
        callback(MakeResponse(request.goal_id(), teleop_rpc::TELEOP_STATE_REJECTED, false,
                              "another task is active"));
        return false;
    }
    if (!drive_client_->ActionServerIsReady()) {
        callback(MakeResponse(request.goal_id(), teleop_rpc::TELEOP_STATE_REJECTED, false,
                              "drive_on_heading server not ready"));
        if (muxer_) {
            muxer_->Release(proto::TASK_TYPE_TELEOP);
        }
        return false;
    }

    nav_actions::DriveOnHeadingAction::Goal goal;
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

    DriveClient::SendGoalOptions options;
    options.feedback_callback =
        [this, request, callback](
            std::shared_ptr<DriveClient::GoalHandle>,
            std::shared_ptr<const nav_actions::DriveOnHeadingAction::Feedback> feedback) {
            if (!feedback) {
                return;
            }
            std::lock_guard<std::mutex> lock(mutex_);
            if (paused_) {
                return;
            }
            auto response = MakeResponse(request.goal_id(), teleop_rpc::TELEOP_STATE_ACTIVE,
                                         true);
            response.set_remaining_distance_meters(
                std::max(0.f, request.distance_meters() - feedback->distance_traveled()));
            callback(response);
        };
    options.result_callback =
        [this, request, callback](
            const DriveClient::GoalHandle::WrappedResult& wrapped) {
            const bool ok =
                wrapped.code == autolink::action::ResultCode::SUCCEEDED;
            teleop_rpc::TeleopState state = teleop_rpc::TELEOP_STATE_FAILED;
            if (wrapped.code == autolink::action::ResultCode::SUCCEEDED) {
                state = teleop_rpc::TELEOP_STATE_SUCCEEDED;
            } else if (wrapped.code == autolink::action::ResultCode::CANCELED) {
                state = teleop_rpc::TELEOP_STATE_CANCELLED;
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                ClearLockedState();
            }
            callback(MakeResponse(request.goal_id(), state, ok));
        };

    const auto accepted = drive_client_->AsyncSendGoal(goal, options);
    if (accepted.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
        callback(MakeResponse(request.goal_id(), teleop_rpc::TELEOP_STATE_REJECTED, false,
                              "drive goal accept timeout"));
        if (muxer_) {
            muxer_->Release(proto::TASK_TYPE_TELEOP);
        }
        return false;
    }
    auto handle = accepted.get();
    if (!handle) {
        callback(MakeResponse(request.goal_id(), teleop_rpc::TELEOP_STATE_REJECTED, false,
                              "drive goal rejected"));
        if (muxer_) {
            muxer_->Release(proto::TASK_TYPE_TELEOP);
        }
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        mode_ = Mode::kDrive;
        goal_id_ = request.goal_id();
        state_ = teleop_rpc::TELEOP_STATE_ACTIVE;
        drive_handle_ = handle;
    }
    callback(MakeResponse(request.goal_id(), teleop_rpc::TELEOP_STATE_ACTIVE, true));
    return true;
}

bool RelativeTeleopStub::BackUp(const teleop_rpc::BackUpRequest& request,
                                StreamCallback callback) {
    if (!callback) {
        return false;
    }
    if (request.distance_meters() <= 0.f) {
        callback(MakeResponse(request.goal_id(), teleop_rpc::TELEOP_STATE_REJECTED, false,
                              "distance_meters must be > 0"));
        return false;
    }
    if (muxer_ && muxer_->CheckEstopActive()) {
        callback(MakeResponse(request.goal_id(), teleop_rpc::TELEOP_STATE_REJECTED, false,
                              "emergency stop active"));
        return false;
    }
    if (muxer_ &&
        !muxer_->TryAcquire(proto::TASK_TYPE_TELEOP, request.goal_id(), "")) {
        callback(MakeResponse(request.goal_id(), teleop_rpc::TELEOP_STATE_REJECTED, false,
                              "another task is active"));
        return false;
    }
    if (!backup_client_->ActionServerIsReady()) {
        callback(MakeResponse(request.goal_id(), teleop_rpc::TELEOP_STATE_REJECTED, false,
                              "backup server not ready"));
        if (muxer_) {
            muxer_->Release(proto::TASK_TYPE_TELEOP);
        }
        return false;
    }

    nav_actions::BackUpAction::Goal goal;
    goal.mutable_target()->set_x(-request.distance_meters());
    float speed = 0.1f;
    if (request.has_options() &&
        request.options().has_maximum_linear_speed()) {
        speed = request.options().maximum_linear_speed();
    }
    goal.set_speed(speed);

    BackUpClient::SendGoalOptions options;
    options.feedback_callback =
        [this, request, callback](
            std::shared_ptr<BackUpClient::GoalHandle>,
            std::shared_ptr<const nav_actions::BackUpAction::Feedback> feedback) {
            if (!feedback) {
                return;
            }
            std::lock_guard<std::mutex> lock(mutex_);
            if (paused_) {
                return;
            }
            auto response = MakeResponse(request.goal_id(), teleop_rpc::TELEOP_STATE_ACTIVE,
                                         true);
            response.set_remaining_distance_meters(
                std::max(0.f, request.distance_meters() - feedback->distance_traveled()));
            callback(response);
        };
    options.result_callback =
        [this, request, callback](
            const BackUpClient::GoalHandle::WrappedResult& wrapped) {
            const bool ok =
                wrapped.code == autolink::action::ResultCode::SUCCEEDED;
            teleop_rpc::TeleopState state = ok ? teleop_rpc::TELEOP_STATE_SUCCEEDED
                                       : (wrapped.code == autolink::action::ResultCode::CANCELED
                                              ? teleop_rpc::TELEOP_STATE_CANCELLED
                                              : teleop_rpc::TELEOP_STATE_FAILED);
            {
                std::lock_guard<std::mutex> lock(mutex_);
                ClearLockedState();
            }
            callback(MakeResponse(request.goal_id(), state, ok));
        };

    const auto accepted = backup_client_->AsyncSendGoal(goal, options);
    if (accepted.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
        callback(MakeResponse(request.goal_id(), teleop_rpc::TELEOP_STATE_REJECTED, false,
                              "backup goal accept timeout"));
        if (muxer_) {
            muxer_->Release(proto::TASK_TYPE_TELEOP);
        }
        return false;
    }
    auto handle = accepted.get();
    if (!handle) {
        callback(MakeResponse(request.goal_id(), teleop_rpc::TELEOP_STATE_REJECTED, false,
                              "backup goal rejected"));
        if (muxer_) {
            muxer_->Release(proto::TASK_TYPE_TELEOP);
        }
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        mode_ = Mode::kBackUp;
        goal_id_ = request.goal_id();
        state_ = teleop_rpc::TELEOP_STATE_ACTIVE;
        backup_handle_ = handle;
    }
    callback(MakeResponse(request.goal_id(), teleop_rpc::TELEOP_STATE_ACTIVE, true));
    return true;
}

bool RelativeTeleopStub::Spin(const teleop_rpc::SpinRequest& request,
                              StreamCallback callback) {
    if (!callback) {
        return false;
    }
    if (muxer_ && muxer_->CheckEstopActive()) {
        callback(MakeResponse(request.goal_id(), teleop_rpc::TELEOP_STATE_REJECTED, false,
                              "emergency stop active"));
        return false;
    }
    if (muxer_ &&
        !muxer_->TryAcquire(proto::TASK_TYPE_TELEOP, request.goal_id(), "")) {
        callback(MakeResponse(request.goal_id(), teleop_rpc::TELEOP_STATE_REJECTED, false,
                              "another task is active"));
        return false;
    }
    if (!spin_client_->ActionServerIsReady()) {
        callback(MakeResponse(request.goal_id(), teleop_rpc::TELEOP_STATE_REJECTED, false,
                              "spin server not ready"));
        if (muxer_) {
            muxer_->Release(proto::TASK_TYPE_TELEOP);
        }
        return false;
    }

    nav_actions::SpinAction::Goal goal;
    goal.set_target_yaw(request.target_yaw_radians());
    if (request.has_options() && request.options().has_timeout_seconds()) {
        goal.mutable_time_allowance()->set_sec(
            static_cast<int32_t>(request.options().timeout_seconds()));
    }

    SpinClient::SendGoalOptions options;
    options.feedback_callback =
        [this, request, callback](
            std::shared_ptr<SpinClient::GoalHandle>,
            std::shared_ptr<const nav_actions::SpinAction::Feedback> feedback) {
            if (!feedback) {
                return;
            }
            std::lock_guard<std::mutex> lock(mutex_);
            if (paused_) {
                return;
            }
            auto response = MakeResponse(request.goal_id(), teleop_rpc::TELEOP_STATE_ACTIVE,
                                         true);
            response.set_remaining_yaw_radians(
                request.target_yaw_radians() - feedback->angular_distance_traveled());
            callback(response);
        };
    options.result_callback =
        [this, request, callback](
            const SpinClient::GoalHandle::WrappedResult& wrapped) {
            const bool ok =
                wrapped.code == autolink::action::ResultCode::SUCCEEDED;
            teleop_rpc::TeleopState state = ok ? teleop_rpc::TELEOP_STATE_SUCCEEDED
                                       : (wrapped.code == autolink::action::ResultCode::CANCELED
                                              ? teleop_rpc::TELEOP_STATE_CANCELLED
                                              : teleop_rpc::TELEOP_STATE_FAILED);
            {
                std::lock_guard<std::mutex> lock(mutex_);
                ClearLockedState();
            }
            callback(MakeResponse(request.goal_id(), state, ok));
        };

    const auto accepted = spin_client_->AsyncSendGoal(goal, options);
    if (accepted.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
        callback(MakeResponse(request.goal_id(), teleop_rpc::TELEOP_STATE_REJECTED, false,
                              "spin goal accept timeout"));
        if (muxer_) {
            muxer_->Release(proto::TASK_TYPE_TELEOP);
        }
        return false;
    }
    auto handle = accepted.get();
    if (!handle) {
        callback(MakeResponse(request.goal_id(), teleop_rpc::TELEOP_STATE_REJECTED, false,
                              "spin goal rejected"));
        if (muxer_) {
            muxer_->Release(proto::TASK_TYPE_TELEOP);
        }
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        mode_ = Mode::kSpin;
        goal_id_ = request.goal_id();
        state_ = teleop_rpc::TELEOP_STATE_ACTIVE;
        spin_handle_ = handle;
    }
    callback(MakeResponse(request.goal_id(), teleop_rpc::TELEOP_STATE_ACTIVE, true));
    return true;
}

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
