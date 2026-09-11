/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/task_muxer.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {

bool TaskMuxer::TryAcquire(const proto::TaskType type, const std::string& cmd_id,
                           const std::string& client_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (estop_) {
        return false;
    }
    if (active_type_ != proto::TASK_TYPE_NONE && active_type_ != type) {
        return false;
    }
    active_type_ = type;
    cmd_id_ = cmd_id;
    client_id_ = client_id;
    return true;
}

void TaskMuxer::Release(const proto::TaskType type) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_type_ == type) {
        active_type_ = proto::TASK_TYPE_NONE;
        cmd_id_.clear();
        client_id_.clear();
    }
}

void TaskMuxer::Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    active_type_ = proto::TASK_TYPE_NONE;
    cmd_id_.clear();
    client_id_.clear();
}

bool TaskMuxer::CheckHasActive() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_type_ != proto::TASK_TYPE_NONE;
}

proto::ActiveTaskInfo TaskMuxer::GetSnapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    proto::ActiveTaskInfo info;
    info.set_type(active_type_);
    info.set_status(active_type_ == proto::TASK_TYPE_NONE
                        ? proto::TASK_STATUS_IDLE
                        : proto::TASK_STATUS_RUNNING);
    info.set_cmd_id(cmd_id_);
    info.set_client_id(client_id_);
    return info;
}

void TaskMuxer::SetEstop(const bool estop) {
    std::lock_guard<std::mutex> lock(mutex_);
    estop_ = estop;
    if (estop) {
        active_type_ = proto::TASK_TYPE_NONE;
        cmd_id_.clear();
        client_id_.clear();
    }
}

bool TaskMuxer::CheckEstopActive() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return estop_;
}

}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
