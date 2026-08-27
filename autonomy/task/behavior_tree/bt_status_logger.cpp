/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/behavior_tree/bt_status_logger.hpp"

#include <chrono>

namespace autonomy {
namespace task {

BtStatusLogger::BtStatusLogger(BT::TreeNode* root_node)
    : BT::StatusChangeLogger(root_node) {}

void BtStatusLogger::setFlushCallback(BtStatusLogCallback callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  flush_callback_ = std::move(callback);
}

void BtStatusLogger::flush() {
  std::vector<BtStatusEvent> batch;
  BtStatusLogCallback callback;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_.empty() || !flush_callback_) {
      return;
    }
    batch.swap(pending_);
    callback = flush_callback_;
  }
  if (callback) {
    callback(batch);
  }
}

void BtStatusLogger::callback(BT::Duration timestamp, const BT::TreeNode& node,
                              BT::NodeStatus prev_status,
                              BT::NodeStatus status) {
  BtStatusEvent event;
  event.node_name = node.fullPath().empty() ? node.name() : node.fullPath();
  event.registration_id = node.registrationName();
  event.previous_status = ToStatusString(prev_status);
  event.current_status = ToStatusString(status);
  event.timestamp_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp).count();

  std::lock_guard<std::mutex> lock(mutex_);
  pending_.push_back(std::move(event));
}

std::string BtStatusLogger::ToStatusString(BT::NodeStatus status) {
  switch (status) {
    case BT::NodeStatus::RUNNING:
      return "RUNNING";
    case BT::NodeStatus::SUCCESS:
      return "SUCCESS";
    case BT::NodeStatus::FAILURE:
      return "FAILURE";
    case BT::NodeStatus::IDLE:
    default:
      return "IDLE";
  }
}

}  // namespace task
}  // namespace autonomy
