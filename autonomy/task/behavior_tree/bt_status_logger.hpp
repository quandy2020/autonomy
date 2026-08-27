/*
 * Copyright 2026 The Openbot Authors
 *
 * Publishes BehaviorTreeLog status transitions for Autoviz Monitor mode.
 */

#pragma once

#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "behaviortree_cpp/loggers/abstract_logger.h"

namespace autonomy {
namespace task {

struct BtStatusEvent {
  std::string node_name;
  std::string registration_id;
  std::string previous_status;
  std::string current_status;
  int64_t timestamp_ns = 0;
};

using BtStatusLogCallback =
    std::function<void(const std::vector<BtStatusEvent>& events)>;

/** BT.CPP StatusChangeLogger that batches transitions for Autolink publish. */
class BtStatusLogger : public BT::StatusChangeLogger {
 public:
  explicit BtStatusLogger(BT::TreeNode* root_node);

  void setFlushCallback(BtStatusLogCallback callback);

  /** Emit buffered events (if any) via the flush callback. */
  void flush() override;

  void callback(BT::Duration timestamp, const BT::TreeNode& node,
                BT::NodeStatus prev_status,
                BT::NodeStatus status) override;

 private:
  static std::string ToStatusString(BT::NodeStatus status);

  std::mutex mutex_;
  std::vector<BtStatusEvent> pending_;
  BtStatusLogCallback flush_callback_;
};

}  // namespace task
}  // namespace autonomy
