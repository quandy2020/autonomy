/*
 * Copyright 2026 The Openbot Authors
 *
 * Autolink SimpleActionServer for ManipulationAction.
 */

#pragma once

#include <memory>
#include <mutex>

#include "autolink/action/simple_action_server.hpp"
#include "autolink/node/node.hpp"
#include "autonomy/manipulation/manipulation_server.hpp"

#include <automsgs/task/manipulation.pb.h>

namespace autonomy {
namespace manipulation {

/**
 * @brief Autolink SimpleActionServer adapter for ManipulationAction.
 *
 * Delegates plan / execute goals to ManipulationServer.
 */
class ManipulationActionServer {
 public:
  using ActionT = ::autonomy::task::proto::ManipulationAction;
  using ServerT = autolink::action::SimpleActionServer<ActionT>;

  /**
   * @brief Bind to a ManipulationServer (not owned).
   * @param[in] server Runtime server that handles planning and execution.
   */
  explicit ManipulationActionServer(ManipulationServer* server);

  /**
   * @brief Create and activate the Autolink action server on @p node.
   * @param[in] node Autolink node that owns the action server.
   * @param[in] action_name Action topic name (e.g. /autonomy/manipulation/move).
   * @return true on successful registration.
   */
  bool Init(const std::shared_ptr<autolink::Node>& node,
            const std::string& action_name);

  /** @brief Tear down the action server. */
  void Shutdown();

 private:
  void ExecuteCallback();

  ManipulationServer* server_ = nullptr;  // not owned
  std::shared_ptr<ServerT> action_server_;
  std::mutex mutex_;
};

}  // namespace manipulation
}  // namespace autonomy
