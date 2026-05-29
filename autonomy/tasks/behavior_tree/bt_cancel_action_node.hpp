/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#ifndef AUTONOMY_TASKS_BEHAVIOR_TREE_BT_CANCEL_ACTION_NODE_HPP_
#define AUTONOMY_TASKS_BEHAVIOR_TREE_BT_CANCEL_ACTION_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "autolink/action/create_client.hpp"
#include "autolink/node/node.hpp"
#include "autolink/state.hpp"
#include "autonomy/tasks/behavior_tree/bt_context.hpp"
#include "behaviortree_cpp/action_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

/**
 * @brief BT node that cancels all goals on an autolink action server
 *        (nav2 BtCancelActionNode).
 *
 * @tparam ActionT Action traits type (Goal / Result from action_type.hpp).
 */
template <typename ActionT>
class BtCancelActionNode : public BT::ActionNodeBase
{
public:
    using ActionClient = autolink::action::Client<ActionT>;

    /**
     * @param xml_tag_name BT XML tag for this node.
     * @param action_name Default autolink action server name.
     * @param conf Node configuration and blackboard.
     */
    BtCancelActionNode(const std::string& xml_tag_name,
                       const std::string& action_name,
                       const BT::NodeConfiguration& conf);

    BtCancelActionNode() = delete;

    /** Standard ports plus optional extra input ports. */
    static BT::PortsList ProvidedBasicPorts(BT::PortsList addition);

    static BT::PortsList providedPorts();

    void halt() override;

    BT::NodeStatus tick() override;

protected:
    std::string action_name_;
    mutable std::shared_ptr<autolink::Node> node_;
    mutable std::shared_ptr<ActionClient> action_client_;

    std::chrono::milliseconds server_timeout_{20000};
    std::chrono::milliseconds wait_for_service_timeout_{1000};

private:
    std::shared_ptr<autolink::Node> GetNode() const;

    std::shared_ptr<ActionClient> GetClient();
};

template <typename ActionT>
BtCancelActionNode<ActionT>::BtCancelActionNode(
    const std::string& xml_tag_name, const std::string& action_name,
    const BT::NodeConfiguration& conf)
    : BT::ActionNodeBase(xml_tag_name, conf), action_name_(action_name) {
    getInput("server_timeout", server_timeout_);
    if (config().blackboard) {
        config().blackboard->get("wait_for_service_timeout",
                                 wait_for_service_timeout_);
    }
    std::string remapped;
    if (getInput("server_name", remapped) && !remapped.empty()) {
        action_name_ = remapped;
    }
}

template <typename ActionT>
BT::PortsList BtCancelActionNode<ActionT>::ProvidedBasicPorts(
    BT::PortsList addition) {
    BT::PortsList basic = {
        BT::InputPort<std::string>("server_name", "Action server name"),
        BT::InputPort<std::chrono::milliseconds>("server_timeout"),
    };
    basic.insert(addition.begin(), addition.end());
    return basic;
}

template <typename ActionT>
BT::PortsList BtCancelActionNode<ActionT>::providedPorts() {
    return ProvidedBasicPorts({});
}

template <typename ActionT>
void BtCancelActionNode<ActionT>::halt() {}

template <typename ActionT>
BT::NodeStatus BtCancelActionNode<ActionT>::tick() {
    setStatus(BT::NodeStatus::RUNNING);
    auto client = GetClient();
    if (!client) {
        return BT::NodeStatus::FAILURE;
    }
    auto future = client->AsyncCancelAllGoals();
    if (future.wait_for(server_timeout_) != std::future_status::ready ||
        !future.get()) {
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
}

template <typename ActionT>
std::shared_ptr<autolink::Node> BtCancelActionNode<ActionT>::GetNode() const {
    if (node_) {
        return node_;
    }
    if (!config().blackboard) {
        return nullptr;
    }
    config().blackboard->get(kBlackboardAutolinkNodeKey, node_);
    return node_;
}

template <typename ActionT>
std::shared_ptr<typename BtCancelActionNode<ActionT>::ActionClient>
BtCancelActionNode<ActionT>::GetClient() {
    if (action_client_) {
        return action_client_;
    }
    auto node = GetNode();
    if (!node) {
        return nullptr;
    }
    action_client_ =
        autolink::action::CreateClient<ActionT>(node, action_name_);
    const auto deadline =
        std::chrono::steady_clock::now() + wait_for_service_timeout_;
    while (autolink::OK() && !action_client_->ActionServerIsReady()) {
        if (std::chrono::steady_clock::now() > deadline) {
            return nullptr;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return action_client_;
}

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#endif  // AUTONOMY_TASKS_BEHAVIOR_TREE_BT_CANCEL_ACTION_NODE_HPP_
