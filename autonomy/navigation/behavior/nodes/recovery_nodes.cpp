/* Copyright 2026 The Navigation Authors (navigation) */
#include "navigation/behavior/nodes/recovery_nodes.hpp"

namespace navigation::behavior {

BT::NodeStatus PipelineSequence::tick() {
    setStatus(BT::NodeStatus::RUNNING);
    for (std::size_t index = 0; index < childrenCount(); ++index) {
        const auto status = children_nodes_[index]->executeTick();
        if (status == BT::NodeStatus::FAILURE) {
            haltChildren();
            last_running_ = 0;
            return status;
        }
        if (status == BT::NodeStatus::RUNNING && index >= last_running_) {
            last_running_ = index;
            return status;
        }
        if (status != BT::NodeStatus::SUCCESS &&
            status != BT::NodeStatus::RUNNING)
            throw BT::RuntimeError(
                "PipelineSequence child returned unsupported status");
    }
    haltChildren();
    last_running_ = 0;
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList RecoveryNode::providedPorts() {
    return {BT::InputPort<unsigned>("number_of_retries", 3,
                                    "maximum recovery attempts")};
}

BT::NodeStatus RecoveryNode::tick() {
    if (childrenCount() != 2)
        throw BT::RuntimeError("RecoveryNode needs exactly two children");
    setStatus(BT::NodeStatus::RUNNING);
    unsigned maximum = 3;
    getInput("number_of_retries", maximum);
    NodeContext* context = nullptr;
    if (config().blackboard)
        (void)config().blackboard->get(keys::kContext, context);
    const auto status = children_nodes_[recovering_ ? 1 : 0]->executeTick();
    if (status == BT::NodeStatus::RUNNING)
        return status;
    if (!recovering_ && status == BT::NodeStatus::SUCCESS) {
        halt();
        return status;
    }
    if (recovering_) {
        haltChild(1);
        recovering_ = false;
        // Both a successful maneuver and a blocked maneuver re-evaluate the
        // main branch. A failed maneuver may try the opposite flank.
        return BT::NodeStatus::RUNNING;
    }
    const unsigned accepted = context && context->session
                                  ? context->session->RecoveryCount()
                                  : retries_;
    if (accepted >= maximum || retries_ >= maximum) {
        halt();
        return BT::NodeStatus::FAILURE;
    }
    haltChild(0);
    recovering_ = true;
    ++retries_;
    return BT::NodeStatus::RUNNING;
}

BT::PortsList WaitForGoalReachedNode::providedPorts() {
    return {BT::InputPort<double>("timeout_s", 600.0, "overall wait timeout")};
}
BT::NodeStatus WaitForGoalReachedNode::OnFirstTick(NodeContext& context) {
    started_ns_ = context.now_ns;
    return OnExecute(context);
}
BT::NodeStatus WaitForGoalReachedNode::OnExecute(NodeContext& context) {
    if (!context.session || !context.session->HasActiveGoal())
        return BT::NodeStatus::FAILURE;
    if (context.session->IsGoalReached())
        return BT::NodeStatus::SUCCESS;
    double timeout = 600.0;
    getInput("timeout_s", timeout);
    if (context.session->State() == lifecycle::MODULE_STATE_FAILED ||
        (timeout > 0.0 && context.now_ns - started_ns_ > timeout * 1e9))
        return BT::NodeStatus::FAILURE;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RecoveryExecutorNode::OnFirstTick(NodeContext& context) {
    if (!context.session || !context.session->HasActiveGoal())
        return BT::NodeStatus::FAILURE;
    requested_ns_ = last_send_ns_ = context.now_ns;
    if (context.session->RecoveryStatus() == RECOVERY_RUNNING) {
        expected_count_ = context.session->RecoveryCount();
    } else {
        expected_count_ = context.session->RecoveryCount() + 1;
        if (!context.session->RequestRecovery("no_progress"))
            return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}
BT::NodeStatus RecoveryExecutorNode::OnExecute(NodeContext& context) {
    if (!context.session || !context.session->HasActiveGoal())
        return BT::NodeStatus::FAILURE;
    if (context.session->RecoveryCount() < expected_count_) {
        if (context.now_ns - requested_ns_ > 2000000000LL)
            return BT::NodeStatus::FAILURE;
        // Re-send if discovery or an IPC sample was missed; accepted counts
        // make duplicate requests idempotent in the navigation process.
        if (context.now_ns - last_send_ns_ >= 200000000LL) {
            (void)context.session->RequestRecovery("no_progress");
            last_send_ns_ = context.now_ns;
        }
        return BT::NodeStatus::RUNNING;
    }
    if (context.session->RecoveryStatus() == RECOVERY_SUCCEEDED)
        return BT::NodeStatus::SUCCESS;
    if (context.session->RecoveryStatus() == RECOVERY_FAILED)
        return BT::NodeStatus::FAILURE;
    return BT::NodeStatus::RUNNING;
}
}  // namespace navigation::behavior

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<navigation::behavior::PipelineSequence>(
        "PipelineSequence");
    factory.registerNodeType<navigation::behavior::RecoveryNode>(
        "RecoveryNode");
    factory.registerNodeType<navigation::behavior::WaitForGoalReachedNode>(
        "WaitForGoalReached");
    factory.registerNodeType<navigation::behavior::RecoveryExecutorNode>(
        "RecoveryExecutor");
}
