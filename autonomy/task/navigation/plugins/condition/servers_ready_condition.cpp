/*
 * Copyright 2026 The Openbot Authors
 *
 * NavTfAvailable: true when transform parent <- child is available.
 * NavServersReady: planning + control action servers are up.
 */

#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/navigation/plugins/plugin_utils.hpp"
#include "autonomy/transform/buffer.hpp"
#include <automsgs/msgs/builtin_interfaces/time.pb.h>

namespace autonomy::task::plugins::navigation {
namespace {

bool TransformAvailable(const std::string& parent_frame,
                        const std::string& child_frame)
{
    if (parent_frame.empty() || child_frame.empty()) {
        return false;
    }
    auto* buffer = ::autonomy::transform::Buffer::Instance();
    if (buffer == nullptr) {
        return false;
    }
    ::automsgs::msgs::builtin_interfaces::Time time;
    time.set_sec(0);
    time.set_nanosec(0);
    return buffer->canTransform(parent_frame, child_frame, time, 0.05f);
}

}  // namespace

class TfAvailableCondition : public BtCondition
{
public:
    TfAvailableCondition(const std::string& name, const BT::NodeConfig& config)
        : BtCondition(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("child", "base_link",
                                       "source / robot frame"),
            BT::InputPort<std::string>("parent", "map",
                                       "target / global frame"),
        };
    }

protected:
    BT::NodeStatus OnEvaluate() override
    {
        std::string child = "base_link";
        std::string parent = "map";
        getInput("child", child);
        getInput("parent", parent);
        return TransformAvailable(parent, child) ? BT::NodeStatus::SUCCESS
                                                 : BT::NodeStatus::FAILURE;
    }
};

class ServersReadyCondition : public BtCondition
{
public:
    ServersReadyCondition(const std::string& name, const BT::NodeConfig& config)
        : BtCondition(name, config) {}

protected:
    BT::NodeStatus OnEvaluate() override
    {
        auto client = ResolveClient(*this);
        return (client->IsPlanningReady() && client->IsControlReady())
                   ? BT::NodeStatus::SUCCESS
                   : BT::NodeStatus::FAILURE;
    }
};

}  // namespace autonomy::task::plugins::navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<
        autonomy::task::plugins::navigation::TfAvailableCondition>(
        "TransformValid");
    factory.registerNodeType<
        autonomy::task::plugins::navigation::ServersReadyCondition>(
        "ServersReady");
}
