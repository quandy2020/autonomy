/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/common/logging.hpp"
#include "autonomy/task/apps/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/apps/exploration/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::exploration {

class SaveExplorationMapAction : public BtSyncAction
{
public:
    SaveExplorationMapAction(const std::string& name, const BT::NodeConfig& config)
        : BtSyncAction(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("map_name", "exploration map name"),
            BT::OutputPort<int>("error_code_id"),
            BT::OutputPort<std::string>("error_msg"),
        };
    }

protected:
    BT::NodeStatus OnExecute() override
    {
        auto client = ResolveClient(*this);
        if (!client) {
            SetErrorPorts(*this, 1, "SaveExplorationMap: missing exploration client");
            return BT::NodeStatus::FAILURE;
        }

        std::string map_name;
        getInput("map_name", map_name);
        if (map_name.empty()) {
            map_name = client->map_name();
        }
        if (map_name.empty()) {
            SetErrorPorts(*this, 2, "SaveExplorationMap: empty map_name");
            return BT::NodeStatus::FAILURE;
        }

        client->MarkFrontierVisited();
        AINFO << "SaveExplorationMap: checkpoint map '" << map_name
              << "' (progress=" << client->exploration_progress() << ")";
        ClearErrorPorts(*this);
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace autonomy::task::plugins::exploration

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<
        autonomy::task::plugins::exploration::SaveExplorationMapAction>(
        "SaveExplorationMap");
}
