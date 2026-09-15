/*
 * Copyright 2026 The Openbot Authors
 */

#include <cmath>
#include <cstdint>

#include "autonomy/common/logging.hpp"
#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/navigation/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::navigation {
namespace {

uint64_t PathFingerprint(const automsgs::msgs::nav_msgs::Path& path)
{
    const int n = path.poses_size();
    if (n <= 0) {
        return 0;
    }
    const auto& a = path.poses(0).pose().position();
    const auto& b = path.poses(n - 1).pose().position();
    // Coarse identity: size + endpoints (mm). Enough to skip duplicate smooth.
    const auto q = [](double v) {
        return static_cast<uint64_t>(std::llround(v * 1000.0)) & 0xfffffu;
    };
    return (static_cast<uint64_t>(n) << 40) ^ (q(a.x()) << 20) ^ q(a.y()) ^
           (q(b.x()) << 30) ^ (q(b.y()) << 10);
}

}  // namespace

class SmoothPathAction : public BtSyncAction
{
public:
    SmoothPathAction(const std::string& name, const BT::NodeConfig& config)
        : BtSyncAction(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<automsgs::msgs::nav_msgs::Path>("unsmoothed_path"),
            BT::OutputPort<automsgs::msgs::nav_msgs::Path>("smoothed_path"),
            BT::InputPort<std::string>("smoother_id", "simple_smoother",
                                       "plugin id"),
            BT::InputPort<double>("max_smoothing_duration", 1.0,
                                  "smoother wall-time budget (s)"),
            BT::InputPort<bool>("check_for_collisions", false,
                                "reject smoothed path on collision"),
            BT::OutputPort<double>("smoothing_duration"),
            BT::OutputPort<bool>("was_completed"),
            BT::OutputPort<int>("error_code_id"),
            BT::OutputPort<std::string>("error_msg"),
        };
    }

protected:
    BT::NodeStatus OnExecute() override
    {
        auto client = ResolveClient(*this);
        automsgs::msgs::nav_msgs::Path unsmoothed;
        std::string smoother_id;
        double max_smoothing_duration = 1.0;
        bool check_for_collisions = false;
        if (!getInput("unsmoothed_path", unsmoothed)) {
            SetErrorPorts(*this, 1, "SmoothPath: missing unsmoothed_path");
            return BT::NodeStatus::FAILURE;
        }
        getInput("smoother_id", smoother_id);
        getInput("max_smoothing_duration", max_smoothing_duration);
        getInput("check_for_collisions", check_for_collisions);

        const uint64_t fingerprint = PathFingerprint(unsmoothed);
        if (fingerprint != 0 && fingerprint == last_fingerprint_) {
            setOutput("smoothed_path", unsmoothed);
            setOutput("smoothing_duration", 0.0);
            setOutput("was_completed", true);
            ClearErrorPorts(*this);
            return BT::NodeStatus::SUCCESS;
        }

        automsgs::msgs::nav_msgs::Path smoothed;
        int error_code = 0;
        std::string error_message;
        if (!client->SmoothPath(unsmoothed, smoother_id, max_smoothing_duration,
                                check_for_collisions, smoothed, &error_code,
                                &error_message)) {
            // Optional stage: keep navigating on the planned path if smoothing
            // fails (timeout / abort / empty). Avoids BT recovery thrashing.
            AWARN_EVERY(20) << "SmoothPath failed (" << error_message
                            << "); using unsmoothed path ("
                            << unsmoothed.poses_size() << " poses)";
            setOutput("smoothed_path", unsmoothed);
            setOutput("smoothing_duration", 0.0);
            setOutput("was_completed", false);
            last_fingerprint_ = fingerprint;
            SetErrorPorts(*this, error_code, error_message);
            return BT::NodeStatus::SUCCESS;
        }

        setOutput("smoothed_path", smoothed);
        setOutput("smoothing_duration", 0.0);
        setOutput("was_completed", true);
        last_fingerprint_ = fingerprint;
        ClearErrorPorts(*this);
        return BT::NodeStatus::SUCCESS;
    }

private:
    uint64_t last_fingerprint_{0};
};

}  // namespace autonomy::task::plugins::navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<autonomy::task::plugins::navigation::SmoothPathAction>(
        "SmoothPath");
}
