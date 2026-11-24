/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file time_expired_condition.cpp
 * @brief Time expired condition.
 */

#include <chrono>

#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"

namespace autonomy::task::plugins::navigation {

class TimeExpiredCondition : public BtCondition
{
public:
    TimeExpiredCondition(const std::string& name, const BT::NodeConfig& config)
        : BtCondition(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<double>("seconds", 0.0, "timeout (s)")};
    }

protected:
    BT::NodeStatus OnEvaluate() override
    {
        // Parent halt() sets status IDLE; restart the timer on next entry.
        if (status() == BT::NodeStatus::IDLE) {
            started_ = false;
        }

        double seconds = 0.0;
        getInput("seconds", seconds);
        const auto now = std::chrono::steady_clock::now();
        if (!started_) {
            started_ = true;
            start_ = now;
        }
        const double elapsed =
            std::chrono::duration<double>(now - start_).count();
        return (elapsed >= seconds) ? BT::NodeStatus::SUCCESS
                                    : BT::NodeStatus::FAILURE;
    }

private:
    bool started_{false};
    std::chrono::steady_clock::time_point start_{};
};

}  // namespace autonomy::task::plugins::navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<
        autonomy::task::plugins::navigation::TimeExpiredCondition>(
        "TimeExpired");
}
