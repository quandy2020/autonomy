/*
 * Copyright 2026 The Openbot Authors
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
 * @file charging_client.hpp
 * @brief Facade used by charging BT nodes; owns DockingEngine.
 */

#pragma once

#include <memory>
#include <string>

#include "autolink/node/node.hpp"
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>
#include <automsgs/task/charging.pb.h>
#include "autonomy/task/charging/dock/docking_engine.hpp"
#include "autonomy/task/navigation/navigation_client.hpp"

namespace BT {
class Blackboard;
class TreeNode;
}  // namespace BT

namespace autonomy {
namespace task {
namespace charging {

constexpr char kChargingClientBlackboardKey[] = "charging_client";

/**
 * @class autonomy::task::charging::ChargingClient
 * @brief Applies ChargingGoal fields and ticks DockingEngine.
 */
class ChargingClient {
public:
    using Ptr = std::shared_ptr<ChargingClient>;

    static Ptr Create(navigation::NavigationClient::Ptr navigation,
                      std::shared_ptr<autolink::Node> node);
    static Ptr Create(std::shared_ptr<autolink::Node> node);
    static void SetShared(const Ptr& client);
    static Ptr FromBlackboard(const std::shared_ptr<BT::Blackboard>& blackboard);
    static Ptr FromNode(const BT::TreeNode& node);

    bool Configure(const ::autonomy::task::proto::DockingOptions& options);
    void ApplyGoal(const ::autonomy::task::proto::ChargingGoal& goal);

    const ::autonomy::task::proto::AutoRechargeOptions& auto_recharge() const {
        return auto_recharge_;
    }
    std::string SelectDockId() const {
        return engine_.SelectDockId(auto_recharge_.preferred_dock_id());
    }
    float battery_percent() const { return engine_.BatteryPercent(); }

    bool StartDock();
    bool StartUndock();
    DockingEngine::RunState Tick();
    void CancelActiveMotion();

    bool RunDockSearch();
    bool IsChargerVisible() const;
    bool MarkConnected();

    const std::string& dock_station_id() const { return dock_station_id_; }
    const std::string& dock_type() const { return dock_type_; }
    bool use_dock_id() const { return use_dock_id_; }
    bool navigate_to_staging() const { return navigate_to_staging_; }
    double max_staging_time() const { return max_staging_time_; }
    double max_undocking_time() const { return max_undocking_time_; }
    const automsgs::msgs::geometry_msgs::PoseStamped& dock_pose() const {
        return dock_pose_;
    }
    float battery_target_percent() const { return battery_target_percent_; }
    ::autonomy::task::proto::DockPhase phase() const { return engine_.phase(); }
    int num_retries() const { return engine_.num_retries(); }
    ::automsgs::msgs::status_msgs::StatusCode error_code() const {
        return engine_.error_code();
    }
    const std::string& error_msg() const { return engine_.error_msg(); }
    float docking_time_sec() const { return engine_.elapsed_sec(); }

    navigation::NavigationClient::Ptr navigation_client() const {
        return navigation_;
    }

    ChargingClient(navigation::NavigationClient::Ptr navigation,
                   std::shared_ptr<autolink::Node> node);

private:
    DockRequest MakeDockRequest() const;
    UndockRequest MakeUndockRequest() const;

    navigation::NavigationClient::Ptr navigation_;
    std::shared_ptr<autolink::Node> node_;
    DockingEngine engine_;
    ::autonomy::task::proto::AutoRechargeOptions auto_recharge_;
    double max_hold_sec_{14400.0};
    std::string dock_station_id_;
    automsgs::msgs::geometry_msgs::PoseStamped dock_pose_;
    std::string dock_type_;
    bool use_dock_id_{true};
    bool navigate_to_staging_{true};
    double max_search_radius_{3.0};
    double max_staging_time_{1000.0};
    double max_undocking_time_{30.0};
    int max_retry_count_{0};
    float battery_target_percent_{100.f};
    bool search_complete_{false};
    bool charger_visible_{false};
    bool dock_connected_{false};
};

}  // namespace charging
}  // namespace task
}  // namespace autonomy
