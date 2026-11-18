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
 * @file docking_engine.hpp
 * @brief Tick-based port of opennav_docking::DockingServer.
 */

#ifndef AUTONOMY_TASK_CHARGING_DOCK_DOCKING_ENGINE_HPP_
#define AUTONOMY_TASK_CHARGING_DOCK_DOCKING_ENGINE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>
#include <automsgs/task/charging.pb.h>
#include "autolink/node/node.hpp"
#include "autolink/node/writer.hpp"
#include "autonomy/task/charging/dock/controller.hpp"
#include "autonomy/task/charging/dock/dock_database.hpp"
#include "autonomy/task/charging/dock/navigator.hpp"
#include "autonomy/task/charging/dock/types.hpp"
#include "autonomy/task/navigation/navigation_client.hpp"

namespace autonomy {
namespace task {
namespace charging {

/**
 * @brief DockRobot request (OpenNav DockRobot.Goal fields).
 */
struct DockRequest {
    bool use_dock_id{true};
    std::string dock_id;
    std::string dock_type;
    ::automsgs::msgs::geometry_msgs::PoseStamped dock_pose;
    bool navigate_to_staging{true};
    double max_staging_time_sec{1000.0};
    int max_retries{-1};
    float hold_until_percent{0.f};
    double max_hold_sec{0.0};
};

/**
 * @brief UndockRobot request (OpenNav UndockRobot.Goal fields).
 */
struct UndockRequest {
    std::string dock_type;
    double max_undocking_time_sec{30.0};
};

/**
 * @class autonomy::task::charging::DockingEngine
 * @brief Stage → detect → spiral approach → wait for charge, with retries.
 *
 * BT plugins call Tick() at the task loop rate instead of blocking on ROS
 * actions. Retries match OpenNav: only approach / wait-for-charge failures
 * return to staging; staging nav and initial perception fail immediately.
 */
class DockingEngine {
public:
    enum class RunState { Idle, Running, Succeeded, Failed, Canceled };

    /**
     * @brief Load plugins, controller, navigator, and cmd_vel writer.
     * @param[in] node Autolink node.
     * @param[in] navigation Staging NavigationClient.
     * @param[in] options DockingOptions from charging.pb.txt.
     * @return true when the database and writers initialized.
     */
    bool Configure(const std::shared_ptr<autolink::Node>& node,
                   navigation::NavigationClient::Ptr navigation,
                   const ::autonomy::task::proto::DockingOptions& options);

    /**
     * @brief Activate dock plugins.
     */
    void Activate();

    /**
     * @brief Cancel motion and deactivate plugins.
     */
    void Deactivate();

    /**
     * @brief Start a dock cycle.
     * @param[in] request Dock id or pose goal.
     * @return true when the FSM entered Running.
     */
    bool StartDock(const DockRequest& request);

    /**
     * @brief Start an undock cycle from the last successful dock type.
     * @param[in] request Optional dock_type override and timeout.
     * @return true when the FSM entered Running.
     */
    bool StartUndock(const UndockRequest& request);

    /**
     * @brief Request cancellation; publishes zero velocity.
     */
    void Cancel();

    /**
     * @brief Advance one control period (rate-limited to controller_frequency).
     * @return Current run state.
     */
    RunState Tick();

    ::autonomy::task::proto::DockPhase phase() const { return phase_; }
    int num_retries() const { return num_retries_; }
    ::automsgs::msgs::status_msgs::StatusCode error_code() const {
        return error_code_;
    }
    const std::string& error_msg() const { return error_msg_; }
    float elapsed_sec() const;
    bool IsCharging() const;
    float BatteryPercent() const;
    const std::string& dock_id() const { return dock_id_; }

    /**
     * @brief Preferred dock, else nearest to the robot, else first instance.
     * @param[in] preferred Preferred dock id from config.
     * @return Dock map key or empty.
     */
    std::string SelectDockId(const std::string& preferred) const;

    /**
     * @brief Reload YAML dock instances.
     * @param[in] filepath YAML path with a `docks:` map.
     * @return true when parsed.
     */
    bool ReloadDatabase(const std::string& filepath);

private:
    void PublishZero();
    void PublishCmd(const ::automsgs::msgs::geometry_msgs::Twist& twist);

    /**
     * @brief One control iteration toward @p pose (OpenNav getCommandToPose).
     * @param[in] pose Target in a TF frame.
     * @param[in] linear_tol Linear arrival tolerance [m].
     * @param[in] angular_tol Angular arrival tolerance [rad].
     * @param[in] is_docking Passed to collision skipping.
     * @param[in] backward Reverse drive.
     * @param[out] reached Set true when within both tolerances.
     * @return true when a command was published or the pose was reached.
     */
    bool CommandToPose(const ::automsgs::msgs::geometry_msgs::PoseStamped& pose,
                       double linear_tol, double angular_tol, bool is_docking,
                       bool backward, bool* reached);
    void SetFailed(const std::exception& ex);
    Dock* ResolveDock(const DockRequest& request);
    void TickDock();
    void TickUndock();

    std::shared_ptr<autolink::Node> node_;
    navigation::NavigationClient::Ptr navigation_;
    ::autonomy::task::proto::DockingOptions options_;
    DockDatabase db_;
    DockController controller_;
    DockNavigator navigator_;
    std::shared_ptr<autolink::Writer<::automsgs::msgs::geometry_msgs::TwistStamped>>
        cmd_writer_;

    RunState state_{RunState::Idle};
    bool undocking_{false};
    ::autonomy::task::proto::DockPhase phase_{
        ::autonomy::task::proto::DOCK_PHASE_NONE};
    int num_retries_{0};
    int max_retries_{3};
    ::automsgs::msgs::status_msgs::StatusCode error_code_{
        ::automsgs::msgs::status_msgs::OK};
    std::string error_msg_;
    std::string dock_id_;
    std::string curr_dock_type_;
    std::string base_frame_{"base_link"};
    std::string fixed_frame_{"odom"};
    bool dock_backwards_{false};
    bool owns_temp_dock_{false};
    Dock temp_dock_;
    Dock* dock_{nullptr};
    ::automsgs::msgs::geometry_msgs::PoseStamped dock_pose_;
    ::automsgs::msgs::geometry_msgs::PoseStamped staging_pose_;
    std::chrono::steady_clock::time_point start_{};
    std::chrono::steady_clock::time_point phase_start_{};
    std::chrono::steady_clock::time_point last_cmd_{};
    std::chrono::milliseconds cmd_period_{20};
    double approach_timeout_{30.0};
    double perception_timeout_{5.0};
    double charge_timeout_{5.0};
    double prestaging_{0.5};
    double undock_lin_{0.05};
    double undock_ang_{0.05};
    double undock_timeout_{30.0};
    float hold_until_percent_{0.f};
    double max_hold_sec_{0.0};
};

}  // namespace charging
}  // namespace task
}  // namespace autonomy

#endif  // AUTONOMY_TASK_CHARGING_DOCK_DOCKING_ENGINE_HPP_
