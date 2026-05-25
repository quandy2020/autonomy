/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <glog/logging.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <thread>

#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/common/gflags.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/common/math/math_utils.hpp"
#include "autonomy/common/version.hpp"
#include "autonomy/tasks/behavior_tree/bt_engine.hpp"
#include "autonomy/tasks/proto/bt_action.pb.h"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/planning/planner_server.hpp"
#include "autonomy/control/utils/odometry_utils.hpp"
#include "autonomy/tasks/task.hpp"
#include "autonomy/system/options.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/geometry_msgs/transform_stamped.h"

namespace autonomy {
namespace tasks {
namespace {

std::atomic<bool> g_shutdown{false};

// Spawn in map frame (see config/common.lua demo_robot_spawn_*).
constexpr double kMockRobotSpawnX = 1.0;
constexpr double kMockRobotSpawnY = 1.0;
constexpr double kMockRobotSpawnYaw = 0.0;
constexpr const char* kOdomFrame = "odom";

struct MockRobotState {
    double x{kMockRobotSpawnX};
    double y{kMockRobotSpawnY};
    double yaw{kMockRobotSpawnYaw};
    std::mutex mutex;
};

void SigintHandler(int /*sig*/) {
    g_shutdown.store(true);
}

std::shared_ptr<proto::NavigateToPoseAction::Goal>
MakeNavigateGoal(double x, double y, double yaw, const std::string& frame) {
    auto goal =
        std::make_shared<proto::NavigateToPoseAction::Goal>();
    auto* pose = goal->mutable_pose();
    pose->mutable_header()->set_frame_id(frame);
    pose->mutable_pose()->mutable_position()->set_x(x);
    pose->mutable_pose()->mutable_position()->set_y(y);
    pose->mutable_pose()->mutable_position()->set_z(0.0);
    const double half_yaw = yaw * 0.5;
    pose->mutable_pose()->mutable_orientation()->set_x(0.0);
    pose->mutable_pose()->mutable_orientation()->set_y(0.0);
    pose->mutable_pose()->mutable_orientation()->set_z(std::sin(half_yaw));
    pose->mutable_pose()->mutable_orientation()->set_w(std::cos(half_yaw));
    return goal;
}

void PublishMockTransform(const std::string& parent_frame,
                          const std::string& child_frame, double x, double y,
                          double yaw, bool is_static) {
    geometry_msgs::TransformStamped tf;
    const auto now = commsgs::builtin_interfaces::Time::Now();
    tf.header.stamp = static_cast<uint64_t>(now.sec) * 1'000'000'000ULL +
                        now.nanosec;
    tf.header.frame_id = parent_frame;
    tf.child_frame_id = child_frame;
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    const double half_yaw = yaw * 0.5;
    tf.transform.rotation.z = std::sin(half_yaw);
    tf.transform.rotation.w = std::cos(half_yaw);

    auto* buffer = autonomy::transform::Buffer::Instance();
    if (!buffer->setTransform(tf, "autonomy_tasks_main", is_static)) {
        AERROR << "Failed to publish mock TF " << parent_frame << " -> "
               << child_frame;
    }
}

void PublishMockStaticTransform(const std::string& parent_frame,
                                const std::string& child_frame, double x,
                                double y, double yaw) {
    PublishMockTransform(parent_frame, child_frame, x, y, yaw, true);
    AINFO << "Mock static TF " << parent_frame << " -> " << child_frame << " ("
          << x << ", " << y << ", yaw=" << yaw << ")";
}

void IntegrateDiffDrive(const commsgs::geometry_msgs::Twist& cmd, double dt,
                        double& x, double& y, double& yaw) {
    const double v = cmd.linear.x;
    const double w = cmd.angular.z;
    x += v * std::cos(yaw) * dt;
    y += v * std::sin(yaw) * dt;
    yaw = ::autonomy::common::math::NormalizeAngle(yaw + w * dt);
}

void PublishMockOdometry(
    const std::shared_ptr<Task>& task,
    const std::string& robot_frame, double x, double y, double yaw,
    const commsgs::geometry_msgs::Twist& twist) {
    const auto ctx = task->task_context();
    if (!ctx || !ctx->odom_smoother) {
        return;
    }
    commsgs::planning_msgs::Odometry odom;
    odom.header.stamp = commsgs::builtin_interfaces::Time::Now();
    odom.header.frame_id = kOdomFrame;
    odom.child_frame_id = robot_frame;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    const double half_yaw = yaw * 0.5;
    odom.pose.pose.orientation.z = std::sin(half_yaw);
    odom.pose.pose.orientation.w = std::cos(half_yaw);
    odom.twist.twist = twist;
    ctx->odom_smoother->UpdateOdometry(odom);
}

void RunMockSimulation(
    const std::shared_ptr<Task>& task,
    const std::string& robot_frame, MockRobotState* state,
    std::atomic<bool>* running) {
    auto last_tick = std::chrono::steady_clock::now();
    while (running->load()) {
        const auto now = std::chrono::steady_clock::now();
        const double dt = std::min(
            std::chrono::duration<double>(now - last_tick).count(), 0.1);
        last_tick = now;

        commsgs::geometry_msgs::Twist cmd{};
        if (const auto ctx = task->task_context()) {
            if (ctx->controller) {
                cmd = ctx->controller->GetLastCmdVel().twist;
            }
        }

        double x = kMockRobotSpawnX;
        double y = kMockRobotSpawnY;
        double yaw = kMockRobotSpawnYaw;
        {
            std::lock_guard<std::mutex> lock(state->mutex);
            IntegrateDiffDrive(cmd, dt, state->x, state->y, state->yaw);
            x = state->x;
            y = state->y;
            yaw = state->yaw;
        }

        PublishMockTransform(kOdomFrame, robot_frame, x, y, yaw, false);
        PublishMockOdometry(task, robot_frame, x, y, yaw, cmd);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void SetupMockTfTree(const std::string& global_frame,
                     const std::string& robot_frame, double robot_x,
                     double robot_y, double robot_yaw) {
    if (auto* buffer = autonomy::transform::Buffer::Instance()) {
        buffer->clear();
    }
    PublishMockStaticTransform(global_frame, kOdomFrame, 0.0, 0.0, 0.0);
    PublishMockTransform(kOdomFrame, robot_frame, robot_x, robot_y, robot_yaw,
                         false);
    AINFO << "Mock TF " << kOdomFrame << " -> " << robot_frame << " (" << robot_x
          << ", " << robot_y << ", yaw=" << robot_yaw << ")";
}

void SeedMockOdometry(
    const std::shared_ptr<Task>& task,
    const std::string& robot_frame, double x, double y, double yaw) {
    PublishMockOdometry(task, robot_frame, x, y, yaw,
                        commsgs::geometry_msgs::Twist{});
    AINFO << "Seeded mock odometry at (" << x << ", " << y << ") in "
          << kOdomFrame;
}

void SeedDemoGlobalCostmap(const std::shared_ptr<Task>& task) {
    const auto ctx = task->task_context();
    if (!ctx || !ctx->planner) {
        return;
    }
    const auto wrapper = ctx->planner->GetCostmapWrapper();
    if (!wrapper) {
        return;
    }
    auto* costmap = wrapper->getCostmap();
    if (!costmap) {
        return;
    }
    std::unique_lock<map::costmap_2d::Costmap2D::mutex_t> lock(
        *costmap->getMutex());
    const unsigned int cell_count =
        costmap->getSizeInCellsX() * costmap->getSizeInCellsY();
    unsigned char* data = costmap->getCharMap();
    std::fill(data, data + cell_count, map::costmap_2d::FREE_SPACE);
    AINFO << "Seeded planner costmap with FREE_SPACE ("
          << (costmap->getSizeInCellsX() * costmap->getResolution()) << "m x "
          << (costmap->getSizeInCellsY() * costmap->getResolution()) << "m, "
          << costmap->getSizeInCellsX() << "x" << costmap->getSizeInCellsY()
          << " cells, res=" << costmap->getResolution() << ")";
}

void Run() {
    signal(SIGINT, SigintHandler);
    signal(SIGTERM, SigintHandler);

    autonomy::common::ShowVersion();
    LOG(INFO) << "Autonomy tasks entry (single-process Task + BT).";

    const auto sys_options = system::CreateOptions(
        autonomy::common::FLAGS_configuration_directory,
        autonomy::common::FLAGS_configuration_basename);

    auto task = std::make_shared<Task>(sys_options);
    task->start();

    RuntimeOptions runtime;
    runtime.enable_bt_tasks = true;
    runtime.config_directory = autonomy::common::FLAGS_configuration_directory;
    task->configure(runtime);

    if (!task->isSchedulerReady()) {
        AERROR << "BT navigators failed to attach.";
        task->shutdown();
        return;
    }

    if (autonomy::common::FLAGS_mock_static_tf) {
        const std::string global_frame = "map";
        const std::string robot_frame = "base_link";
        SetupMockTfTree(global_frame, robot_frame, kMockRobotSpawnX,
                        kMockRobotSpawnY, kMockRobotSpawnYaw);
        SeedDemoGlobalCostmap(task);
        SeedMockOdometry(task, robot_frame, kMockRobotSpawnX,
                         kMockRobotSpawnY, kMockRobotSpawnYaw);
    }

    if (autonomy::common::FLAGS_run_navigate_to_pose) {
        const std::string frame = "map";
        const std::string robot_frame = "base_link";
        auto goal = MakeNavigateGoal(autonomy::common::FLAGS_nav_goal_x,
                                   autonomy::common::FLAGS_nav_goal_y,
                                   autonomy::common::FLAGS_nav_goal_yaw, frame);
        AINFO << "NavigateToPose goal (" << autonomy::common::FLAGS_nav_goal_x
              << ", " << autonomy::common::FLAGS_nav_goal_y << ") in frame "
              << frame;

        std::thread cancel_on_shutdown([&task]() {
            while (!g_shutdown.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            task->requestCancel();
        });

        MockRobotState mock_state;
        std::atomic<bool> mock_sim_running{false};
        std::thread mock_sim_thread;
        if (autonomy::common::FLAGS_run_navigate_to_pose &&
            autonomy::common::FLAGS_mock_static_tf) {
            mock_sim_running.store(true);
            mock_sim_thread = std::thread(RunMockSimulation, task,
                                          robot_frame, &mock_state,
                                          &mock_sim_running);
            AINFO << "Mock cmd_vel integrator running (odom -> " << robot_frame
                  << ").";
        }

        const auto status = task->navigateToPose(goal);

        if (mock_sim_running.load()) {
            mock_sim_running.store(false);
            if (mock_sim_thread.joinable()) {
                mock_sim_thread.join();
            }
            AINFO << "Mock robot final pose (" << mock_state.x << ", "
                  << mock_state.y << ", yaw=" << mock_state.yaw << ")";
        }

        switch (status) {
            case behavior_tree::BtStatus::SUCCEEDED:
                AINFO << "NavigateToPose succeeded (goal reached).";
                break;
            case behavior_tree::BtStatus::CANCELED:
                AWARN << "NavigateToPose canceled.";
                break;
            case behavior_tree::BtStatus::FAILED:
            default:
                AERROR << "NavigateToPose failed.";
                break;
        }
        AINFO << "NavigateToPose finished with BT status "
              << static_cast<int>(status);

        cancel_on_shutdown.detach();
    }

    while (!g_shutdown.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    task->shutdown();
    LOG(INFO) << "Task shut down.";
}

}  // namespace
}  // namespace tasks
}  // namespace autonomy

int main(int argc, char** argv) {
    google::SetUsageMessage(
        "\n\n"
        "\033[31m Single-process autonomy tasks (behavior-tree + "
        "PlannerServer/ControllerServer).\033[0m \n"
        "  Optional: --run_navigate_to_pose --nav_goal_x=1 --nav_goal_y=2\n"
        "  Default: --mock_static_tf integrates cmd_vel into odom TF for demo\n");

    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (autonomy::common::FLAGS_verbose) {
        autonomy::common::ShowVersion();
        google::ShutdownGoogleLogging();
        return EXIT_SUCCESS;
    }

    autonomy::tasks::Run();
    google::ShutdownGoogleLogging();
    return EXIT_SUCCESS;
}
