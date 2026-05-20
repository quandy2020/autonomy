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
#include <thread>

#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/common/gflags.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/common/version.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_engine.hpp"
#include "autonomy/tasks/navigator/proto/action.pb.h"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/planning/planner_server.hpp"
#include "autonomy/tasks/scheduler/task_scheduler.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/geometry_msgs/transform_stamped.h"

namespace autonomy {
namespace tasks {
namespace {

std::atomic<bool> g_shutdown{false};

// Spawn away from costmap (0,0) corner: NavFn setupNavFn() marks map borders lethal.
constexpr double kMockRobotSpawnX = 1.0;
constexpr double kMockRobotSpawnY = 1.0;

void SigintHandler(int /*sig*/) {
    g_shutdown.store(true);
}

std::shared_ptr<behavior_tree::proto::NavigateToPoseAction::Goal>
MakeNavigateGoal(double x, double y, double yaw, const std::string& frame) {
    auto goal =
        std::make_shared<behavior_tree::proto::NavigateToPoseAction::Goal>();
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

void PublishMockStaticTransform(const std::string& parent_frame,
                                const std::string& child_frame, double x,
                                double y, double yaw) {
    geometry_msgs::TransformStamped tf;
    tf.header.stamp =
        autonomy::commsgs::builtin_interfaces::Time::Now().Nanoseconds();
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
    if (!buffer->setTransform(tf, "autonomy_tasks_main", true)) {
        AERROR << "Failed to publish mock static TF " << parent_frame << " -> "
               << child_frame;
        return;
    }
    AINFO << "Mock static TF " << parent_frame << " -> " << child_frame
          << " (" << x << ", " << y << ", yaw=" << yaw << ")";
}

void SetupMockTfTree(const std::string& global_frame,
                     const std::string& robot_frame, double robot_x,
                     double robot_y) {
    constexpr const char* kOdomFrame = "odom";
    PublishMockStaticTransform(global_frame, kOdomFrame, 0.0, 0.0, 0.0);
    PublishMockStaticTransform(kOdomFrame, robot_frame, robot_x, robot_y, 0.0);
}

void SeedDemoGlobalCostmap(
    const std::shared_ptr<scheduler::TaskScheduler>& scheduler) {
    const auto ctx = scheduler->TaskContext();
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
          << costmap->getSizeInCellsX() << "x" << costmap->getSizeInCellsY()
          << ", res=" << costmap->getResolution() << ")";
}

void Run() {
    signal(SIGINT, SigintHandler);
    signal(SIGTERM, SigintHandler);

    autonomy::common::ShowVersion();
    LOG(INFO) << "Autonomy tasks entry (single-process TaskScheduler).";

    auto scheduler = std::make_shared<scheduler::TaskScheduler>();
    scheduler->Initialize(autonomy::common::FLAGS_configuration_directory);

    if (autonomy::common::FLAGS_mock_static_tf) {
        const auto ctx = scheduler->TaskContext();
        const std::string global_frame =
            ctx && !ctx->global_frame.empty() ? ctx->global_frame : "map";
        const std::string robot_frame = ctx && !ctx->robot_base_frame.empty()
                                            ? ctx->robot_base_frame
                                            : "base_link";
        SetupMockTfTree(global_frame, robot_frame, kMockRobotSpawnX,
                        kMockRobotSpawnY);
        SeedDemoGlobalCostmap(scheduler);
    }

    if (autonomy::common::FLAGS_run_navigate_to_pose) {
        const auto ctx = scheduler->TaskContext();
        const std::string frame =
            ctx && !ctx->global_frame.empty() ? ctx->global_frame : "map";
        auto goal = MakeNavigateGoal(autonomy::common::FLAGS_nav_goal_x,
                                   autonomy::common::FLAGS_nav_goal_y,
                                   autonomy::common::FLAGS_nav_goal_yaw, frame);
        AINFO << "NavigateToPose goal (" << autonomy::common::FLAGS_nav_goal_x
              << ", " << autonomy::common::FLAGS_nav_goal_y << ") in frame "
              << frame;

        std::thread cancel_on_shutdown([&scheduler]() {
            while (!g_shutdown.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            scheduler->RequestCancel();
        });

        const auto status = scheduler->NavigateToPose(goal);
        cancel_on_shutdown.join();

        AINFO << "NavigateToPose finished with BT status "
              << static_cast<int>(status);
    }

    while (!g_shutdown.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    scheduler->Shutdown();
    LOG(INFO) << "TaskScheduler shut down.";
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
        "  Default: --mock_static_tf publishes map->odom->base_link\n");

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
