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
 
 /*
 * Graceful Controller demo application.
 *
 * 参考 nav2 graceful_controller 用法，在无 ROS2 节点的环境下：
 * - 手动创建一条简单路径（全局坐标系 "map"）
 * - 使用 Costmap2DWrapper 创建局部代价地图
 * - 调用 GracefulController::Configure / SetPlan / ComputeVelocityCommands
 * - 在一个简单循环中更新机器人位姿并打印控制指令
 *
 * 注意：这里只做控制逻辑演示，不依赖 TF 栈，tf_buffer 传入 nullptr，
 *       PathHandler 内部会直接使用路径 frame（"map"）与 costmap global frame 一致的情况。
 */

#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <atomic>
#include <vector>

#include "autolink/autolink/common/log.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/control/controller/graceful_controller/graceful_controller.hpp"
#include "autonomy/control/proto/controller_options.pb.h"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/map/proto/map_2d_option.pb.h"

using namespace autonomy;
using namespace autonomy::control;
using namespace autonomy::control::controller;
using namespace autonomy::map;
using namespace autonomy::map::costmap_2d;

namespace {

constexpr double CONTROL_DT = 0.1;   // 10 Hz
constexpr int MAX_STEPS = 500;       // 仿真步数

std::atomic<bool> g_stop_requested{false};

void SignalHandler(int /*signum*/) {
    g_stop_requested.store(true);
}

// 创建一条简单直线路径，从 (0,0) 到 (8,0)
commsgs::planning_msgs::Path CreateStraightPath() {
    commsgs::planning_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = commsgs::builtin_interfaces::Time::Now();

    auto add_pose = [&](double x, double y, double yaw) {
        commsgs::geometry_msgs::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = std::cos(yaw / 2.0);
        pose.pose.orientation.z = std::sin(yaw / 2.0);
        path.poses.push_back(pose);
    };

    constexpr double length = 8.0;
    constexpr int num_points = 20;
    for (int i = 0; i <= num_points; ++i) {
        double x = length * static_cast<double>(i) / num_points;
        add_pose(x, 0.0, 0.0);
    }

    return path;
}

// 创建一个简单的 20m x 20m 代价地图，不添加额外障碍物
std::shared_ptr<Costmap2DWrapper> CreateCostmap() {
    autonomy::map::proto::Costmap2DOptions options;
    options.set_enabled(true);
    options.set_frame_id("map");
    options.set_resolution(0.05);  // 5cm resolution
    options.set_robot_radius(0.2);
    options.set_footprint_padding(0.1);
    options.set_update_frequency(10.0);
    options.set_rolling_window(false);

    // Set costmap size (20m x 20m)
    options.set_width(400);   // 20m / 0.05m = 400 cells
    options.set_height(400);

    auto costmap = std::make_shared<Costmap2DWrapper>(options, "graceful_costmap", nullptr);
    costmap->Start();
    return costmap;
}

// 使用简单的差动运动模型更新机器人位姿
void UpdatePose(commsgs::geometry_msgs::PoseStamped& pose,
                const commsgs::geometry_msgs::TwistStamped& cmd_vel,
                double dt) {
    double vx = cmd_vel.twist.linear.x;
    double vy = cmd_vel.twist.linear.y;
    double omega = cmd_vel.twist.angular.z;

    const auto& orient = pose.pose.orientation;
    double yaw = 2.0 * std::atan2(orient.z, orient.w);

    pose.pose.position.x += (vx * std::cos(yaw) - vy * std::sin(yaw)) * dt;
    pose.pose.position.y += (vx * std::sin(yaw) + vy * std::cos(yaw)) * dt;
    yaw += omega * dt;

    pose.pose.orientation.w = std::cos(yaw / 2.0);
    pose.pose.orientation.z = std::sin(yaw / 2.0);
}

}  // namespace

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    AINFO << "Graceful Controller Application Demo";
    AINFO << "===================================";

    // 创建控制器与代价地图
    auto controller = std::make_unique<GracefulController>();
    auto costmap = CreateCostmap();

    // 构造空的 ControllerOptions，当前实现会在内部使用一套安全默认参数
    autonomy::control::proto::ControllerOptions options;

    try {
        controller->Configure(options, "graceful_app", /*tf_buffer=*/nullptr, costmap);
        controller->Activate();
    } catch (const std::exception& e) {
        AERROR << "Failed to configure graceful controller: " << e.what();
        return 1;
    }

    // 设置路径
    auto path = CreateStraightPath();
    controller->SetPlan(path);
    AINFO << "Path created, points: " << path.poses.size();

    // 初始化机器人状态
    commsgs::geometry_msgs::PoseStamped robot_pose = path.poses[0];
    commsgs::geometry_msgs::TwistStamped robot_velocity;
    robot_velocity.header.frame_id = "base_link";
    robot_velocity.header.stamp = commsgs::builtin_interfaces::Time::Now();
    robot_velocity.twist.linear.x = 0.0;
    robot_velocity.twist.linear.y = 0.0;
    robot_velocity.twist.angular.z = 0.0;

    AINFO << "Starting graceful controller simulation (Ctrl+C to quit)";

    for (int step = 0; step < MAX_STEPS && !g_stop_requested.load(); ++step) {
        commsgs::geometry_msgs::TwistStamped cmd_vel;
        std::string message;
        robot_pose.header.stamp = commsgs::builtin_interfaces::Time::Now();

        try {
            controller->ComputeVelocityCommands(robot_pose, robot_velocity, cmd_vel, nullptr, message);
        } catch (const std::exception& e) {
            AERROR << "ComputeVelocityCommands exception: " << e.what();
            break;
        }

        UpdatePose(robot_pose, cmd_vel, CONTROL_DT);
        robot_velocity.twist = cmd_vel.twist;

        std::stringstream info;
        info << "Step: " << step
             << " | x: " << std::fixed << std::setprecision(2) << robot_pose.pose.position.x
             << " | y: " << robot_pose.pose.position.y
             << " | v: " << cmd_vel.twist.linear.x
             << " | w: " << cmd_vel.twist.angular.z;
        AINFO << info.str();

        // 简单结束条件：接近路径终点
        const auto& goal = path.poses.back().pose.position;
        double dx = robot_pose.pose.position.x - goal.x;
        double dy = robot_pose.pose.position.y - goal.y;
        if (std::sqrt(dx * dx + dy * dy) < 0.2) {
            AINFO << "Reached goal neighborhood, stopping simulation.";
            break;
        }

        std::this_thread::sleep_for(std::chrono::duration<double>(CONTROL_DT));
    }

    controller->Deactivate();
    controller->Cleanup();

    AINFO << "Graceful controller simulation completed.";
    return 0;
}


