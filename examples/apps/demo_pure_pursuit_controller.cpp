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
 * Regulated Pure Pursuit Controller demo application.
 *
 * 功能：
 * - 构造一条圆形路径（frame: "map"）
 * - 创建一个简化 costmap（不启动 costmap 更新线程，直接写入代价值）
 * - 注入 /tf：每个控制周期更新 map->base_link 变换（基于当前仿真位姿）
 * - 在 costmap 上放置一个“动态障碍物”（沿圆周小幅移动），测试碰撞前向预测是否能触发停止
 *
 * 预期输出：
 * - 打印每步机器人位姿/速度/障碍物位置
 * - 若碰撞检测触发，会捕获异常并打印 “collision imminent”
 */

#include <unistd.h>

#include <atomic>
#include <csignal>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <thread>
#include <utility>
#include <vector>

#include "autolink/autolink.hpp"
#include "autolink/common/log.hpp"
#include "autolink/message/message_traits.hpp"
#include "autolink/proto/role_attributes.pb.h"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/proto/planning_msgs.pb.h"
#include "autonomy/control/controller/pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "autonomy/control/proto/controller_options.pb.h"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/map/proto/map_2d_option.pb.h"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/geometry_msgs/transform_stamped.h"

using namespace autonomy;
using namespace autonomy::control;
using namespace autonomy::control::controller::pure_pursuit_controller;
using namespace autonomy::map;
using namespace autonomy::map::costmap_2d;

namespace {

constexpr double CONTROL_DT = 0.1;   // 10 Hz
constexpr int MAX_STEPS = 1500;

std::atomic<bool> g_stop_requested{false};

void SignalHandler(int /*signum*/) {
    g_stop_requested.store(true);
}

// autolink 默认从 ${AUTOLINK_PATH}/conf/autolink.pb.conf 读取配置。
// demo 常在 /workspace/autonomy 下运行，此时环境变量可能未设置，导致读取 /autolink 下的默认配置而缺字段。
// 这里在 Init 前做一次“就地探测”，尽量让 demo 开箱可跑。
std::string GuessAutolinkWorkRoot() {
    namespace fs = std::filesystem;
    const fs::path cwd = fs::current_path();
    const fs::path conf_rel = fs::path("conf") / "autolink.pb.conf";

    // 允许直接把 conf 放到当前目录
    {
        const fs::path conf = cwd / conf_rel;
        std::error_code ec;
        if (fs::exists(conf, ec) && !ec) {
            return conf.parent_path().parent_path().string();
        }
    }

    // 从当前目录开始向上回溯若干层，查找常见源码布局：
    // <repo_root>/src/autonomy/autolink/autolink/conf/autolink.pb.conf
    fs::path base = cwd;
    for (int depth = 0; depth < 8; ++depth) {
        const fs::path conf = base / "src" / "autonomy" / "autolink" / "autolink" / conf_rel;
        std::error_code ec;
        if (fs::exists(conf, ec) && !ec) {
            return conf.parent_path().parent_path().string();
        }
        if (!base.has_parent_path()) {
            break;
        }
        base = base.parent_path();
    }
    return "";
}

double GetYaw(const commsgs::geometry_msgs::Quaternion& q) {
    return 2.0 * std::atan2(q.z, q.w);
}

std::shared_ptr<commsgs::proto::planning_msgs::Path> ToProtoPath(const commsgs::planning_msgs::Path& path) {
    auto msg = std::make_shared<commsgs::proto::planning_msgs::Path>();
    *msg->mutable_header() = commsgs::std_msgs::ToProto(path.header);
    msg->mutable_poses()->Reserve(static_cast<int>(path.poses.size()));
    for (const auto& pose : path.poses) {
        *msg->add_poses() = commsgs::geometry_msgs::ToProto(pose);
    }
    return msg;
}

std::shared_ptr<commsgs::proto::planning_msgs::Odometry> ToProtoOdom(const commsgs::geometry_msgs::PoseStamped& pose,
                                                                     const commsgs::geometry_msgs::TwistStamped& vel,
                                                                     const std::string& child_frame_id) {
    auto msg = std::make_shared<commsgs::proto::planning_msgs::Odometry>();
    *msg->mutable_header() = commsgs::std_msgs::ToProto(pose.header);
    msg->set_child_frame_id(child_frame_id);

    commsgs::geometry_msgs::PoseWithCovariance pwc;
    pwc.pose = pose.pose;
    pwc.covariance.assign(36, 0.0);
    *msg->mutable_pose() = commsgs::geometry_msgs::ToProto(pwc);

    commsgs::geometry_msgs::TwistWithCovariance twc;
    twc.twist = vel.twist;
    twc.covariance.assign(36, 0.0);
    *msg->mutable_twist() = commsgs::geometry_msgs::ToProto(twc);

    return msg;
}

commsgs::geometry_msgs::Quaternion FromYaw(double yaw) {
    commsgs::geometry_msgs::Quaternion q;
    q.w = std::cos(yaw / 2.0);
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw / 2.0);
    return q;
}

// 差动模型更新位姿（只用 vx / wz）
void UpdatePose(commsgs::geometry_msgs::PoseStamped& pose, const commsgs::geometry_msgs::TwistStamped& cmd, double dt) {
    const double yaw = GetYaw(pose.pose.orientation);
    const double v = cmd.twist.linear.x;
    const double w = cmd.twist.angular.z;

    pose.pose.position.x += v * std::cos(yaw) * dt;
    pose.pose.position.y += v * std::sin(yaw) * dt;
    pose.pose.orientation = FromYaw(yaw + w * dt);
}

commsgs::planning_msgs::Path CreateCirclePath(double cx, double cy, double r, int num_points, double start_angle,
                                              double end_angle) {
    commsgs::planning_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = commsgs::builtin_interfaces::Time::Now();

    const double total = end_angle - start_angle;
    // 这里构造“闭合”圆形路径：首尾相同，表示一圈回到起点。
    // 为了避免第 0 步就被判定“到达终点”，我们在主循环里会设置最小步数门槛，
    // 只有跑够一段时间后才允许触发 goal reached。
    for (int i = 0; i <= num_points; ++i) {
        const double t = start_angle + total * static_cast<double>(i) / static_cast<double>(num_points);
        const double x = cx + r * std::cos(t);
        const double y = cy + r * std::sin(t);
        const double yaw_tangent = t + M_PI_2;  // 逆时针切向

        commsgs::geometry_msgs::PoseStamped ps;
        ps.header = path.header;
        ps.pose.position.x = x;
        ps.pose.position.y = y;
        ps.pose.position.z = 0.0;
        ps.pose.orientation = FromYaw(yaw_tangent);
        path.poses.push_back(ps);
    }
    return path;
}

std::shared_ptr<Costmap2DWrapper> CreateCostmap(double width_m, double height_m, double resolution,
                                                autolink::Node* node) {
    autonomy::map::proto::Costmap2DOptions options;
    options.set_enabled(true);
    options.set_frame_id("map");
    options.set_resolution(resolution);
    options.set_robot_radius(0.22);
    options.set_footprint_padding(0.05);
    options.set_update_frequency(10.0);
    options.set_rolling_window(false);

    // 注意：wrapper 内部把 width/height 当作 “米”（但 proto 是 int32）
    options.set_width(static_cast<int32_t>(std::lround(width_m)));
    options.set_height(static_cast<int32_t>(std::lround(height_m)));

    auto costmap = std::make_shared<Costmap2DWrapper>(options, "pp_costmap", node);

    // 不启动 costmap 的 update thread（Start() 会周期性 updateMap，从而覆盖手动写入的障碍物）
    // costmap->Start();
    return costmap;
}

geometry_msgs::TransformStamped MakeMapToBaseLinkTf(const commsgs::geometry_msgs::PoseStamped& pose) {
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = pose.header.stamp.Nanoseconds();
    tf.header.frame_id = "map";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = pose.pose.position.x;
    tf.transform.translation.y = pose.pose.position.y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = pose.pose.orientation.x;
    tf.transform.rotation.y = pose.pose.orientation.y;
    tf.transform.rotation.z = pose.pose.orientation.z;
    tf.transform.rotation.w = pose.pose.orientation.w;
    return tf;
}

// 将圆形障碍物写入 costmap（返回本次写入的 cell 集合，便于下次清理）
std::vector<std::pair<unsigned int, unsigned int>> PaintObstacle(Costmap2D* costmap, double x, double y,
                                                                 double radius_m,
                                                                 unsigned char cost_value = LETHAL_OBSTACLE) {
    std::vector<std::pair<unsigned int, unsigned int>> cells;
    if (!costmap) {
        return cells;
    }

    const double res = costmap->getResolution();
    const int r_cells = std::max(1, static_cast<int>(std::ceil(radius_m / res)));

    unsigned int cx, cy;
    if (!costmap->worldToMap(x, y, cx, cy)) {
        return cells;
    }

    for (int dx = -r_cells; dx <= r_cells; ++dx) {
        for (int dy = -r_cells; dy <= r_cells; ++dy) {
            if (dx * dx + dy * dy > r_cells * r_cells) {
                continue;
            }
            const int mx = static_cast<int>(cx) + dx;
            const int my = static_cast<int>(cy) + dy;
            if (mx < 0 || my < 0) {
                continue;
            }
            const auto umx = static_cast<unsigned int>(mx);
            const auto umy = static_cast<unsigned int>(my);
            // setCost 内部会检查越界
            costmap->setCost(umx, umy, cost_value);
            cells.emplace_back(umx, umy);
        }
    }
    return cells;
}

void ClearCells(Costmap2D* costmap, const std::vector<std::pair<unsigned int, unsigned int>>& cells,
                unsigned char value = FREE_SPACE) {
    if (!costmap) {
        return;
    }
    for (const auto& c : cells) {
        costmap->setCost(c.first, c.second, value);
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    // 在使用任何 autolink API（包括 Costmap2DWrapper 内部创建 Node/加载插件）之前必须初始化
    // 说明：autolink 当前的 atexit 清理链路可能在进程退出阶段触发析构顺序问题（导致退出时崩溃）。
    // 这个 demo 不依赖 autolink 的优雅退出，因此显式关闭 atexit 清理，避免“运行结束后段错误”。
    ::setenv("AUTOLINK_DISABLE_ATEXIT", "1", /*overwrite=*/0);

    if (std::getenv("AUTOLINK_PATH") == nullptr) {
        const auto guessed = GuessAutolinkWorkRoot();
        if (!guessed.empty()) {
            ::setenv("AUTOLINK_PATH", guessed.c_str(), /*overwrite=*/0);
        }
    }

    if (argc > 0 && argv && argv[0]) {
        autolink::Init(argv[0]);
    } else {
        autolink::Init("demo_pure_pursuit_controller");
    }

    AINFO << "Regulated Pure Pursuit Controller Demo";
    AINFO << "=====================================";

    // 0) 创建 autolink node 与 writers（用于可视化）
    auto node = autolink::CreateNode("demo_pure_pursuit_controller_" + std::to_string(getpid()));
    if (!node) {
        AERROR << "Failed to create autolink node";
        return 1;
    }
    using autolink::message::MessageType;
    using autolink::proto::RoleAttributes;

    // 发布路径（planning_msgs/Path）
    RoleAttributes path_attr;
    path_attr.set_channel_name("/demo/pure_pursuit/path");
    path_attr.set_message_type(MessageType<commsgs::proto::planning_msgs::Path>());
    auto path_writer = node->CreateWriter<commsgs::proto::planning_msgs::Path>(path_attr);

    // 发布里程计（planning_msgs/Odometry：包含 pose + velocity，visualization 已支持速度箭头显示）
    RoleAttributes odom_attr;
    odom_attr.set_channel_name("/demo/pure_pursuit/odometry");
    odom_attr.set_message_type(MessageType<commsgs::proto::planning_msgs::Odometry>());
    auto odom_writer = node->CreateWriter<commsgs::proto::planning_msgs::Odometry>(odom_attr);

    if (!path_writer || !odom_writer) {
        AERROR << "Failed to create path/odometry writers";
        return 1;
    }

    // 1) 构造 costmap 与 tf buffer
    auto costmap_wrapper = CreateCostmap(/*width_m=*/30.0, /*height_m=*/30.0, /*resolution=*/0.05, node.get());
    auto* costmap = costmap_wrapper->getCostmap();

    // transform::Buffer 使用 DECLARE_SINGLETON 定义为单例（构造函数私有）
    // 这里用空 deleter 的 shared_ptr 仅用于满足接口签名，不负责释放单例对象。
    auto* tf_raw = autonomy::transform::Buffer::Instance();
    if (!tf_raw) {
        AERROR << "Failed to create transform::Buffer singleton";
        return 1;
    }
    auto tf =
        std::shared_ptr<autonomy::transform::Buffer>(tf_raw, [](autonomy::transform::Buffer*) { /* no-op */ });

    // 2) 构造 controller options（重点：pure_pursuit_controller_options）
    autonomy::control::proto::ControllerOptions options;
    auto* pp = options.mutable_pure_pursuit_controller_options();
    pp->set_desired_linear_vel(0.7);
    pp->set_lookahead_dist(0.8);
    pp->set_use_velocity_scaled_lookahead_dist(false);
    pp->set_use_collision_detection(true);
    pp->set_max_allowed_time_to_collision_up_to_carrot(1.2);
    pp->set_use_rotate_to_heading(true);
    pp->set_rotate_to_heading_min_angle(0.785);
    pp->set_rotate_to_heading_angular_vel(1.5);
    pp->set_max_angular_accel(3.2);
    pp->set_use_regulated_linear_velocity_scaling(true);
    pp->set_regulated_linear_scaling_min_radius(0.9);
    pp->set_regulated_linear_scaling_min_speed(0.25);
    pp->set_use_cost_regulated_linear_velocity_scaling(true);
    pp->set_cost_scaling_dist(0.6);
    pp->set_cost_scaling_gain(1.0);
    pp->set_inflation_cost_scaling_factor(3.0);
    pp->set_min_approach_linear_velocity(0.05);
    pp->set_approach_velocity_scaling_dist(0.6);
    pp->set_transform_tolerance(0.1);
    pp->set_stateful(true);

    // 3) 创建圆形路径（持续循环跟随）
    const double cx = 15.0;
    const double cy = 15.0;
    const double r = 6.0;
    auto path = CreateCirclePath(cx, cy, r, /*num_points=*/220, /*start=*/0.0, /*end=*/2.0 * M_PI);
    auto path_proto = ToProtoPath(path);
    // 给 AutoDiscovery 一点时间发现 writer
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    path_writer->Write(path_proto);

    // 4) 初始化 controller
    RegulatedPurePursuitController controller;
    controller.Configure(options, "demo_pure_pursuit_controller", tf, costmap_wrapper);
    controller.Activate();

    // 5) 循环跟随：到达目标/碰撞/超时后，停一会并重置再开始下一圈
    int lap = 0;
    while (autolink::OK() && !g_stop_requested.load()) {
        ++lap;
        AINFO << "====================";
        AINFO << "Start lap #" << lap;
        AINFO << "====================";

        // 每圈都重置 plan / controller 状态，并重新发布 path（避免可视化端后连接时看不到）
        controller.SetPlan(path);
        controller.Reset();
        path_writer->Write(path_proto);

        // 初始化机器人状态（从圆形起点出发）
        commsgs::geometry_msgs::PoseStamped robot_pose = path.poses.front();
        robot_pose.header.frame_id = "map";
        commsgs::geometry_msgs::TwistStamped robot_vel;
        robot_vel.header.frame_id = "base_link";
        robot_vel.twist.linear.x = 0.0;
        robot_vel.twist.angular.z = 0.0;

        std::vector<std::pair<unsigned int, unsigned int>> last_obstacle_cells;
        bool reached_goal = false;
        bool collision_triggered = false;

        // 至少跑够一定步数再允许判定“到达终点”，避免闭合路径在第 0 步误判
        const int min_steps_before_goal_check = 300;  // 约 30s（dt=0.1）

        for (int step = 0; step < MAX_STEPS && autolink::OK() && !g_stop_requested.load(); ++step) {
            const auto now = commsgs::builtin_interfaces::Time::Now();
            robot_pose.header.stamp = now;
            robot_vel.header.stamp = now;

            // 更新 TF: map->base_link
            const auto tf_map_base = MakeMapToBaseLinkTf(robot_pose);
            tf->setTransform(tf_map_base, "demo", /*is_static=*/false);

            // 动态障碍物：在某一段时间内沿圆周小幅摆动（落在 path 上）
            ClearCells(costmap, last_obstacle_cells, FREE_SPACE);
            last_obstacle_cells.clear();

            const bool obstacle_active = (step > 150 && step < 650);
            double obs_x = 0.0, obs_y = 0.0;
            if (obstacle_active) {
                const double obs_angle = M_PI_2 + 0.6 * std::sin(0.03 * step);
                obs_x = cx + r * std::cos(obs_angle);
                obs_y = cy + r * std::sin(obs_angle);
                last_obstacle_cells = PaintObstacle(costmap, obs_x, obs_y, /*radius_m=*/0.35, LETHAL_OBSTACLE);
            }

            // 计算控制指令
            commsgs::geometry_msgs::TwistStamped cmd;
            std::string message;
            try {
                controller.ComputeVelocityCommands(robot_pose, robot_vel, cmd, /*goal_checker=*/nullptr, message);
            } catch (const std::exception& e) {
                collision_triggered = true;
                AERROR << "ComputeVelocityCommands exception (likely collision): " << e.what();
                break;
            }

            // 发布 odometry（用于可视化机器人位置与速度）
            odom_writer->Write(ToProtoOdom(robot_pose, cmd, /*child_frame_id=*/"base_link"));

            // 更新仿真状态
            UpdatePose(robot_pose, cmd, CONTROL_DT);
            robot_vel.twist = cmd.twist;

            // 打印信息（每圈从 step=0 重新计数）
            std::stringstream ss;
            ss << "lap=" << std::setw(3) << lap << " step=" << std::setw(4) << step
               << " pose=(" << std::fixed << std::setprecision(2) << robot_pose.pose.position.x << ", "
               << robot_pose.pose.position.y << ", yaw=" << GetYaw(robot_pose.pose.orientation) << ")"
               << " cmd=(v=" << cmd.twist.linear.x << ", w=" << cmd.twist.angular.z << ")";
            if (obstacle_active) {
                ss << " obstacle=(" << obs_x << ", " << obs_y << ")";
            }
            AINFO << ss.str();

            // 结束条件：完成一圈（回到起点附近）
            if (step >= min_steps_before_goal_check &&
                controller.IsGoalReached(/*dist_tol=*/0.35, /*angle_tol=*/0.60)) {
                reached_goal = true;
                AINFO << "Goal reached (controller.IsGoalReached)!";
                break;
            }

            std::this_thread::sleep_for(std::chrono::duration<double>(CONTROL_DT));
        }

        // 清理障碍物并“停一下”
        ClearCells(costmap, last_obstacle_cells, FREE_SPACE);

        if (reached_goal) {
            AINFO << "LAP RESULT: SUCCESS (reached goal) -> restarting...";
        } else if (collision_triggered) {
            AINFO << "LAP RESULT: STOPPED (collision imminent) -> restarting...";
        } else {
            AINFO << "LAP RESULT: INCONCLUSIVE (max steps reached) -> restarting...";
        }

        if (g_stop_requested.load()) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    AINFO << "Shutdown requested, exiting.";
    google::FlushLogFiles(google::GLOG_INFO);
    std::_Exit(0);
}

