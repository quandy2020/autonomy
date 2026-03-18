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
 * Navigation-to-pose demo (global NavFn + local pure pursuit).
 *
 * 目标：
 * - 参考 demo_navfn_planner.cpp：在 2D 栅格地图上用 NavfnPlanner 做全局路径规划
 * - 参考 demo_pure_pursuit_controller.cpp：用 RegulatedPurePursuitController 做局部路径跟踪（仿真差动模型）
 * - 完成一个“规划->跟踪到终点”的周期，并重复执行（默认无限循环，Ctrl+C 退出）
 *
 * 可视化（autonomy.visualization.launcher 自动发现）：
 * - /demo/nav_to_pose/map        (map_msgs/OccupancyGrid -> Foxglove Grid)
 * - /demo/nav_to_pose/global_path (planning_msgs/Path -> SceneUpdate)
 * - /demo/nav_to_pose/start_goal  (geometry_msgs/PoseArray -> SceneUpdate)
 * - /demo/nav_to_pose/odometry    (planning_msgs/Odometry -> SceneUpdate)
 *
 * 用法（容器内）：
 *   # 默认无限循环，Ctrl+C 退出
 *   ./autonomy.examples.apps.demo_navigation_to_pose --map turtlebot3_house
 *
 *   # 也可指定循环次数
 *   ./autonomy.examples.apps.demo_navigation_to_pose --map turtlebot3_house --cycles 2
 *   ./autonomy.examples.apps.demo_navigation_to_pose --map /path/to/map.yaml --cycles 3
 *   ./autonomy.examples.apps.demo_navigation_to_pose --map /path/to/map.pgm --resolution 0.05 --origin_x 0 --origin_y 0
 */

#include <unistd.h>

#include <atomic>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <string>
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
#include "autonomy/commsgs/proto/geometry_msgs.pb.h"
#include "autonomy/commsgs/proto/map_msgs.pb.h"
#include "autonomy/commsgs/proto/planning_msgs.pb.h"
#include "autonomy/commsgs/std_msgs.hpp"
#include "autonomy/control/controller/pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "autonomy/control/proto/controller_options.pb.h"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/map/proto/map_2d_option.pb.h"
#include "autonomy/map/utils/data_loader_utils.hpp"
#include "autonomy/planning/planner/navfn/navfn_planner.hpp"
#include "autonomy/planning/proto/planning_options.pb.h"
#include "autonomy/planning/utils/pgm_converter.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/geometry_msgs/transform_stamped.h"

using namespace autonomy;
using namespace autonomy::control;
using namespace autonomy::control::controller::pure_pursuit_controller;
using namespace autonomy::map;
using namespace autonomy::map::costmap_2d;

namespace {

constexpr double CONTROL_DT = 0.1;  // 10 Hz
constexpr int MAX_STEPS_PER_CYCLE = 2500;

std::atomic<bool> g_stop_requested{false};

void SignalHandler(int /*signum*/) {
    g_stop_requested.store(true);
}

std::string GuessAutolinkWorkRoot() {
    namespace fs = std::filesystem;
    const fs::path cwd = fs::current_path();
    const fs::path conf_rel = fs::path("conf") / "autolink.pb.conf";

    // <repo_root>/src/autonomy/autolink/autolink/conf/autolink.pb.conf
    fs::path base = cwd;
    for (int depth = 0; depth < 10; ++depth) {
        const fs::path conf = base / "src" / "autonomy" / "autolink" / "autolink" / conf_rel;
        std::error_code ec;
        if (fs::exists(conf, ec) && !ec) {
            return conf.parent_path().parent_path().string();  // .../autolink/autolink
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

std::shared_ptr<commsgs::proto::geometry_msgs::PoseArray> MakeStartGoalPoseArray(const commsgs::geometry_msgs::PoseStamped& start,
                                                                                 const commsgs::geometry_msgs::PoseStamped& goal) {
    auto arr = std::make_shared<commsgs::proto::geometry_msgs::PoseArray>();
    *arr->mutable_header() = commsgs::std_msgs::ToProto(start.header);
    *arr->add_poses() = commsgs::geometry_msgs::ToProto(start.pose);
    *arr->add_poses() = commsgs::geometry_msgs::ToProto(goal.pose);
    return arr;
}

commsgs::geometry_msgs::PoseStamped CreatePose(double x, double y, const std::string& frame_id = "map") {
    commsgs::geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.header.stamp = commsgs::builtin_interfaces::Time::Now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    return pose;
}

commsgs::proto::std_msgs::Header CreateHeaderProto(const std::string& frame_id) {
    commsgs::std_msgs::Header header;
    header.frame_id = frame_id;
    header.stamp = commsgs::builtin_interfaces::Time::Now();
    return commsgs::std_msgs::ToProto(header);
}

std::shared_ptr<commsgs::proto::map_msgs::OccupancyGrid> ToProtoOccupancyGrid(const autonomy::map::costmap_2d::Costmap2D& costmap,
                                                                              const std::string& frame_id) {
    auto msg = std::make_shared<commsgs::proto::map_msgs::OccupancyGrid>();
    *msg->mutable_header() = CreateHeaderProto(frame_id);

    auto* info = msg->mutable_info();
    info->set_width(costmap.getSizeInCellsX());
    info->set_height(costmap.getSizeInCellsY());
    info->set_resolution(static_cast<float>(costmap.getResolution()));
    *info->mutable_map_load_time() = commsgs::builtin_interfaces::ToProto(commsgs::builtin_interfaces::Time::Now());

    auto* origin = info->mutable_origin();
    origin->mutable_position()->set_x(costmap.getOriginX());
    origin->mutable_position()->set_y(costmap.getOriginY());
    origin->mutable_position()->set_z(0.0);
    origin->mutable_orientation()->set_x(0.0);
    origin->mutable_orientation()->set_y(0.0);
    origin->mutable_orientation()->set_z(0.0);
    origin->mutable_orientation()->set_w(1.0);

    const int width = static_cast<int>(costmap.getSizeInCellsX());
    const int height = static_cast<int>(costmap.getSizeInCellsY());
    msg->mutable_data()->Reserve(width * height);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const unsigned char c = costmap.getCost(static_cast<unsigned int>(x), static_cast<unsigned int>(y));
            int32_t occ = 0;
            if (c == autonomy::map::costmap_2d::NO_INFORMATION) {
                occ = -1;
            } else if (c >= autonomy::map::costmap_2d::LETHAL_OBSTACLE) {
                occ = 100;
            } else if (c <= autonomy::map::costmap_2d::FREE_SPACE) {
                occ = 0;
            } else {
                const double scaled =
                    100.0 * static_cast<double>(c) / static_cast<double>(autonomy::map::costmap_2d::LETHAL_OBSTACLE);
                occ = static_cast<int32_t>(std::max(0.0, std::min(100.0, scaled)));
            }
            msg->mutable_data()->Add(occ);
        }
    }
    return msg;
}

double Dist2D(const commsgs::geometry_msgs::PoseStamped& a, const commsgs::geometry_msgs::PoseStamped& b) {
    const double dx = a.pose.position.x - b.pose.position.x;
    const double dy = a.pose.position.y - b.pose.position.y;
    return std::sqrt(dx * dx + dy * dy);
}

bool SampleFreePose(const Costmap2D& cm, std::mt19937& rng, commsgs::geometry_msgs::PoseStamped* out_pose) {
    if (!out_pose) {
        return false;
    }
    const unsigned int size_x = cm.getSizeInCellsX();
    const unsigned int size_y = cm.getSizeInCellsY();
    if (size_x == 0 || size_y == 0) {
        return false;
    }

    std::uniform_int_distribution<unsigned int> dist_x(0, size_x - 1);
    std::uniform_int_distribution<unsigned int> dist_y(0, size_y - 1);

    const int max_tries = 2000;
    for (int i = 0; i < max_tries; ++i) {
        const unsigned int mx = dist_x(rng);
        const unsigned int my = dist_y(rng);
        const unsigned char c = cm.getCost(mx, my);
        if (c < autonomy::map::costmap_2d::LETHAL_OBSTACLE && c != autonomy::map::costmap_2d::NO_INFORMATION) {
            double wx = 0.0, wy = 0.0;
            cm.mapToWorld(mx, my, wx, wy);
            *out_pose = CreatePose(wx, wy, "map");
            return true;
        }
    }
    return false;
}

struct Args {
    std::string map_arg;  // name or path
    // cycles <= 0 表示无限循环（直到 Ctrl+C）
    int cycles = 0;
    bool has_resolution = false;
    double resolution = 0.05;
    bool has_origin = false;
    double origin_x = 0.0;
    double origin_y = 0.0;
    double origin_yaw = 0.0;
    bool has_start = false;
    double start_x = 0.0;
    double start_y = 0.0;
    bool has_goal = false;
    double goal_x = 0.0;
    double goal_y = 0.0;
    uint32_t seed = 0;
    bool has_seed = false;
};

Args ParseArgs(int argc, char* argv[]) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i] ? argv[i] : "";
        auto need_next = [&]() -> bool { return i + 1 < argc; };
        auto next = [&]() -> std::string { return argv[++i]; };
        auto next_double = [&]() -> double { return std::stod(argv[++i]); };
        auto next_int = [&]() -> int { return std::stoi(argv[++i]); };
        auto next_u32 = [&]() -> uint32_t { return static_cast<uint32_t>(std::stoul(argv[++i])); };

        if (a == "--help" || a == "-h") {
            std::cout << "Usage:\n"
                      << "  " << argv[0] << " [--map <name|path.(pgm|yaml)>] [--cycles N]\n"
                      << "     [--start x y] [--goal x y]\n"
                      << "     [--resolution r] [--origin_x ox] [--origin_y oy] [--origin_yaw yaw]\n";
            std::_Exit(0);
        } else if (a == "--map" && need_next()) {
            args.map_arg = next();
        } else if (a == "--cycles" && need_next()) {
            args.cycles = std::max(1, next_int());
        } else if (a == "--resolution" && need_next()) {
            args.has_resolution = true;
            args.resolution = next_double();
        } else if (a == "--origin_x" && need_next()) {
            args.has_origin = true;
            args.origin_x = next_double();
        } else if (a == "--origin_y" && need_next()) {
            args.has_origin = true;
            args.origin_y = next_double();
        } else if (a == "--origin_yaw" && need_next()) {
            args.has_origin = true;
            args.origin_yaw = next_double();
        } else if (a == "--start" && i + 2 < argc) {
            args.has_start = true;
            args.start_x = next_double();
            args.start_y = next_double();
        } else if (a == "--goal" && i + 2 < argc) {
            args.has_goal = true;
            args.goal_x = next_double();
            args.goal_y = next_double();
        } else if (a == "--seed" && need_next()) {
            args.has_seed = true;
            args.seed = next_u32();
        }
    }
    return args;
}

struct MapSelection {
    enum class Kind { kYaml, kPgm };
    Kind kind;
    std::string path;
};

MapSelection ResolveMapSelection(const std::string& map_arg, const std::string& default_map_dir) {
    namespace fs = std::filesystem;
    std::string input = map_arg.empty() ? "turtlebot3_house" : map_arg;

    const bool has_path_sep = (input.find('/') != std::string::npos);
    const bool has_ext = (input.find('.') != std::string::npos);
    if (!has_path_sep && !has_ext) {
        const fs::path base = fs::path(default_map_dir) / input;
        const fs::path yaml = base.string() + ".yaml";
        const fs::path pgm = base.string() + ".pgm";
        std::error_code ec;
        if (fs::exists(yaml, ec) && !ec) {
            return {MapSelection::Kind::kYaml, yaml.string()};
        }
        if (fs::exists(pgm, ec) && !ec) {
            return {MapSelection::Kind::kPgm, pgm.string()};
        }
        return {MapSelection::Kind::kPgm, base.string()};
    }

    fs::path p(input);
    std::string ext = p.extension().string();
    for (auto& ch : ext) {
        ch = static_cast<char>(std::tolower(ch));
    }

    if (ext == ".pgm" || ext == ".png") {
        fs::path yaml = p;
        yaml.replace_extension(".yaml");
        std::error_code ec;
        if (fs::exists(yaml, ec) && !ec) {
            return {MapSelection::Kind::kYaml, yaml.string()};
        }
        return {MapSelection::Kind::kPgm, p.string()};
    }

    if (ext == ".yaml" || ext == ".yml") {
        return {MapSelection::Kind::kYaml, p.string()};
    }

    fs::path yaml = p;
    yaml.replace_extension(".yaml");
    fs::path pgm = p;
    pgm.replace_extension(".pgm");
    std::error_code ec;
    if (fs::exists(yaml, ec) && !ec) {
        return {MapSelection::Kind::kYaml, yaml.string()};
    }
    if (fs::exists(pgm, ec) && !ec) {
        return {MapSelection::Kind::kPgm, pgm.string()};
    }
    return {MapSelection::Kind::kPgm, p.string()};
}

int Run(int argc, char* argv[]) {
    using autonomy::planning::planner::navfn::NavfnPlanner;
    using autonomy::planning::utils::PgmConverter;
    using autolink::message::MessageType;
    using autolink::proto::RoleAttributes;

    const Args args = ParseArgs(argc, argv);
    const std::string default_map_dir = autonomy::map::utils::GetMapDataFilesDirectory();
    const auto selection = ResolveMapSelection(args.map_arg, default_map_dir);

    AINFO << "Navigation-to-pose demo (NavFn + PurePursuit)";
    AINFO << "Map: " << selection.path << (selection.kind == MapSelection::Kind::kYaml ? " (yaml)" : " (pgm)");
    if (args.cycles > 0) {
        AINFO << "Cycles: " << args.cycles;
    } else {
        AINFO << "Cycles: infinite (Ctrl+C to stop)";
    }

    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    // 1) load map -> costmap
    autonomy::map::costmap_2d::Costmap2D::SharedPtr map_costmap;
    if (selection.kind == MapSelection::Kind::kYaml) {
        map_costmap = PgmConverter::loadFromYaml(selection.path);
    } else {
        PgmConverter::LoadParameters lp;
        if (args.has_resolution) {
            lp.resolution = args.resolution;
        }
        if (args.has_origin) {
            lp.origin_x = args.origin_x;
            lp.origin_y = args.origin_y;
            lp.origin_yaw = args.origin_yaw;
        }
        map_costmap = PgmConverter::loadFromPgm(selection.path, lp);
    }
    if (!map_costmap) {
        AERROR << "Failed to load map: " << selection.path;
        return 1;
    }
    AINFO << "Map loaded: " << map_costmap->getSizeInCellsX() << "x" << map_costmap->getSizeInCellsY()
          << " cells, resolution=" << map_costmap->getResolution() << " origin=(" << map_costmap->getOriginX() << ", "
          << map_costmap->getOriginY() << ")";

    // 2) create autolink node & writers (visualization)
    auto node = autolink::CreateNode("demo_navigation_to_pose_" + std::to_string(getpid()));
    if (!node) {
        AERROR << "Failed to create autolink node";
        return 1;
    }

    RoleAttributes map_attr;
    map_attr.set_channel_name("/demo/nav_to_pose/map");
    map_attr.set_message_type(MessageType<commsgs::proto::map_msgs::OccupancyGrid>());
    auto map_writer = node->CreateWriter<commsgs::proto::map_msgs::OccupancyGrid>(map_attr);

    RoleAttributes path_attr;
    path_attr.set_channel_name("/demo/nav_to_pose/global_path");
    path_attr.set_message_type(MessageType<commsgs::proto::planning_msgs::Path>());
    auto path_writer = node->CreateWriter<commsgs::proto::planning_msgs::Path>(path_attr);

    RoleAttributes start_goal_attr;
    start_goal_attr.set_channel_name("/demo/nav_to_pose/start_goal");
    start_goal_attr.set_message_type(MessageType<commsgs::proto::geometry_msgs::PoseArray>());
    auto start_goal_writer = node->CreateWriter<commsgs::proto::geometry_msgs::PoseArray>(start_goal_attr);

    RoleAttributes odom_attr;
    odom_attr.set_channel_name("/demo/nav_to_pose/odometry");
    odom_attr.set_message_type(MessageType<commsgs::proto::planning_msgs::Odometry>());
    auto odom_writer = node->CreateWriter<commsgs::proto::planning_msgs::Odometry>(odom_attr);

    // 额外发布 cmd_vel（速度指令）
    RoleAttributes cmd_attr;
    cmd_attr.set_channel_name("cmd_vel");
    cmd_attr.set_message_type(MessageType<commsgs::proto::geometry_msgs::TwistStamped>());
    auto cmd_writer = node->CreateWriter<commsgs::proto::geometry_msgs::TwistStamped>(cmd_attr);

    if (!map_writer || !path_writer || !start_goal_writer || !odom_writer || !cmd_writer) {
        AERROR << "Failed to create one or more writers";
        return 1;
    }

    // 让 AutoDiscovery 有机会发现 writer（避免第一次发布就“看不见”）
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // 3) build Costmap2DWrapper (planner & controller use wrapper)
    autonomy::map::proto::Costmap2DOptions costmap_options;
    costmap_options.set_enabled(true);
    costmap_options.set_frame_id("map");
    costmap_options.set_name("nav_to_pose_costmap");
    costmap_options.set_resolution(map_costmap->getResolution());
    costmap_options.set_robot_radius(0.22);
    costmap_options.set_footprint_padding(0.05);

    auto costmap_wrapper = std::make_shared<Costmap2DWrapper>(costmap_options, "nav_to_pose_costmap", nullptr);
    auto* wrapper_cm = costmap_wrapper->getCostmap();
    if (!wrapper_cm) {
        AERROR << "Wrapper costmap is null";
        return 1;
    }

    const unsigned int size_x = map_costmap->getSizeInCellsX();
    const unsigned int size_y = map_costmap->getSizeInCellsY();
    wrapper_cm->resizeMap(size_x, size_y, map_costmap->getResolution(), map_costmap->getOriginX(), map_costmap->getOriginY());
    for (unsigned int y = 0; y < size_y; ++y) {
        for (unsigned int x = 0; x < size_x; ++x) {
            wrapper_cm->setCost(x, y, map_costmap->getCost(x, y));
        }
    }

    // 发布地图（周期性重发，防止订阅晚于第一次发布）
    map_writer->Write(ToProtoOccupancyGrid(*map_costmap, "map"));

    // 4) TF buffer singleton
    auto* tf_raw = autonomy::transform::Buffer::Instance();
    if (!tf_raw) {
        AERROR << "Failed to get transform::Buffer singleton";
        return 1;
    }
    auto tf = std::shared_ptr<autonomy::transform::Buffer>(tf_raw, [](autonomy::transform::Buffer*) { /*no-op*/ });

    // 5) configure NavfnPlanner
    autonomy::planning::proto::PlannerOptions planner_options;
    auto* navfn_opt = planner_options.mutable_navfn();
    navfn_opt->set_tolerance(0.0);
    navfn_opt->set_use_astar(false);
    navfn_opt->set_allow_unknown(true);
    navfn_opt->set_use_final_approach_orientation(false);

    NavfnPlanner planner;
    if (!planner.Configure(planner_options, "nav_to_pose_navfn", costmap_wrapper)) {
        AERROR << "Failed to configure NavfnPlanner";
        return 1;
    }
    planner.Activate();

    // 6) configure RegulatedPurePursuitController
    autonomy::control::proto::ControllerOptions ctrl_options;
    auto* pp = ctrl_options.mutable_pure_pursuit_controller_options();
    pp->set_desired_linear_vel(0.7);
    pp->set_lookahead_dist(0.8);
    pp->set_use_velocity_scaled_lookahead_dist(false);
    // navigation demo 默认关闭碰撞检测（避免随机起终点/路径贴障导致每步都抛异常）
    pp->set_use_collision_detection(false);
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

    RegulatedPurePursuitController controller;
    controller.Configure(ctrl_options, "nav_to_pose_pp", tf, costmap_wrapper);
    controller.Activate();

    // 7) RNG
    std::mt19937 rng(args.has_seed ? args.seed : std::random_device{}());

    // 8) cycles: plan -> track to goal
    commsgs::geometry_msgs::PoseStamped robot_pose;
    bool robot_pose_initialized = false;

    int cycle = 0;
    while (autolink::OK() && !g_stop_requested.load()) {
        ++cycle;
        if (args.cycles > 0 && cycle > args.cycles) {
            break;
        }
        AINFO << "==============================";
        AINFO << "Cycle #" << cycle << " planning...";
        AINFO << "==============================";

        commsgs::geometry_msgs::PoseStamped start_pose;
        if (cycle == 1 && args.has_start) {
            start_pose = CreatePose(args.start_x, args.start_y, "map");
        } else if (robot_pose_initialized) {
            start_pose = robot_pose;
            start_pose.header.frame_id = "map";
        } else {
            if (!SampleFreePose(*wrapper_cm, rng, &start_pose)) {
                AERROR << "Failed to sample free start pose";
                return 1;
            }
        }

        commsgs::geometry_msgs::PoseStamped goal_pose;
        if (cycle == 1 && args.has_goal) {
            goal_pose = CreatePose(args.goal_x, args.goal_y, "map");
        } else {
            // 随机目标，尽量离起点远一点
            bool ok = false;
            for (int tries = 0; tries < 50; ++tries) {
                if (!SampleFreePose(*wrapper_cm, rng, &goal_pose)) {
                    continue;
                }
                if (Dist2D(start_pose, goal_pose) > 2.0) {
                    ok = true;
                    break;
                }
            }
            if (!ok) {
                AERROR << "Failed to sample valid goal pose";
                return 1;
            }
        }

        AINFO << "Start=(" << start_pose.pose.position.x << ", " << start_pose.pose.position.y << ")"
              << " Goal=(" << goal_pose.pose.position.x << ", " << goal_pose.pose.position.y << ")";

        // plan
        commsgs::planning_msgs::Path global_path;
        auto cancel_checker = []() { return false; };
        const autonomy::uint32 result_code = planner.CreatePlan(start_pose, goal_pose, global_path, cancel_checker);
        if (result_code != static_cast<autonomy::uint32>(autonomy::planning::proto::PlannerResultCode::PLANNER_SUCCESS) ||
            global_path.poses.empty()) {
            AERROR << "CreatePlan failed, code=" << result_code << " poses=" << global_path.poses.size();
            continue;
        }

        global_path.header.frame_id = "map";
        global_path.header.stamp = commsgs::builtin_interfaces::Time::Now();

        // publish map + start/goal + global path
        map_writer->Write(ToProtoOccupancyGrid(*map_costmap, "map"));
        start_goal_writer->Write(MakeStartGoalPoseArray(start_pose, goal_pose));
        path_writer->Write(ToProtoPath(global_path));

        // local tracking setup
        controller.SetPlan(global_path);
        controller.Reset();

        robot_pose = start_pose;
        robot_pose_initialized = true;

        commsgs::geometry_msgs::TwistStamped robot_vel;
        robot_vel.header.frame_id = "base_link";
        robot_vel.twist.linear.x = 0.0;
        robot_vel.twist.angular.z = 0.0;

        bool reached_goal = false;
        bool failed = false;

        // 每 2s 重发一次地图（确保 Grid 可见）
        const int map_publish_period_steps = static_cast<int>(std::lround(2.0 / CONTROL_DT));

        for (int step = 0; step < MAX_STEPS_PER_CYCLE && autolink::OK() && !g_stop_requested.load(); ++step) {
            const auto now = commsgs::builtin_interfaces::Time::Now();
            robot_pose.header.stamp = now;
            robot_vel.header.stamp = now;

            // publish TF map->base_link
            const auto tf_map_base = MakeMapToBaseLinkTf(robot_pose);
            tf->setTransform(tf_map_base, "demo", /*is_static=*/false);

            commsgs::geometry_msgs::TwistStamped cmd;
            std::string message;
            try {
                controller.ComputeVelocityCommands(robot_pose, robot_vel, cmd, /*goal_checker=*/nullptr, message);
                // 发布速度指令到 cmd_vel（frame_id=base_link）
                cmd_writer->Write(std::make_shared<commsgs::proto::geometry_msgs::TwistStamped>(commsgs::geometry_msgs::ToProto(cmd)));
            } catch (const std::exception& e) {
                AERROR << "ComputeVelocityCommands exception: " << e.what();
                failed = true;
                break;
            }

            odom_writer->Write(ToProtoOdom(robot_pose, cmd, "base_link"));
    
            // update sim
            UpdatePose(robot_pose, cmd, CONTROL_DT);
            robot_vel.twist = cmd.twist;

            // logs
            if ((step % 10) == 0) {
                std::stringstream ss;
                ss << "cycle=" << cycle << " step=" << std::setw(4) << step
                   << " pose=(" << std::fixed << std::setprecision(2) << robot_pose.pose.position.x << ", "
                   << robot_pose.pose.position.y << ", yaw=" << GetYaw(robot_pose.pose.orientation) << ")"
                   << " cmd=(v=" << cmd.twist.linear.x << ", w=" << cmd.twist.angular.z << ")";
                AINFO << ss.str();
            }

            if ((step % map_publish_period_steps) == 0) {
                map_writer->Write(ToProtoOccupancyGrid(*map_costmap, "map"));
            }

            if (controller.IsGoalReached(/*dist_tol=*/0.35, /*angle_tol=*/0.60)) {
                reached_goal = true;
                AINFO << "Reached goal (IsGoalReached) in cycle #" << cycle << ", step=" << step;
                break;
            }

            std::this_thread::sleep_for(std::chrono::duration<double>(CONTROL_DT));
        }

        if (g_stop_requested.load()) {
            AINFO << "Stop requested (Ctrl+C).";
            break;
        }

        if (reached_goal) {
            AINFO << "CYCLE RESULT: SUCCESS -> replan next.";
        } else if (failed) {
            AINFO << "CYCLE RESULT: FAILED -> replan next.";
        } else {
            AINFO << "CYCLE RESULT: TIMEOUT -> replan next.";
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    AINFO << "Demo exiting.";
    google::FlushLogFiles(google::GLOG_INFO);
    std::_Exit(0);
}

}  // namespace

int main(int argc, char** argv) {
    // 避免退出阶段 atexit 链路导致段错误（与其他 demo 保持一致）
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
        autolink::Init("demo_navigation_to_pose");
    }

    return Run(argc, argv);
}

