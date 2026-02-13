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
 * NavFn Planner demo application.
 *
 * 参考 navfn_planner_test.cpp 中的 RunPathPlanning 独立模式实现，
 * 提供一个命令行工具，通过 NavfnPlanner 在栅格地图上规划路径并将结果渲染到图片。
 *
 * 用法示例（在容器内）：
 *   # 使用默认地图（turtlebot3_house.pgm），默认起终点，输出到 /tmp/navfn_path_result.png
 *   ./autonomy.examples.apps.demo_navfn_planner
 *
 *   # 使用默认地图，指定输出文件和起终点
 *   ./autonomy.examples.apps.demo_navfn_planner result.png 0.0 0.0 5.0 5.0
 *
 *   # 使用自定义地图（PGM/PNG），指定输出文件与起终点
 *   ./autonomy.examples.apps.demo_navfn_planner map.pgm result.png 2.0 2.0 8.0 8.0 0.05
 */

#include <cmath>
#include <cctype>
#include <algorithm>
#include <atomic>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <unistd.h>

#include "autolink/autolink.hpp"
#include "autolink/common/log.hpp"
#include "autolink/message/message_traits.hpp"
#include "autolink/proto/role_attributes.pb.h"
#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/proto/geometry_msgs.pb.h"
#include "autonomy/commsgs/proto/map_msgs.pb.h"
#include "autonomy/commsgs/proto/planning_msgs.pb.h"
#include "autonomy/commsgs/std_msgs.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/map/costmap_2d/utils/occ_grid_values.hpp"
#include "autonomy/map/proto/map_2d_option.pb.h"
#include "autonomy/map/utils/data_loader_utils.hpp"
#include "autonomy/planning/planner/navfn/navfn_planner.hpp"
#include "autonomy/planning/proto/planning_options.pb.h"
#include "autonomy/planning/utils/pgm_converter.hpp"

using autonomy::commsgs::geometry_msgs::PoseStamped;
using autonomy::commsgs::planning_msgs::Path;

namespace {

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

// Helper function to create a pose in "map" frame
PoseStamped CreatePose(double x, double y, const std::string& frame_id = "map") {
    PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.header.stamp = autonomy::commsgs::builtin_interfaces::Time::Now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    return pose;
}

std::shared_ptr<autonomy::commsgs::proto::planning_msgs::Path> ToProtoPath(const Path& path) {
    auto msg = std::make_shared<autonomy::commsgs::proto::planning_msgs::Path>();
    *msg->mutable_header() = autonomy::commsgs::std_msgs::ToProto(path.header);
    msg->mutable_poses()->Reserve(static_cast<int>(path.poses.size()));
    for (const auto& pose : path.poses) {
        *msg->add_poses() = autonomy::commsgs::geometry_msgs::ToProto(pose);
    }
    return msg;
}

std::shared_ptr<autonomy::commsgs::proto::geometry_msgs::PoseArray> MakeStartGoalPoseArray(const PoseStamped& start,
                                                                                          const PoseStamped& goal) {
    auto arr = std::make_shared<autonomy::commsgs::proto::geometry_msgs::PoseArray>();
    *arr->mutable_header() = autonomy::commsgs::std_msgs::ToProto(start.header);
    *arr->add_poses() = autonomy::commsgs::geometry_msgs::ToProto(start.pose);
    *arr->add_poses() = autonomy::commsgs::geometry_msgs::ToProto(goal.pose);
    return arr;
}

autonomy::commsgs::proto::std_msgs::Header CreateHeaderProto(const std::string& frame_id) {
    autonomy::commsgs::std_msgs::Header header;
    header.frame_id = frame_id;
    header.stamp = autonomy::commsgs::builtin_interfaces::Time::Now();
    return autonomy::commsgs::std_msgs::ToProto(header);
}

std::shared_ptr<autonomy::commsgs::proto::map_msgs::OccupancyGrid> ToProtoOccupancyGrid(
    const autonomy::map::costmap_2d::Costmap2D& costmap, const std::string& frame_id) {
    auto msg = std::make_shared<autonomy::commsgs::proto::map_msgs::OccupancyGrid>();
    *msg->mutable_header() = CreateHeaderProto(frame_id);

    auto* info = msg->mutable_info();
    info->set_width(costmap.getSizeInCellsX());
    info->set_height(costmap.getSizeInCellsY());
    info->set_resolution(static_cast<float>(costmap.getResolution()));
    *info->mutable_map_load_time() =
        autonomy::commsgs::builtin_interfaces::ToProto(autonomy::commsgs::builtin_interfaces::Time::Now());

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

struct ParsedArgs {
    bool flag_mode = false;
    // If non-empty, user explicitly specified map (flag mode)
    std::string map_arg;
    std::string output_arg;
    bool has_resolution = false;
    double resolution = 0.05;
    bool has_origin = false;
    double origin_x = 0.0;
    double origin_y = 0.0;
    double origin_yaw = 0.0;
};

ParsedArgs ParseArgs(int argc, char* argv[]) {
    ParsedArgs args;
    // defaults
    args.output_arg = "/tmp/navfn_path_result.png";
    args.resolution = 0.05;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i] ? argv[i] : "";
        auto take_next = [&](std::string* out) -> bool {
            if (i + 1 >= argc) {
                return false;
            }
            *out = argv[++i];
            return true;
        };
        auto take_next_double = [&](double* out) -> bool {
            if (i + 1 >= argc) {
                return false;
            }
            *out = std::stod(argv[++i]);
            return true;
        };

        if (a == "--help" || a == "-h") {
            args.flag_mode = true;
            std::cout << "Usage:\n"
                      << "  " << argv[0] << " [output.png] [resolution]\n"
                      << "  " << argv[0] << " <map.(pgm|yaml)> <output.png> [resolution]\n"
                      << "  " << argv[0] << " --map <name|path.(pgm|yaml)> [--output <out.png>]\n"
                      << "        [--resolution <m_per_cell>] [--origin_x <m>] [--origin_y <m>] [--origin_yaw <rad>]\n";
            std::_Exit(0);
        } else if (a == "--map") {
            args.flag_mode = true;
            (void)take_next(&args.map_arg);
        } else if (a == "--output") {
            args.flag_mode = true;
            (void)take_next(&args.output_arg);
        } else if (a == "--resolution") {
            args.flag_mode = true;
            args.has_resolution = take_next_double(&args.resolution);
        } else if (a == "--origin_x") {
            args.flag_mode = true;
            args.has_origin = take_next_double(&args.origin_x);
        } else if (a == "--origin_y") {
            args.flag_mode = true;
            args.has_origin = take_next_double(&args.origin_y);
        } else if (a == "--origin_yaw") {
            args.flag_mode = true;
            args.has_origin = take_next_double(&args.origin_yaw);
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

    // If empty -> default map base name
    std::string input = map_arg.empty() ? "turtlebot3_house" : map_arg;

    // If it's a bare name (no slash and no extension), resolve inside default map dir.
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
        return {MapSelection::Kind::kPgm, base.string()};  // will fail later with clearer log
    }

    // Otherwise treat it as a path.
    fs::path p(input);
    std::string ext = p.extension().string();
    for (auto& ch : ext) {
        ch = static_cast<char>(std::tolower(ch));
    }

    // If user passed pgm, prefer sibling yaml if exists (metadata better).
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

    // Unknown extension: try yaml then pgm with same base
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

int RunPathPlanning(int argc, char* argv[]) {
    using namespace autonomy::planning::planner::navfn;
    using namespace autonomy::planning::utils;
    using namespace autonomy::map::utils;

    // 初始化 autolink（用于可视化发布）
    ::setenv("AUTOLINK_DISABLE_ATEXIT", "1", /*overwrite=*/0);
    if (std::getenv("AUTOLINK_PATH") == nullptr) {
        const auto guessed = GuessAutolinkWorkRoot();
        if (!guessed.empty()) {
            ::setenv("AUTOLINK_PATH", guessed.c_str(), /*overwrite=*/0);
        }
    }
    autolink::Init(argv[0]);

    const auto parsed = ParseArgs(argc, argv);
    const std::string default_map_dir = GetMapDataFilesDirectory();

    std::string output_image = parsed.output_arg;
    std::string map_arg = parsed.map_arg;

    // 兼容旧的 positional 形式：
    //   demo_navfn_planner [output.png] [resolution]
    //   demo_navfn_planner <map.(pgm|yaml)> <output.png> [resolution]
    if (!parsed.flag_mode && parsed.map_arg.empty() && argc >= 2) {
        const std::string a1 = argv[1];
        std::error_code ec;
        const bool exists = std::filesystem::exists(a1, ec) && !ec;
        const bool looks_like_map = (a1.find(".pgm") != std::string::npos || a1.find(".yaml") != std::string::npos ||
                                     a1.find(".yml") != std::string::npos);
        if (exists && looks_like_map) {
            map_arg = a1;
            if (argc >= 3) {
                output_image = argv[2];
            }
            if (argc >= 4) {
                // optional resolution override for pgm-only maps
                // (yaml map ignores this)
                // NOLINTNEXTLINE(runtime/references)
                const std::string ext = std::filesystem::path(map_arg).extension().string();
                if (ext == ".pgm" || ext == ".png") {
                    // allow: demo_navfn_planner map.pgm out.png 0.05
                    // (keep behavior simple and explicit)
                }
            }
        } else {
            output_image = a1;
        }
    }

    // positional resolution override: demo_navfn_planner out.png 0.05
    // or: demo_navfn_planner map.pgm out.png 0.05
    double resolution_override = parsed.resolution;
    bool has_resolution_override = parsed.has_resolution;
    if (!parsed.flag_mode && !has_resolution_override) {
        if (map_arg.empty() && argc >= 3) {
            // argv[1]=output, argv[2]=resolution
            resolution_override = std::stod(argv[2]);
            has_resolution_override = true;
        } else if (!map_arg.empty() && argc >= 4) {
            // argv[1]=map, argv[2]=output, argv[3]=resolution
            resolution_override = std::stod(argv[3]);
            has_resolution_override = true;
        }
    }

    AINFO << "=== NavFn Path Planning Demo (1s replan, random start/goal) ===";
    AINFO << "Map arg: " << (map_arg.empty() ? "<default>" : map_arg);
    AINFO << "Output path prefix: " << output_image;
    if (has_resolution_override) {
        AINFO << "Resolution override: " << resolution_override << " m/pixel";
    }

    try {
        // 从图像加载 costmap
        const auto selection = ResolveMapSelection(map_arg, default_map_dir);
        AINFO << "Loading map from: " << selection.path
              << (selection.kind == MapSelection::Kind::kYaml ? " (yaml)" : " (pgm)");

        autonomy::map::costmap_2d::Costmap2D::SharedPtr costmap;
        if (selection.kind == MapSelection::Kind::kYaml) {
            costmap = PgmConverter::loadFromYaml(selection.path);
        } else {
            PgmConverter::LoadParameters load_params;
            if (has_resolution_override) {
                load_params.resolution = resolution_override;
            }
            if (parsed.has_origin) {
                load_params.origin_x = parsed.origin_x;
                load_params.origin_y = parsed.origin_y;
                load_params.origin_yaw = parsed.origin_yaw;
            }
            costmap = PgmConverter::loadFromPgm(selection.path, load_params);
        }

        if (!costmap) {
            AERROR << "Failed to load map from: " << selection.path;
            return 1;
        }

        AINFO << "Map loaded: " << costmap->getSizeInCellsX() << "x" << costmap->getSizeInCellsY()
              << " cells, resolution: " << costmap->getResolution() << " m/pixel";

        // 创建 autolink node 与 writers（用于 Foxglove 可视化）
        auto node = autolink::CreateNode("demo_navfn_planner_" + std::to_string(getpid()));
        if (!node) {
            AERROR << "Failed to create autolink node";
            return 1;
        }
        using autolink::message::MessageType;
        using autolink::proto::RoleAttributes;

        RoleAttributes path_attr;
        path_attr.set_channel_name("/demo/navfn/path");
        path_attr.set_message_type(MessageType<autonomy::commsgs::proto::planning_msgs::Path>());
        auto path_writer = node->CreateWriter<autonomy::commsgs::proto::planning_msgs::Path>(path_attr);

        RoleAttributes start_goal_attr;
        start_goal_attr.set_channel_name("/demo/navfn/start_goal");
        start_goal_attr.set_message_type(MessageType<autonomy::commsgs::proto::geometry_msgs::PoseArray>());
        auto start_goal_writer =
            node->CreateWriter<autonomy::commsgs::proto::geometry_msgs::PoseArray>(start_goal_attr);

        RoleAttributes map_attr;
        map_attr.set_channel_name("/demo/navfn/map");
        map_attr.set_message_type(MessageType<autonomy::commsgs::proto::map_msgs::OccupancyGrid>());
        auto map_writer = node->CreateWriter<autonomy::commsgs::proto::map_msgs::OccupancyGrid>(map_attr);

        if (!path_writer || !start_goal_writer || !map_writer) {
            AERROR << "Failed to create writers for visualization";
            return 1;
        }

        // 构造 Costmap2DOptions 与 Costmap2DWrapper
        autonomy::map::proto::Costmap2DOptions costmap_options;
        costmap_options.set_enabled(true);
        costmap_options.set_resolution(costmap->getResolution());
        costmap_options.set_frame_id("map");
        costmap_options.set_name("navfn_demo_costmap");
        // demo 只需要静态 costmap 数据，禁用 layer 插件加载，避免额外依赖
        costmap_options.add_plugins("noop_layer");

        auto costmap_wrapper =
            std::make_shared<autonomy::map::costmap_2d::Costmap2DWrapper>(costmap_options, "navfn_demo_costmap", nullptr);

        // 将加载到的 costmap 数据拷贝到 wrapper 的内部 costmap
        auto* wrapper_costmap = costmap_wrapper->getCostmap();
        if (wrapper_costmap) {
            unsigned int size_x = costmap->getSizeInCellsX();
            unsigned int size_y = costmap->getSizeInCellsY();
            wrapper_costmap->resizeMap(size_x, size_y, costmap->getResolution(), costmap->getOriginX(),
                                       costmap->getOriginY());

            for (unsigned int y = 0; y < size_y; ++y) {
                for (unsigned int x = 0; x < size_x; ++x) {
                    unsigned char c = costmap->getCost(x, y);
                    wrapper_costmap->setCost(x, y, c);
                }
            }

            AINFO << "Copied costmap data to wrapper: " << size_x << "x" << size_y;
        }

        // 发布地图（重复发布，确保 AutoDiscovery 扫描后能看到）
        const int map_publish_period_iters = 5;  // 约 0.2Hz（循环每秒一次）
        int map_publish_counter = 0;

        // 构造 NavFn PlannerOptions
        autonomy::planning::proto::PlannerOptions planner_options;
        auto* navfn = planner_options.mutable_navfn();
        navfn->set_tolerance(0.0);
        navfn->set_use_astar(false);
        navfn->set_allow_unknown(true);
        navfn->set_use_final_approach_orientation(false);

        // 创建并配置 NavfnPlanner
        NavfnPlanner planner;
        if (!planner.Configure(planner_options, "navfn_demo", costmap_wrapper)) {
            AERROR << "Failed to configure NavfnPlanner";
            return 1;
        }

        planner.Activate();
        AINFO << "OpenCV removed: only save results to file and publish via autolink.";

        // 随机数引擎，用于动态选取 start / goal
        std::mt19937 rng(std::random_device{}());

        // 每秒重新规划一次，直到 Ctrl+C 或程序被终止
        int iteration = 0;
        while (autolink::OK() && !g_stop_requested.load()) {
            ++iteration;

            // 动态选取地图中的两个可行走点作为起点和终点
            PoseStamped start_pose;
            PoseStamped goal_pose;

            auto* cm = wrapper_costmap;
            unsigned int size_x = cm->getSizeInCellsX();
            unsigned int size_y = cm->getSizeInCellsY();
            std::uniform_int_distribution<unsigned int> dist_x(0, size_x - 1);
            std::uniform_int_distribution<unsigned int> dist_y(0, size_y - 1);

            auto sample_free_pose = [&](PoseStamped& out_pose) -> bool {
                const int max_tries = 1000;
                for (int i = 0; i < max_tries; ++i) {
                    unsigned int mx = dist_x(rng);
                    unsigned int my = dist_y(rng);
                    unsigned char c = cm->getCost(mx, my);
                    // 仅接受非致命障碍和非未知区域
                    if (c < autonomy::map::costmap_2d::LETHAL_OBSTACLE &&
                        c != autonomy::map::costmap_2d::NO_INFORMATION) {
                        double wx, wy;
                        cm->mapToWorld(mx, my, wx, wy);
                        out_pose = CreatePose(wx, wy, "map");
                        return true;
                    }
                }
                return false;
            };

            if (!sample_free_pose(start_pose) || !sample_free_pose(goal_pose)) {
                AERROR << "Failed to sample free start/goal poses from costmap.";
                break;
            }

            AINFO << "Iteration " << iteration << " - Planning from (" << start_pose.pose.position.x << ", "
                  << start_pose.pose.position.y << ") to (" << goal_pose.pose.position.x << ", "
                  << goal_pose.pose.position.y << ")";

            // 调用 CreatePlan 生成路径
            Path plan;
            auto cancel_checker = []() { return false; };  // 不支持取消

            autonomy::uint32 result_code = planner.CreatePlan(start_pose, goal_pose, plan, cancel_checker);
            if (result_code != static_cast<autonomy::uint32>(
                                    autonomy::planning::proto::PlannerResultCode::PLANNER_SUCCESS)) {
                AERROR << "Path planning failed with result code: " << result_code;
            } else {
                AINFO << "  -> Generated path with " << plan.poses.size() << " poses";

                // 发布 start/goal + path（用于 Foxglove 可视化）
                // 确保 header 的 stamp 更新
                plan.header.frame_id = "map";
                plan.header.stamp = autonomy::commsgs::builtin_interfaces::Time::Now();
                path_writer->Write(ToProtoPath(plan));
                start_goal_writer->Write(MakeStartGoalPoseArray(start_pose, goal_pose));
                if ((map_publish_counter++ % map_publish_period_iters) == 0) {
                    map_writer->Write(ToProtoOccupancyGrid(*costmap, "map"));
                }

                // 将路径渲染到图片（使用输出前缀加迭代计数）
                std::string iter_output = output_image;
                if (output_image.find('.') != std::string::npos) {
                    // 在扩展名前插入迭代编号
                    auto pos = output_image.find_last_of('.');
                    iter_output = output_image.substr(0, pos) + "_" + std::to_string(iteration) +
                                  output_image.substr(pos);
                } else {
                    iter_output = output_image + "_" + std::to_string(iteration) + ".png";
                }

                PgmConverter::RenderParameters render_params;
                render_params.draw_start_marker = true;
                render_params.draw_goal_marker = true;
                render_params.draw_path_points = false;
                render_params.path_line_width = 3.0;
                render_params.start_marker_size = 10.0;
                render_params.goal_marker_size = 10.0;

                bool saved = PgmConverter::savePathToImage(*costmap, plan, iter_output, render_params);
                if (!saved) {
                    AERROR << "Failed to save path to image file: " << iter_output;
                } else {
                    AINFO << "  -> Result saved to: " << iter_output;
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }
        }

        planner.Deactivate();
        planner.Cleanup();

        google::FlushLogFiles(google::GLOG_INFO);
        std::_Exit(0);
    } catch (const std::exception& e) {
        AERROR << "Error during path planning: " << e.what();
        return 1;
    }
}

}  // namespace

int main(int argc, char* argv[]) {
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);
    return RunPathPlanning(argc, argv);
}


