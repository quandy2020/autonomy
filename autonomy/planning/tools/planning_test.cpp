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

/**
 * Offline planner benchmark/visualization tool.
 *
 * Loads config/planner/planner.lua, runs every configured planner plugin on a
 * synthetic (or YAML) map, saves PNG snapshots and MP4 videos under --output_dir.
 *
 * Example:
 *   autonomy_planning_test \
 *     --configuration_directory=/workspace/autonomy/src/autonomy/config \
 *     --output_dir=/tmp/planning_vis \
 *     --start_x=30 --start_y=30 --goal_x=370 --goal_y=370 \
 *     --fps=10 --hold_frames=20
 *
 * Default: planner.lua costmap plugins + config/data/map.yaml (map.pgm).
 * Use --use_synthetic_map for plugins=none and a small synthetic grid.
 */

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/common/gflags.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/utils/pgm_converter.hpp"
#include "autonomy/planning/common/planner_exceptions.hpp"
#include "autonomy/planning/planner_options.hpp"
#include "autonomy/planning/planner_server.hpp"

DEFINE_string(output_dir, "/tmp/autonomy_planning_vis",
              "Directory for PNG and MP4 outputs.");
DEFINE_string(map_yaml, "",
              "Map YAML path. Empty loads config/data/map.yaml (map.pgm) "
              "via configuration_directory search paths.");
DEFINE_bool(use_synthetic_map, false,
            "Use empty synthetic costmap instead of config/data/map.yaml.");
DEFINE_double(start_x, 30.0, "Start pose x [m] (default for data/map.pgm).");
DEFINE_double(start_y, 30.0, "Start pose y [m].");
DEFINE_double(goal_x, 370.0, "Goal pose x [m].");
DEFINE_double(goal_y, 370.0, "Goal pose y [m].");
DEFINE_double(fps, 10.0, "Video frame rate.");
DEFINE_int32(hold_frames, 20,
             "Frames to hold the final path per planner segment.");
DEFINE_int32(animate_poses_per_frame, 2,
             "Path poses added per animation frame (>=1).");
DEFINE_bool(per_planner_video, true,
            "Write one MP4 per planner in addition to combined video.");
DEFINE_string(combined_video_name, "planning_all_planners.mp4",
              "Combined video filename under output_dir.");
DEFINE_bool(add_demo_obstacles, true,
            "Add a central obstacle patch on synthetic maps.");

namespace autonomy {
namespace planning {
namespace tools {
namespace {

namespace fs = std::filesystem;

std::string ResolveMapYamlPath(const std::string& configuration_directory,
                               const std::string& map_yaml_override) {
    if (!map_yaml_override.empty()) {
        return map_yaml_override;
    }
    const auto dirs = ::autonomy::common::ConfigurationSearchDirectories(
        configuration_directory);
    ::autonomy::common::ConfigurationFileResolver resolver(dirs);
    return resolver.GetFullPathOrDie("data/map.yaml");
}

proto::PlannerOptions LoadPlannerOptionsFromLua(
    const std::string& configuration_directory) {
    const auto dirs = ::autonomy::common::ConfigurationSearchDirectories(
        configuration_directory);
    auto file_resolver =
        std::make_unique<::autonomy::common::ConfigurationFileResolver>(dirs);
    const std::string code = ::autonomy::common::GetLuaScriptWithCommonOrDie(
        *file_resolver, "planner/planner.lua");
    ::autonomy::common::LuaParameterDictionary lua_dictionary(
        code, std::move(file_resolver));
    return LoadOptions(lua_dictionary.GetDictionary("AUTONOMY_PLANNER").get());
}

proto::PlannerOptions PrepareOfflineOptions(proto::PlannerOptions options,
                                            bool use_synthetic_map) {
    options.set_auto_smooth_after_plan(false);
    if (use_synthetic_map) {
        options.mutable_costmap()->clear_plugins();
        options.mutable_costmap()->add_plugins("none");
    }
    return options;
}

void LogOfflineCostmapMode(const proto::PlannerOptions& options) {
    std::string plugins;
    for (int i = 0; i < options.costmap().plugins_size(); ++i) {
        if (!plugins.empty()) {
            plugins += ", ";
        }
        plugins += options.costmap().plugins(i);
    }
    LOG(INFO) << "Costmap layer plugins from planner.lua: [" << plugins << "]";
}

void FinalizeOfflineCostmap(PlannerServer& server) {
    server.ReconfigurePlugins();
    auto wrapper = server.GetCostmapWrapper();
    if (wrapper == nullptr) {
        return;
    }
    wrapper->updateMap();
    wrapper->Stop();
    LOG(INFO) << "Applied costmap layer pipeline and stopped update thread";
}

commsgs::geometry_msgs::PoseStamped MakePose(double x, double y,
                                             const std::string& frame_id) {
    commsgs::geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    return pose;
}

void SetupSyntheticCostmap(map::costmap_2d::Costmap2D* costmap,
                           bool add_obstacles) {
    if (costmap == nullptr) {
        return;
    }
    costmap->resetMapToValue(0, 0, costmap->getSizeInCellsX(),
                             costmap->getSizeInCellsY(),
                             map::costmap_2d::FREE_SPACE);
    if (!add_obstacles) {
        return;
    }
    const unsigned int w = costmap->getSizeInCellsX();
    const unsigned int h = costmap->getSizeInCellsY();
    const unsigned int x0 = w / 3;
    const unsigned int x1 = 2 * w / 3;
    const unsigned int y0 = h / 3;
    const unsigned int y1 = 2 * h / 3;
    for (unsigned int y = y0; y < y1; ++y) {
        for (unsigned int x = x0; x < x1; ++x) {
            costmap->setCost(x, y, map::costmap_2d::LETHAL_OBSTACLE);
        }
    }
}

commsgs::planning_msgs::Path TruncatePath(
    const commsgs::planning_msgs::Path& path, size_t pose_count) {
    commsgs::planning_msgs::Path partial = path;
    partial.poses.clear();
    const size_t n = std::min(pose_count, path.poses.size());
    partial.poses.insert(partial.poses.end(), path.poses.begin(),
                         path.poses.begin() + static_cast<std::ptrdiff_t>(n));
    return partial;
}

std::string SanitizeFilename(const std::string& name) {
    std::string out = name;
    for (char& c : out) {
        if (c == '/' || c == ':' || c == ' ') {
            c = '_';
        }
    }
    return out;
}

cv::Mat RenderPathFrame(const map::costmap_2d::Costmap2D& costmap,
                        const commsgs::planning_msgs::Path& path,
                        const std::string& label, const std::string& tmp_png) {
    map::utils::PgmConverter::RenderParameters render;
    render.draw_start_marker = true;
    render.draw_goal_marker = true;
    render.path_line_width = 2.0;
    if (!map::utils::PgmConverter::savePathToImage(costmap, path, tmp_png,
                                                   render)) {
        return {};
    }
    cv::Mat frame = cv::imread(tmp_png);
    if (frame.empty()) {
        return frame;
    }
    cv::putText(frame, label, cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    if (path.poses.empty()) {
        cv::putText(frame, "PLAN FAILED", cv::Point(12, 56),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2,
                    cv::LINE_AA);
    }
    return frame;
}

bool WriteVideo(const std::string& path, const std::vector<cv::Mat>& frames,
                double fps) {
    if (frames.empty()) {
        return false;
    }
    const cv::Size size = frames.front().size();
    const int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
    cv::VideoWriter writer(path, fourcc, fps, size, true);
    if (!writer.isOpened()) {
        LOG(ERROR) << "Failed to open VideoWriter: " << path;
        return false;
    }
    for (const auto& frame : frames) {
        if (frame.size() != size) {
            cv::Mat resized;
            cv::resize(frame, resized, size);
            writer.write(resized);
        } else {
            writer.write(frame);
        }
    }
    writer.release();
    return true;
}

std::vector<cv::Mat> BuildPlannerAnimation(
    const map::costmap_2d::Costmap2D& costmap,
    const commsgs::planning_msgs::Path& path, const std::string& planner_id,
    const fs::path& scratch_dir, int animate_step, int hold_frames) {
    std::vector<cv::Mat> frames;
    const std::string label = "planner: " + planner_id;
    const fs::path tmp_png = scratch_dir / (SanitizeFilename(planner_id) + ".png");

    if (path.poses.size() < 2) {
        frames.push_back(RenderPathFrame(costmap, path, label, tmp_png.string()));
        for (int i = 0; i < hold_frames; ++i) {
            frames.push_back(frames.back().clone());
        }
        return frames;
    }

    const int step = std::max(1, animate_step);
    for (size_t count = 2; count <= path.poses.size(); count += step) {
        const auto partial = TruncatePath(path, count);
        frames.push_back(
            RenderPathFrame(costmap, partial, label, tmp_png.string()));
    }
    if (path.poses.size() % step != 0 || path.poses.size() == 2) {
        frames.push_back(RenderPathFrame(costmap, path, label, tmp_png.string()));
    }
    const cv::Mat final_frame = frames.back().clone();
    for (int i = 0; i < hold_frames; ++i) {
        frames.push_back(final_frame.clone());
    }
    return frames;
}

struct PlannerRunResult {
    std::string planner_id;
    bool success{false};
    commsgs::planning_msgs::Path path;
    std::string error;
};

PlannerRunResult RunPlanner(PlannerServer& server,
                            const commsgs::geometry_msgs::PoseStamped& start,
                            const commsgs::geometry_msgs::PoseStamped& goal,
                            const std::string& planner_id) {
    PlannerRunResult result;
    result.planner_id = planner_id;
    try {
        result.path =
            server.ComputePathToPose(start, goal, planner_id, []() { return false; });
        result.success = !result.path.poses.empty();
        if (!result.success) {
            result.error = "empty path";
        }
    } catch (const common::PlannerException& e) {
        result.error = e.what();
    } catch (const std::exception& e) {
        result.error = e.what();
    }
    return result;
}

}  // namespace

int RunPlanningVisualization(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    const fs::path output_root(FLAGS_output_dir);
    const fs::path scratch_dir = output_root / "frames";
    fs::create_directories(scratch_dir);

    proto::PlannerOptions options = PrepareOfflineOptions(
        LoadPlannerOptionsFromLua(
            ::autonomy::common::FLAGS_configuration_directory),
        FLAGS_use_synthetic_map);
    LogOfflineCostmapMode(options);

    PlannerServer server(options);

    auto* costmap_wrapper = server.GetCostmapWrapper().get();
    auto* costmap = costmap_wrapper != nullptr ? costmap_wrapper->getCostmap()
                                               : nullptr;
    if (costmap == nullptr) {
        LOG(FATAL) << "Costmap is null after PlannerServer construction";
        return 1;
    }

    if (FLAGS_use_synthetic_map) {
        SetupSyntheticCostmap(costmap, FLAGS_add_demo_obstacles);
        LOG(INFO) << "Using synthetic costmap with demo obstacles="
                  << FLAGS_add_demo_obstacles;
    } else {
        std::string map_yaml_path;
        try {
            map_yaml_path = ResolveMapYamlPath(
                ::autonomy::common::FLAGS_configuration_directory,
                FLAGS_map_yaml);
        } catch (const std::exception& e) {
            LOG(FATAL) << "Failed to resolve map YAML (data/map.yaml): "
                       << e.what();
            return 1;
        }
        if (!costmap_wrapper->loadMap(map_yaml_path)) {
            LOG(FATAL) << "Failed to load map into costmap layers: "
                       << map_yaml_path;
            return 1;
        }
        LOG(INFO) << "Loaded map via costmap layers (static/denoise/inflation) "
                  << "from " << map_yaml_path;
    }

    server.Start();
    FinalizeOfflineCostmap(server);

    const std::string frame_id = options.costmap().frame_id().empty()
                                     ? "map"
                                     : options.costmap().frame_id();
    const auto start = MakePose(FLAGS_start_x, FLAGS_start_y, frame_id);
    const auto goal = MakePose(FLAGS_goal_x, FLAGS_goal_y, frame_id);

    std::vector<std::string> planner_ids;
    if (options.planner_plugins_size() > 0) {
        for (const auto& id : options.planner_plugins()) {
            planner_ids.push_back(id);
        }
    } else {
        planner_ids = {"navfn_planner", "dijkstra_planner", "theta_star_planner"};
    }

    std::vector<cv::Mat> combined_frames;
    int success_count = 0;

    for (const auto& planner_id : planner_ids) {
        LOG(INFO) << "Running planner: " << planner_id;
        const PlannerRunResult run =
            RunPlanner(server, start, goal, planner_id);

        if (run.success) {
            ++success_count;
            const std::string png_path =
                (output_root / (SanitizeFilename(planner_id) + ".png")).string();
            map::utils::PgmConverter::savePathToImage(*costmap, run.path, png_path);
            LOG(INFO) << "Saved snapshot: " << png_path;
        } else {
            LOG(WARNING) << "Planner " << planner_id
                         << " failed: " << run.error;
        }

        const auto animation = BuildPlannerAnimation(
            *costmap, run.path, planner_id, scratch_dir,
            FLAGS_animate_poses_per_frame, FLAGS_hold_frames);

        if (FLAGS_per_planner_video && !animation.empty()) {
            const std::string video_path =
                (output_root / (SanitizeFilename(planner_id) + ".mp4")).string();
            if (WriteVideo(video_path, animation, FLAGS_fps)) {
                LOG(INFO) << "Saved video: " << video_path;
            }
        }

        combined_frames.insert(combined_frames.end(), animation.begin(),
                               animation.end());
    }

    if (!combined_frames.empty()) {
        const std::string combined_path =
            (output_root / FLAGS_combined_video_name).string();
        if (WriteVideo(combined_path, combined_frames, FLAGS_fps)) {
            LOG(INFO) << "Saved combined video: " << combined_path;
        }
    }

    LOG(INFO) << "Finished: " << success_count << "/" << planner_ids.size()
              << " planners succeeded. Output: " << output_root;
    return success_count > 0 ? 0 : 2;
}

}  // namespace tools
}  // namespace planning
}  // namespace autonomy

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    return autonomy::planning::tools::RunPlanningVisualization(argc, argv);
}
