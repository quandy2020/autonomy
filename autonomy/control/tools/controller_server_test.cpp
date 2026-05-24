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
 * Offline controller benchmark/visualization tool.
 *
 * Loads config/control/controller.lua and config/planner/planner.lua, plans a
 * global path, runs each configured controller in closed-loop simulation, and
 * saves PNG snapshots plus MP4/GIF under --output_dir.
 *
 * Example:
 *   autonomy_controller_test \
 *     --configuration_directory=/workspace/autonomy/src/autonomy/config \
 *     --output_dir=/tmp/controller_vis \
 *     --start_x=30 --start_y=30 --goal_x=370 --goal_y=370 \
 *     --fps=10 --output_format=both
 */

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#ifndef _WIN32
#include <sys/wait.h>
#include <unistd.h>
#endif

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/common/gflags.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/common/math/math_utils.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/control/control_options.hpp"
#include "autonomy/control/controller_factory.hpp"
#include "autonomy/control/controller_server.hpp"
#include "autonomy/control/utils/odometry_utils.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/utils/pgm_converter.hpp"
#include "autonomy/planning/common/planner_exceptions.hpp"
#include "autonomy/planning/planner_options.hpp"
#include "autonomy/planning/planner_server.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/geometry_msgs/transform_stamped.h"
#include "autonomy/transform/tf2/utils.h"

DEFINE_string(output_dir, "/tmp/autonomy_controller_vis",
              "Directory for PNG, MP4, and GIF outputs.");
DEFINE_string(output_format, "mp4",
              "Output animation format: mp4, gif, or both.");
DEFINE_string(map_yaml, "",
              "Map YAML path. Empty loads config/data/map.yaml.");
DEFINE_bool(use_synthetic_map, false,
            "Use empty synthetic costmap instead of config/data/map.yaml.");
DEFINE_string(planner_id, "",
              "Planner plugin id. Empty uses planner.lua default.");
DEFINE_string(controllers, "",
              "Comma-separated controller ids. Empty auto-detects from "
              "controller.lua blocks.");
DEFINE_double(start_x, 30.0, "Start pose x [m].");
DEFINE_double(start_y, 30.0, "Start pose y [m].");
DEFINE_double(goal_x, 370.0, "Goal pose x [m].");
DEFINE_double(goal_y, 370.0, "Goal pose y [m].");
DEFINE_double(fps, 10.0, "Video/GIF frame rate.");
DEFINE_int32(hold_frames, 15,
             "Frames to hold the final trajectory per controller.");
DEFINE_int32(animate_poses_per_frame, 3,
             "Trajectory poses added per animation frame (>=1).");
DEFINE_int32(max_animation_frames, 400,
             "Cap animation frames per controller (larger step when exceeded).");
DEFINE_int32(max_sim_steps, 800,
             "Minimum control ticks per controller; auto-scales with path "
             "length when reference path is longer.");
DEFINE_bool(per_controller_video, true,
            "Write one MP4/GIF per controller.");
DEFINE_string(combined_video_name, "controller_all.mp4",
              "Combined MP4 filename under output_dir.");
DEFINE_string(combined_gif_name, "controller_all.gif",
              "Combined GIF filename under output_dir.");
DEFINE_bool(save_reference_png, true,
            "Save planner reference path PNG per controller.");
DEFINE_bool(add_demo_obstacles, true,
            "Add a central obstacle patch on synthetic maps.");
DEFINE_bool(skip_mppi, false,
            "Skip mppi_controller (large batch_size; slow offline runs).");
DEFINE_bool(write_run_summary, true,
            "Write run_summary.txt under output_dir.");
DEFINE_double(path_pose_spacing, 0.05,
              "Resample planner path so consecutive poses are at most this "
              "many meters apart (0 disables).");
DEFINE_int32(min_reference_path_poses, 10,
             "Warn if planner path has fewer poses after post-process.");

namespace autonomy {
namespace control {
namespace tools {
namespace {

namespace fs = std::filesystem;
using Time = commsgs::builtin_interfaces::Time;

constexpr const char* kGlobalFrame = "map";
constexpr const char* kRobotFrame = "base_link";

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

planning::proto::PlannerOptions LoadPlannerOptionsFromLua(
    const std::string& configuration_directory) {
    const auto dirs = ::autonomy::common::ConfigurationSearchDirectories(
        configuration_directory);
    auto file_resolver =
        std::make_unique<::autonomy::common::ConfigurationFileResolver>(dirs);
    const std::string code = ::autonomy::common::GetLuaScriptWithCommonOrDie(
        *file_resolver, "planner/planner.lua");
    ::autonomy::common::LuaParameterDictionary lua_dictionary(
        code, std::move(file_resolver));
    return planning::LoadOptions(
        lua_dictionary.GetDictionary("AUTONOMY_PLANNER").get());
}

proto::ControllerOptions LoadControllerOptionsFromLua(
    const std::string& configuration_directory) {
    const auto dirs = ::autonomy::common::ConfigurationSearchDirectories(
        configuration_directory);
    auto file_resolver =
        std::make_unique<::autonomy::common::ConfigurationFileResolver>(dirs);
    const std::string code = ::autonomy::common::GetLuaScriptWithCommonOrDie(
        *file_resolver, "control/controller.lua");
    ::autonomy::common::LuaParameterDictionary lua_dictionary(
        code, std::move(file_resolver));
    return LoadOptions(
        lua_dictionary.GetDictionary("AUTONOMY_CONTROLLER").get());
}

planning::proto::PlannerOptions PrepareOfflinePlannerOptions(
    planning::proto::PlannerOptions options, bool use_synthetic_map) {
    // Douglas-Peucker (planner.lua path_simplify_epsilon) collapses grid paths
    // to start+goal only; controllers need a dense polyline.
    options.set_auto_smooth_after_plan(false);
    options.set_path_simplify_epsilon(0.0);
    if (use_synthetic_map) {
        options.mutable_costmap()->clear_plugins();
        options.mutable_costmap()->add_plugins("none");
    }
    return options;
}

void LogMapWorldBounds(const map::costmap_2d::Costmap2D& costmap,
                       const std::string& map_label) {
    const double ox = costmap.getOriginX();
    const double oy = costmap.getOriginY();
    const double max_x = ox + costmap.getSizeInMetersX();
    const double max_y = oy + costmap.getSizeInMetersY();
    LOG(INFO) << "Map world bounds (" << map_label << "): x=[" << ox << ", "
              << max_x << "] y=[" << oy << ", " << max_y
              << "] resolution=" << costmap.getResolution() << " m/cell ("
              << costmap.getSizeInCellsX() << "x" << costmap.getSizeInCellsY()
              << " cells)";
}

bool PoseInsideMap(const map::costmap_2d::Costmap2D& costmap, double x,
                   double y, const char* label) {
    const double ox = costmap.getOriginX();
    const double oy = costmap.getOriginY();
    const double max_x = ox + costmap.getSizeInMetersX();
    const double max_y = oy + costmap.getSizeInMetersY();
    if (x < ox || x > max_x || y < oy || y > max_y) {
        LOG(ERROR) << label << " (" << x << ", " << y
                   << ") is outside map bounds x=[" << ox << ", " << max_x
                   << "] y=[" << oy << ", " << max_y
                   << "]. Check map.yaml resolution and --start_* / --goal_*.";
        return false;
    }
    return true;
}

void FinalizeOfflineCostmap(planning::PlannerServer& server) {
    server.ReconfigurePlugins();
    auto wrapper = server.GetCostmapWrapper();
    if (wrapper == nullptr) {
        return;
    }
    wrapper->updateMap();
    wrapper->Stop();
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

commsgs::geometry_msgs::PoseStamped PoseFromState(
    double x, double y, double yaw, const std::string& frame_id) {
    commsgs::geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.header.stamp = Time::Now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    const double half_yaw = yaw * 0.5;
    pose.pose.orientation.z = std::sin(half_yaw);
    pose.pose.orientation.w = std::cos(half_yaw);
    return pose;
}

commsgs::planning_msgs::Path DensifyPath(const commsgs::planning_msgs::Path& path,
                                          double max_spacing) {
    if (path.poses.size() < 2 || max_spacing <= 0.0) {
        return path;
    }

    commsgs::planning_msgs::Path dense;
    dense.header = path.header;
    dense.poses.reserve(path.poses.size() * 8);
    dense.poses.push_back(path.poses.front());

    const std::string frame_id = path.poses.front().header.frame_id.empty()
                                     ? kGlobalFrame
                                     : path.poses.front().header.frame_id;

    for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
        const double x0 = path.poses[i].pose.position.x;
        const double y0 = path.poses[i].pose.position.y;
        const double x1 = path.poses[i + 1].pose.position.x;
        const double y1 = path.poses[i + 1].pose.position.y;
        const double dx = x1 - x0;
        const double dy = y1 - y0;
        const double dist = std::hypot(dx, dy);
        const int segments =
            std::max(1, static_cast<int>(std::ceil(dist / max_spacing)));
        const double yaw = std::atan2(dy, dx);

        for (int s = 1; s < segments; ++s) {
            const double t = static_cast<double>(s) / segments;
            dense.poses.push_back(
                PoseFromState(x0 + t * dx, y0 + t * dy, yaw, frame_id));
        }
    }

    const auto& goal = path.poses.back();
    const double gx = goal.pose.position.x;
    const double gy = goal.pose.position.y;
    const double lx = dense.poses.back().pose.position.x;
    const double ly = dense.poses.back().pose.position.y;
    if (std::hypot(gx - lx, gy - ly) > 1e-4) {
        dense.poses.push_back(goal);
    } else {
        dense.poses.back() = goal;
    }
    return dense;
}

commsgs::planning_msgs::Path AssignPathOrientations(
    const commsgs::planning_msgs::Path& path) {
    if (path.poses.size() < 2) {
        return path;
    }
    commsgs::planning_msgs::Path oriented = path;
    const std::string frame_id =
        oriented.poses.front().header.frame_id.empty()
            ? kGlobalFrame
            : oriented.poses.front().header.frame_id;
    for (size_t i = 0; i + 1 < oriented.poses.size(); ++i) {
        const double dx = oriented.poses[i + 1].pose.position.x -
                          oriented.poses[i].pose.position.x;
        const double dy = oriented.poses[i + 1].pose.position.y -
                          oriented.poses[i].pose.position.y;
        const double yaw = std::atan2(dy, dx);
        oriented.poses[i] = PoseFromState(oriented.poses[i].pose.position.x,
                                          oriented.poses[i].pose.position.y, yaw,
                                          frame_id);
    }
    return oriented;
}

commsgs::planning_msgs::Path EnsurePathFrame(
    commsgs::planning_msgs::Path path, const std::string& frame_id) {
    if (path.header.frame_id.empty()) {
        path.header.frame_id = frame_id;
    }
    for (auto& pose : path.poses) {
        if (pose.header.frame_id.empty()) {
            pose.header.frame_id = frame_id;
        }
    }
    return path;
}

commsgs::planning_msgs::Path PrepareReferencePath(
    const commsgs::planning_msgs::Path& planned) {
    commsgs::planning_msgs::Path path = AssignPathOrientations(planned);
    double spacing = FLAGS_path_pose_spacing;
    const size_t min_poses =
        static_cast<size_t>(std::max(2, FLAGS_min_reference_path_poses));
    for (int attempt = 0; attempt < 8 && spacing > 0.0; ++attempt) {
        path = DensifyPath(path, spacing);
        if (path.poses.size() >= min_poses) {
            break;
        }
        spacing *= 0.5;
    }
    return path;
}

void IntegrateDiffDrive(const commsgs::geometry_msgs::Twist& cmd, double dt,
                        double& x, double& y, double& yaw) {
    const double v = cmd.linear.x;
    const double w = cmd.angular.z;
    x += v * std::cos(yaw) * dt;
    y += v * std::sin(yaw) * dt;
    yaw = ::autonomy::common::math::NormalizeAngle(yaw + w * dt);
}

void PublishOdometry(const std::shared_ptr<utils::OdomSmoother>& odom_smoother,
                     const std::string& global_frame, double x, double y,
                     double yaw, const commsgs::geometry_msgs::Twist& twist) {
    commsgs::planning_msgs::Odometry odom;
    odom.header.stamp = Time::Now();
    odom.header.frame_id = global_frame;
    odom.child_frame_id = kRobotFrame;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    const double half_yaw = yaw * 0.5;
    odom.pose.pose.orientation.z = std::sin(half_yaw);
    odom.pose.pose.orientation.w = std::cos(half_yaw);
    odom.twist.twist = twist;
    odom_smoother->UpdateOdometry(odom);
}

bool PublishMapToBaseTf(double x, double y, double yaw) {
    geometry_msgs::TransformStamped tf;
    const auto now = Time::Now();
    tf.header.stamp = static_cast<uint64_t>(now.sec) * 1'000'000'000ULL +
                      now.nanosec;
    tf.header.frame_id = kGlobalFrame;
    tf.child_frame_id = kRobotFrame;
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    const double half_yaw = yaw * 0.5;
    tf.transform.rotation.z = std::sin(half_yaw);
    tf.transform.rotation.w = std::cos(half_yaw);
    return transform::Buffer::Instance()->setTransform(
        tf, "autonomy_controller_test", false);
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

std::vector<std::string> SplitCommaList(const std::string& value) {
    std::vector<std::string> parts;
    std::string current;
    for (char c : value) {
        if (c == ',' || c == ' ') {
            if (!current.empty()) {
                parts.push_back(current);
                current.clear();
            }
            continue;
        }
        current.push_back(c);
    }
    if (!current.empty()) {
        parts.push_back(current);
    }
    return parts;
}

std::vector<std::string> DiscoverControllers(
    const proto::ControllerOptions& options) {
    std::vector<std::string> ids;
    auto try_add = [&](const std::string& id) {
        if (!ControllerFactory::HasController(id)) {
            LOG(WARNING) << "Controller factory has no plugin for '" << id
                         << "', skipping.";
            return;
        }
        if (std::find(ids.begin(), ids.end(), id) == ids.end()) {
            ids.push_back(id);
        }
    };
    if (options.has_graceful_controller_options()) {
        try_add("graceful_controller");
    }
    if (options.has_nmpc_controller_options()) {
        try_add("nmpc_controller");
    }
    if (options.has_tdmpc_controller_options()) {
        try_add("tdmpc_controller");
    }
    if (options.has_mppi_controller_options()) {
        try_add("mppi_controller");
    }
    return ids;
}

std::string NormalizeOutputFormat(std::string format) {
    std::transform(format.begin(), format.end(), format.begin(),
                   [](unsigned char c) {
                       return static_cast<char>(std::tolower(c));
                   });
    return format;
}

std::vector<std::string> FilterKnownControllers(
    const std::vector<std::string>& ids) {
    std::vector<std::string> filtered;
    for (const auto& id : ids) {
        if (FLAGS_skip_mppi && id == "mppi_controller") {
            LOG(INFO) << "Skipping mppi_controller (--skip_mppi).";
            continue;
        }
        const std::string resolved =
            ControllerFactory::ResolveControllerTypeId(id);
        if (!ControllerFactory::HasController(resolved)) {
            LOG(WARNING) << "Unknown controller id '" << id
                         << "' (resolved=" << resolved << "), skipping.";
            continue;
        }
        filtered.push_back(id);
    }
    return filtered;
}

std::vector<std::string> ResolveControllerIds(
    const proto::ControllerOptions& options) {
    std::vector<std::string> ids;
    if (!FLAGS_controllers.empty()) {
        ids = SplitCommaList(FLAGS_controllers);
    } else {
        ids = DiscoverControllers(options);
    }
    return FilterKnownControllers(ids);
}

bool WantsMp4(const std::string& format) {
    return format == "mp4" || format == "both";
}

bool WantsGif(const std::string& format) {
    return format == "gif" || format == "both";
}

double ReferencePathLengthMeters(
    const commsgs::planning_msgs::Path& path) {
    if (path.poses.size() < 2) {
        return 0.0;
    }
    double length = 0.0;
    for (size_t i = 1; i < path.poses.size(); ++i) {
        const auto& a = path.poses[i - 1].pose.position;
        const auto& b = path.poses[i].pose.position;
        length += std::hypot(b.x - a.x, b.y - a.y);
    }
    return length;
}

int ResolveMaxSimSteps(const commsgs::planning_msgs::Path& reference_path) {
    const double path_len = ReferencePathLengthMeters(reference_path);
    // ~2 ticks per 2 cm along the path, plus margin for rotation / slowdown.
    const int from_length =
        static_cast<int>(path_len / 0.02 * 2.0) + 500;
    const int from_poses =
        static_cast<int>(reference_path.poses.size() * 1.5) + 500;
    const int scaled = std::max(from_length, from_poses);
    constexpr int kMaxSimStepsCap = 100000;
    const int capped = std::min(scaled, kMaxSimStepsCap);
    return std::max(FLAGS_max_sim_steps, capped);
}

#ifndef _WIN32
int RunProcess(const std::vector<std::string>& args) {
    if (args.empty()) {
        return -1;
    }
    const pid_t pid = fork();
    if (pid < 0) {
        return -1;
    }
    if (pid == 0) {
        std::vector<char*> argv;
        argv.reserve(args.size() + 1);
        for (const auto& arg : args) {
            argv.push_back(const_cast<char*>(arg.c_str()));
        }
        argv.push_back(nullptr);
        execvp(argv[0], argv.data());
        _exit(127);
    }
    int status = 0;
    if (waitpid(pid, &status, 0) < 0) {
        return -1;
    }
    if (WIFEXITED(status)) {
        return WEXITSTATUS(status);
    }
    return -1;
}
#endif

void DrawPathLegend(cv::Mat& frame) {
    const int y0 = frame.rows - 72;
    cv::line(frame, cv::Point(12, y0), cv::Point(52, y0), cv::Scalar(255, 200, 0),
             2, cv::LINE_AA);
    cv::putText(frame, "planner global path", cv::Point(60, y0 + 6),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(220, 220, 220), 1,
                cv::LINE_AA);
    cv::line(frame, cv::Point(12, y0 + 22), cv::Point(52, y0 + 22),
             cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
    cv::putText(frame, "executed trajectory", cv::Point(60, y0 + 28),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(220, 220, 220), 1,
                cv::LINE_AA);
    cv::circle(frame, cv::Point(32, y0 + 48), 6, cv::Scalar(0, 255, 255), -1,
              cv::LINE_AA);
    cv::putText(frame, "robot pose", cv::Point(60, y0 + 54),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(220, 220, 220), 1,
                cv::LINE_AA);
}

cv::Mat RenderDualPathFrame(
    const map::costmap_2d::Costmap2D& costmap,
    const commsgs::planning_msgs::Path& global_plan_path,
    const commsgs::planning_msgs::Path& executed_path, const std::string& label,
    const std::string& status_line, bool success) {
    cv::Mat frame = map::utils::PgmConverter::renderDualPathsToImage(
        costmap, global_plan_path, executed_path);
    if (frame.empty()) {
        return frame;
    }
    cv::putText(frame, label, cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    if (!status_line.empty()) {
        const cv::Scalar status_color =
            success ? cv::Scalar(0, 220, 0) : cv::Scalar(0, 0, 255);
        cv::putText(frame, status_line, cv::Point(12, 56),
                    cv::FONT_HERSHEY_SIMPLEX, 0.65, status_color, 2,
                    cv::LINE_AA);
    }
    if (executed_path.poses.size() < 2) {
        cv::putText(frame, "NO TRAJECTORY", cv::Point(12, 84),
                    cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 0, 255), 2,
                    cv::LINE_AA);
    }
    DrawPathLegend(frame);
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

bool WriteGifWithFfmpeg(const std::string& path,
                        const std::vector<cv::Mat>& frames, double fps,
                        const fs::path& scratch_dir) {
    if (frames.empty()) {
        return false;
    }
    fs::create_directories(scratch_dir);
    for (size_t i = 0; i < frames.size(); ++i) {
        char name[16];
        std::snprintf(name, sizeof(name), "%04zu.png", i);
        const fs::path frame_path = scratch_dir / name;
        if (!cv::imwrite(frame_path.string(), frames[i])) {
            LOG(ERROR) << "Failed to write frame: " << frame_path;
            return false;
        }
    }
#ifndef _WIN32
    const std::string input_pattern = (scratch_dir / "%04d.png").string();
    const std::string filter =
        "fps=" + std::to_string(fps) +
        ",scale=640:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]"
        "paletteuse";
    const std::vector<std::string> args = {
        "ffmpeg",       "-y",          "-hide_banner", "-loglevel", "error",
        "-framerate",   std::to_string(fps), "-i",    input_pattern,
        "-vf",          filter,        path};
    const int rc = RunProcess(args);
#else
    const int rc = -1;
#endif
    if (rc != 0) {
        LOG(WARNING) << "ffmpeg GIF export failed (rc=" << rc
                     << "). Install ffmpeg or use --output_format=mp4.";
        return false;
    }
    return true;
}

std::vector<cv::Mat> BuildTrajectoryAnimation(
    const map::costmap_2d::Costmap2D& costmap,
    const commsgs::planning_msgs::Path& global_plan_path,
    const commsgs::planning_msgs::Path& trajectory,
    const std::string& controller_id, const std::string& status_line,
    bool success, int animate_step, int hold_frames) {
    std::vector<cv::Mat> frames;
    const std::string label = "controller: " + controller_id;

    if (trajectory.poses.size() < 2) {
        frames.push_back(RenderDualPathFrame(costmap, global_plan_path, trajectory,
                                             label, status_line, success));
        for (int i = 0; i < hold_frames; ++i) {
            frames.push_back(frames.back().clone());
        }
        return frames;
    }

    int step = std::max(1, animate_step);
    const size_t pose_count = trajectory.poses.size();
    const size_t max_frames =
        static_cast<size_t>(std::max(2, FLAGS_max_animation_frames));
    if (pose_count > 2) {
        const size_t estimated =
            (pose_count - 2) / static_cast<size_t>(step) + 2;
        if (estimated > max_frames) {
            step = static_cast<int>(
                (pose_count + max_frames - 1) / max_frames);
            step = std::max(step, animate_step);
            LOG(INFO) << "Animation for " << controller_id
                      << ": " << pose_count << " poses, step=" << step
                      << " (cap " << max_frames << " frames)";
        }
    }

    for (size_t count = 2; count <= trajectory.poses.size(); count += step) {
        const auto partial = TruncatePath(trajectory, count);
        frames.push_back(RenderDualPathFrame(costmap, global_plan_path, partial,
                                             label, status_line, success));
    }
    if (trajectory.poses.size() % static_cast<size_t>(step) != 0 ||
        trajectory.poses.size() == 2) {
        frames.push_back(RenderDualPathFrame(costmap, global_plan_path, trajectory,
                                             label, status_line, success));
    }
    const cv::Mat final_frame = frames.back().clone();
    for (int i = 0; i < hold_frames; ++i) {
        frames.push_back(final_frame.clone());
    }
    return frames;
}

struct PlannerRunResult {
    bool success{false};
    commsgs::planning_msgs::Path path;
    std::string error;
};

PlannerRunResult PlanGlobalPath(
    planning::PlannerServer& planner,
    const commsgs::geometry_msgs::PoseStamped& start,
    const commsgs::geometry_msgs::PoseStamped& goal,
    const std::string& planner_id) {
    PlannerRunResult result;
    try {
        result.path = planner.ComputePathToPose(start, goal, planner_id,
                                                []() { return false; });
        result.success = result.path.poses.size() >= 2;
        if (!result.success) {
            result.error = "empty or too short path";
        } else if (result.path.poses.size() <
                   static_cast<size_t>(FLAGS_min_reference_path_poses)) {
            LOG(WARNING) << "Planner path has only " << result.path.poses.size()
                         << " poses after post-process; controllers may spin "
                            "in place. path_simplify_epsilon is forced to 0 "
                            "for this tool — check planner or use "
                            "--path_pose_spacing.";
        }
    } catch (const planning::common::PlannerException& e) {
        result.error = e.what();
    } catch (const std::exception& e) {
        result.error = e.what();
    }
    return result;
}

struct ControllerRunResult {
    bool success{false};
    std::string status;
    commsgs::planning_msgs::Path executed;
};

struct ControllerRunSummary {
    std::string controller_id;
    bool success{false};
    std::string status;
    size_t executed_poses{0};
};

void WriteRunSummary(const fs::path& output_root,
                     const std::string& planner_id,
                     const PlannerRunResult& plan,
                     size_t controller_reference_poses,
                     const std::vector<ControllerRunSummary>& runs) {
    const fs::path summary_path = output_root / "run_summary.txt";
    std::ofstream out(summary_path);
    if (!out) {
        LOG(WARNING) << "Failed to write " << summary_path;
        return;
    }
    out << "planner_id=" << planner_id << "\n";
    out << "global_plan_poses=" << plan.path.poses.size() << "\n";
    out << "controller_reference_poses=" << controller_reference_poses << "\n";
    out << "plan_success=" << (plan.success ? "true" : "false") << "\n";
    if (!plan.error.empty()) {
        out << "plan_error=" << plan.error << "\n";
    }
    out << "\n";
    for (const auto& run : runs) {
        out << "controller=" << run.controller_id << "\n";
        out << "  success=" << (run.success ? "true" : "false") << "\n";
        out << "  status=" << run.status << "\n";
        out << "  executed_poses=" << run.executed_poses << "\n";
    }
    LOG(INFO) << "Saved run summary: " << summary_path;
}

ControllerRunResult RunControllerSimulation(
    ControllerServer& server, const std::shared_ptr<utils::OdomSmoother>& odom,
    const commsgs::planning_msgs::Path& reference_path,
    const std::string& controller_id, double dt, int max_steps) {
    ControllerRunResult result;
    if (reference_path.poses.empty()) {
        result.status = "empty reference path";
        return result;
    }

    double x = reference_path.poses.front().pose.position.x;
    double y = reference_path.poses.front().pose.position.y;
    double yaw = transform::tf2::getYaw(
        reference_path.poses.front().pose.orientation);

    commsgs::geometry_msgs::Twist zero{};
    PublishOdometry(odom, kGlobalFrame, x, y, yaw, zero);
    PublishMapToBaseTf(x, y, yaw);

    if (!server.BeginFollowPath(reference_path, controller_id, "goal_checker",
                                "progress_checker")) {
        result.status = "BeginFollowPath failed";
        return result;
    }

    result.executed.header = reference_path.header;
    result.executed.poses.push_back(PoseFromState(x, y, yaw, kGlobalFrame));

    for (int step = 0; step < max_steps; ++step) {
        const auto tick =
            server.TickFollowPath([]() { return false; });
        if (tick == ControllerServer::FollowPathTickResult::Succeeded) {
            result.success = true;
            result.status = "goal reached";
            break;
        }
        if (tick == ControllerServer::FollowPathTickResult::Failed) {
            result.status = "controller failed";
            break;
        }
        if (tick == ControllerServer::FollowPathTickResult::Cancelled) {
            result.status = "cancelled";
            break;
        }

        const auto cmd = server.GetLastCmdVel().twist;
        IntegrateDiffDrive(cmd, dt, x, y, yaw);
        PublishOdometry(odom, kGlobalFrame, x, y, yaw, cmd);
        PublishMapToBaseTf(x, y, yaw);
        result.executed.poses.push_back(PoseFromState(x, y, yaw, kGlobalFrame));
    }

    if (!result.success && result.status.empty()) {
        result.status = "max steps exceeded";
    }
    server.EndFollowPath();
    return result;
}

}  // namespace

int RunControllerVisualization(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    const std::string format = NormalizeOutputFormat(FLAGS_output_format);
    if (!WantsMp4(format) && !WantsGif(format)) {
        LOG(FATAL) << "Invalid --output_format='" << FLAGS_output_format
                   << "'. Use mp4, gif, or both.";
        return 1;
    }

    const fs::path output_root(FLAGS_output_dir);
    const fs::path scratch_dir = output_root / "frames";
    fs::create_directories(scratch_dir);

    const std::string config_dir =
        ::autonomy::common::FLAGS_configuration_directory;
    auto planner_options = PrepareOfflinePlannerOptions(
        LoadPlannerOptionsFromLua(config_dir), FLAGS_use_synthetic_map);
    const auto controller_options = LoadControllerOptionsFromLua(config_dir);

    planning::PlannerServer planner(planner_options);
    auto costmap_wrapper = planner.GetCostmapWrapper();
    auto* costmap =
        costmap_wrapper != nullptr ? costmap_wrapper->getCostmap() : nullptr;
    if (costmap == nullptr) {
        LOG(FATAL) << "Planner costmap is null after construction";
        return 1;
    }

    if (FLAGS_use_synthetic_map) {
        SetupSyntheticCostmap(costmap, FLAGS_add_demo_obstacles);
        LOG(INFO) << "Using synthetic costmap";
        LogMapWorldBounds(*costmap, "synthetic");
    } else {
        try {
            const std::string map_yaml =
                ResolveMapYamlPath(config_dir, FLAGS_map_yaml);
            if (!costmap_wrapper->loadMap(map_yaml)) {
                LOG(FATAL) << "Failed to load map: " << map_yaml;
                return 1;
            }
            LOG(INFO) << "Loaded map: " << map_yaml;
            LogMapWorldBounds(*costmap, map_yaml);
        } catch (const std::exception& e) {
            LOG(FATAL) << "Map load failed: " << e.what();
            return 1;
        }
    }

    if (!PoseInsideMap(*costmap, FLAGS_start_x, FLAGS_start_y, "Start") ||
        !PoseInsideMap(*costmap, FLAGS_goal_x, FLAGS_goal_y, "Goal")) {
        return 1;
    }

    planner.Start();
    FinalizeOfflineCostmap(planner);

    const std::string frame_id = planner_options.costmap().frame_id().empty()
                                     ? kGlobalFrame
                                     : planner_options.costmap().frame_id();
    const auto start = MakePose(FLAGS_start_x, FLAGS_start_y, frame_id);
    const auto goal = MakePose(FLAGS_goal_x, FLAGS_goal_y, frame_id);

    std::string planner_id = FLAGS_planner_id;
    if (planner_id.empty()) {
        planner_id = planner_options.default_planner_id().empty()
                         ? "navfn_planner"
                         : planner_options.default_planner_id();
    }

    const PlannerRunResult plan =
        PlanGlobalPath(planner, start, goal, planner_id);
    if (!plan.success) {
        LOG(FATAL) << "Global planning failed (" << planner_id
                   << "): " << plan.error;
        return 1;
    }

    const commsgs::planning_msgs::Path global_plan_path =
        EnsurePathFrame(plan.path, frame_id);
    const commsgs::planning_msgs::Path reference_path =
        PrepareReferencePath(global_plan_path);
    LOG(INFO) << "Global plan: " << global_plan_path.poses.size()
              << " poses; controller reference (densified): "
              << reference_path.poses.size() << " poses (" << planner_id << ")";

    if (FLAGS_save_reference_png) {
        const std::string ref_png =
            (output_root / "reference_path.png").string();
        map::utils::PgmConverter::savePathToImage(*costmap, global_plan_path,
                                                  ref_png);
        LOG(INFO) << "Saved planner global path: " << ref_png;
    }

    ControllerServer controller(controller_options);
    auto tf = std::shared_ptr<transform::Buffer>(transform::Buffer::Instance(),
                                                 [](transform::Buffer*) {});
    controller.SetNavigationContext(tf, frame_id, kRobotFrame);
    controller.SetSharedCostmap(costmap_wrapper);

    const double control_hz =
        controller_options.controller_frequency() > 0.0
            ? controller_options.controller_frequency()
            : 20.0;
    const double dt = 1.0 / control_hz;
    controller.Start();
    auto odom_smoother = controller.GetOdomSmoother();

    const auto controller_ids = ResolveControllerIds(controller_options);
    if (controller_ids.empty()) {
        LOG(FATAL) << "No controllers to run. Configure blocks in "
                      "controller.lua or pass --controllers.";
        return 1;
    }

    const int max_sim_steps = ResolveMaxSimSteps(reference_path);
    LOG(INFO) << "Simulation max steps: " << max_sim_steps
              << " (reference length "
              << ReferencePathLengthMeters(reference_path) << " m)";

    std::vector<cv::Mat> combined_frames;
    std::vector<ControllerRunSummary> run_summaries;
    run_summaries.reserve(controller_ids.size());
    int success_count = 0;

    for (const auto& controller_id : controller_ids) {
        LOG(INFO) << "Running controller: " << controller_id;
        const ControllerRunResult run = RunControllerSimulation(
            controller, odom_smoother, reference_path, controller_id, dt,
            max_sim_steps);

        ControllerRunSummary summary;
        summary.controller_id = controller_id;
        summary.success = run.success;
        summary.status = run.status;
        summary.executed_poses = run.executed.poses.size();
        run_summaries.push_back(summary);

        if (run.success) {
            ++success_count;
        } else {
            LOG(WARNING) << "Controller " << controller_id
                         << " finished without goal: " << run.status
                         << " (poses=" << run.executed.poses.size() << ")";
        }

        const std::string status_line =
            (run.success ? "OK: " : "FAIL: ") + run.status;
        const std::string base_name = SanitizeFilename(controller_id);
        const std::string tracking_png =
            (output_root / (base_name + "_tracking.png")).string();
        cv::Mat tracking_frame = map::utils::PgmConverter::renderDualPathsToImage(
            *costmap, global_plan_path, run.executed);
        if (!tracking_frame.empty()) {
            DrawPathLegend(tracking_frame);
            if (cv::imwrite(tracking_png, tracking_frame)) {
                LOG(INFO) << "Saved tracking overlay (reference + executed): "
                          << tracking_png;
            }
        }

        const auto animation = BuildTrajectoryAnimation(
            *costmap, global_plan_path, run.executed, controller_id, status_line,
            run.success, FLAGS_animate_poses_per_frame, FLAGS_hold_frames);

        if (FLAGS_per_controller_video && !animation.empty()) {
            if (WantsMp4(format)) {
                const std::string video_path =
                    (output_root / (base_name + ".mp4")).string();
                if (WriteVideo(video_path, animation, FLAGS_fps)) {
                    LOG(INFO) << "Saved video: " << video_path;
                }
            }
            if (WantsGif(format)) {
                const std::string gif_path =
                    (output_root / (base_name + ".gif")).string();
                const fs::path gif_scratch = scratch_dir / (base_name + "_gif");
                if (WriteGifWithFfmpeg(gif_path, animation, FLAGS_fps,
                                       gif_scratch)) {
                    LOG(INFO) << "Saved GIF: " << gif_path;
                }
            }
        }

        combined_frames.insert(combined_frames.end(), animation.begin(),
                               animation.end());
    }

    if (!combined_frames.empty()) {
        if (WantsMp4(format)) {
            const std::string combined_mp4 =
                (output_root / FLAGS_combined_video_name).string();
            if (WriteVideo(combined_mp4, combined_frames, FLAGS_fps)) {
                LOG(INFO) << "Saved combined video: " << combined_mp4;
            }
        }
        if (WantsGif(format)) {
            const std::string combined_gif =
                (output_root / FLAGS_combined_gif_name).string();
            if (WriteGifWithFfmpeg(combined_gif, combined_frames, FLAGS_fps,
                                  scratch_dir / "combined_gif")) {
                LOG(INFO) << "Saved combined GIF: " << combined_gif;
            }
        }
    }

    if (FLAGS_write_run_summary) {
        WriteRunSummary(output_root, planner_id, plan,
                        reference_path.poses.size(), run_summaries);
    }

    controller.Shutdown();
    planner.Shutdown();

    LOG(INFO) << "Finished: " << success_count << "/"
              << controller_ids.size()
              << " controllers reached goal. Output: " << output_root;
    return success_count > 0 ? 0 : 2;
}

}  // namespace tools
}  // namespace control
}  // namespace autonomy

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    return autonomy::control::tools::RunControllerVisualization(argc, argv);
}
