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

#pragma once

#include <memory>
#include <string>

#include <opencv2/core.hpp>

#include <automsgs/msgs/planning_msgs/planning_msgs.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include "autonomy/map/costmap_2d/costmap_2d.hpp"

namespace autonomy {
namespace map {
namespace utils {

// Utility for converting PGM maps to Costmap2D and rendering paths.
class PgmConverter
{
public:
    struct LoadParameters {
        double resolution{0.05};       // meters/pixel
        double origin_x{0.0};          // meters
        double origin_y{0.0};          // meters
        double origin_yaw{0.0};        // radians
        double free_thresh{0.196};     // [0, 1]
        double occupied_thresh{0.65};  // [0, 1]
        bool negate{false};
    };

    // Loads PGM file and converts to Costmap2D. Returns nullptr on failure.
    static map::costmap_2d::Costmap2D::SharedPtr loadFromPgm(
        const std::string& pgm_file_path, const LoadParameters& params);

    // Overload with default parameters.
    static map::costmap_2d::Costmap2D::SharedPtr loadFromPgm(
        const std::string& pgm_file_path);

    // Loads map from YAML metadata file. Returns nullptr on failure.
    static map::costmap_2d::Costmap2D::SharedPtr loadFromYaml(
        const std::string& yaml_file_path);

    struct RenderParameters {
        std::string output_format{"png"};
        double path_line_width{2.0};
        double start_marker_size{5.0};
        double goal_marker_size{5.0};
        bool draw_start_marker{true};
        bool draw_goal_marker{true};
        bool draw_path_points{false};
        bool log_on_save{true};
        // BGR path line color (OpenCV convention).
        uint8_t path_color_b{0};
        uint8_t path_color_g{0};
        uint8_t path_color_r{255};
    };

    // Renders costmap with path overlay in memory. Empty Mat on failure.
    static cv::Mat renderPathToImage(
        const map::costmap_2d::Costmap2D& costmap,
        const automsgs::msgs::planning_msgs::Path& path,
        const RenderParameters& params);

    // Planner global path (cyan) + executed trajectory (red) on one image.
    static cv::Mat renderDualPathsToImage(
        const map::costmap_2d::Costmap2D& costmap,
        const automsgs::msgs::planning_msgs::Path& global_plan_path,
        const automsgs::msgs::planning_msgs::Path& executed_path);

    static bool saveDualPathsToImage(
        const map::costmap_2d::Costmap2D& costmap,
        const automsgs::msgs::planning_msgs::Path& global_plan_path,
        const automsgs::msgs::planning_msgs::Path& executed_path,
        const std::string& output_file_path);

    // Renders costmap with path overlay and saves as image. Returns false on
    // failure.
    static bool savePathToImage(const map::costmap_2d::Costmap2D& costmap,
                                const automsgs::msgs::planning_msgs::Path& path,
                                const std::string& output_file_path,
                                const RenderParameters& params);

    // Overload with default parameters.
    static bool savePathToImage(const map::costmap_2d::Costmap2D& costmap,
                                const automsgs::msgs::planning_msgs::Path& path,
                                const std::string& output_file_path);
};

}  // namespace utils
}  // namespace map
}  // namespace autonomy
