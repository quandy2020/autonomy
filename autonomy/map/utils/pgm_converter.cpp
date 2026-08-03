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

#include "autonomy/map/utils/pgm_converter.hpp"

#include <algorithm>
#include <cctype>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "autonomy/common/logging.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/map_io.hpp"
#include "autonomy/map/costmap_2d/map_mode.hpp"

namespace {

constexpr double kUnknownGray = 0.5;
constexpr double kFreeGray = 1.0;
constexpr double kObstacleGray = 0.0;
constexpr double kCostNormFactor = 0.7;
constexpr double kPathPointRadius = 2.0;
constexpr int kMarkerStrokeWidth = 1;

double costToGray(unsigned char cost) {
    if (cost == autonomy::map::costmap_2d::NO_INFORMATION) {
        return kUnknownGray;
    }
    if (cost == autonomy::map::costmap_2d::FREE_SPACE) {
        return kFreeGray;
    }
    if (cost >= autonomy::map::costmap_2d::LETHAL_OBSTACLE) {
        return kObstacleGray;
    }
    const double norm =
        static_cast<double>(cost) /
        static_cast<double>(autonomy::map::costmap_2d::LETHAL_OBSTACLE);
    return kFreeGray - norm * kCostNormFactor;
}

cv::Mat createImageFromCostmap(
    const autonomy::map::costmap_2d::Costmap2D& costmap) {
    const unsigned int width = costmap.getSizeInCellsX();
    const unsigned int height = costmap.getSizeInCellsY();
    cv::Mat image(static_cast<int>(height), static_cast<int>(width), CV_8UC3,
                  cv::Scalar(255, 255, 255));

    const unsigned char* data = costmap.getCharMap();
    for (unsigned int y = 0; y < height; ++y) {
        for (unsigned int x = 0; x < width; ++x) {
            const double gray = costToGray(data[costmap.getIndex(x, y)]);
            const uint8_t value =
                static_cast<uint8_t>(std::clamp(gray, 0.0, 1.0) * 255.0);
            const unsigned int img_y = height - 1 - y;
            image.at<cv::Vec3b>(static_cast<int>(img_y),
                                static_cast<int>(x)) =
                cv::Vec3b(value, value, value);
        }
    }
    return image;
}

std::vector<cv::Point> convertPathToImageCoords(
    const autonomy::map::costmap_2d::Costmap2D& costmap,
    const automsgs::msgs::planning_msgs::Path& path, unsigned int height,
    unsigned int width) {
    std::vector<cv::Point> coords;
    coords.reserve(path.poses_size());
    const double ox = costmap.getOriginX();
    const double oy = costmap.getOriginY();
    const double max_x = ox + costmap.getSizeInMetersX();
    const double max_y = oy + costmap.getSizeInMetersY();
    for (const auto& pose : path.poses()) {
        double wx = pose.pose().position().x();
        double wy = pose.pose().position().y();
        wx = std::clamp(wx, ox, max_x);
        wy = std::clamp(wy, oy, max_y);
        unsigned int mx = 0;
        unsigned int my = 0;
        if (costmap.worldToMap(wx, wy, mx, my)) {
            coords.emplace_back(
                static_cast<int>(std::min(mx, width - 1)),
                static_cast<int>(std::min(height - 1 - my, height - 1)));
        }
    }
    return coords;
}

cv::Scalar pathLineColor(
    const autonomy::map::utils::PgmConverter::RenderParameters& params) {
    return cv::Scalar(params.path_color_b, params.path_color_g,
                      params.path_color_r);
}

void drawPath(cv::Mat& image, const std::vector<cv::Point>& coords,
              const autonomy::map::utils::PgmConverter::RenderParameters&
                  params) {
    if (coords.size() < 2) {
        return;
    }

    const cv::Scalar path_color = pathLineColor(params);
    for (size_t i = 0; i + 1 < coords.size(); ++i) {
        cv::line(image, coords[i], coords[i + 1], path_color,
                 static_cast<int>(params.path_line_width), cv::LINE_AA);
    }

    if (params.draw_path_points) {
        for (const auto& pt : coords) {
            cv::circle(image, pt, static_cast<int>(kPathPointRadius), path_color,
                       -1, cv::LINE_AA);
        }
    }

    if (params.draw_start_marker) {
        cv::circle(image, coords.front(),
                   static_cast<int>(params.start_marker_size),
                   cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
    }
    if (params.draw_goal_marker) {
        cv::circle(image, coords.back(),
                   static_cast<int>(params.goal_marker_size),
                   cv::Scalar(255, 0, 0), -1, cv::LINE_AA);
    }
}

std::string normalizeFormat(const std::string& format) {
    std::string norm = format;
    std::transform(norm.begin(), norm.end(), norm.begin(), ::tolower);
    return (norm == "jpg") ? "jpeg" : norm;
}

std::string ensureExtension(const std::string& path,
                            const std::string& format) {
    return (path.find('.') == std::string::npos) ? path + "." + format : path;
}

}  // namespace

namespace autonomy {
namespace map {
namespace utils {

map::costmap_2d::Costmap2D::SharedPtr PgmConverter::loadFromPgm(
    const std::string& pgm_file_path) {
    return loadFromPgm(pgm_file_path, LoadParameters());
}

map::costmap_2d::Costmap2D::SharedPtr PgmConverter::loadFromPgm(
    const std::string& pgm_file_path, const LoadParameters& params) {
    try {
        map::costmap_2d::LoadParameters load_params;
        load_params.image_file_name = pgm_file_path;
        load_params.resolution = params.resolution;
        load_params.origin = {params.origin_x, params.origin_y,
                              params.origin_yaw};
        load_params.free_thresh = params.free_thresh;
        load_params.occupied_thresh = params.occupied_thresh;
        load_params.mode = map::costmap_2d::MapMode::Trinary;
        load_params.negate = params.negate;

        automsgs::msgs::map_msgs::OccupancyGrid occupancy_grid;
        map::costmap_2d::loadMapFromFile(load_params, occupancy_grid);
        auto costmap =
            std::make_shared<map::costmap_2d::Costmap2D>(occupancy_grid);
        return costmap;
    } catch (const std::exception& e) {
        AERROR << "Failed to load PGM map from " << pgm_file_path << ": "
               << e.what();
        return nullptr;
    }
}

map::costmap_2d::Costmap2D::SharedPtr PgmConverter::loadFromYaml(
    const std::string& yaml_file_path) {
    try {
        map::costmap_2d::LoadParameters load_params =
            map::costmap_2d::loadMapYaml(yaml_file_path);
        automsgs::msgs::map_msgs::OccupancyGrid occupancy_grid;
        map::costmap_2d::loadMapFromFile(load_params, occupancy_grid);
        auto costmap =
            std::make_shared<map::costmap_2d::Costmap2D>(occupancy_grid);

        AINFO << "Loaded map from YAML: " << yaml_file_path << " -> "
              << costmap->getSizeInCellsX() << "x" << costmap->getSizeInCellsY()
              << " cells, resolution: " << costmap->getResolution()
              << " m/pixel";
        return costmap;
    } catch (const std::exception& e) {
        AERROR << "Failed to load map from YAML " << yaml_file_path << ": "
               << e.what();
        return nullptr;
    }
}

cv::Mat PgmConverter::renderPathToImage(
    const map::costmap_2d::Costmap2D& costmap,
    const automsgs::msgs::planning_msgs::Path& path, const RenderParameters& params) {
    try {
        cv::Mat image = createImageFromCostmap(costmap);
        const unsigned int width = costmap.getSizeInCellsX();
        const unsigned int height = costmap.getSizeInCellsY();

        if (!path.poses().empty()) {
            const std::vector<cv::Point> coords =
                convertPathToImageCoords(costmap, path, height, width);
            if (coords.size() >= 2) {
                drawPath(image, coords, params);
            }
        }
        return image;
    } catch (const std::exception& e) {
        AERROR << "Failed to render map with path: " << e.what();
        return {};
    }
}

bool PgmConverter::savePathToImage(const map::costmap_2d::Costmap2D& costmap,
                                   const automsgs::msgs::planning_msgs::Path& path,
                                   const std::string& output_file_path) {
    return savePathToImage(costmap, path, output_file_path, RenderParameters());
}

bool PgmConverter::savePathToImage(const map::costmap_2d::Costmap2D& costmap,
                                   const automsgs::msgs::planning_msgs::Path& path,
                                   const std::string& output_file_path,
                                   const RenderParameters& params) {
    try {
        const cv::Mat image = renderPathToImage(costmap, path, params);
        if (image.empty()) {
            return false;
        }

        const std::string format = normalizeFormat(params.output_format);
        const std::string final_path =
            ensureExtension(output_file_path, format);
        if (!cv::imwrite(final_path, image)) {
            throw std::runtime_error("OpenCV failed to write image: " +
                                   final_path);
        }

        if (params.log_on_save) {
            AINFO << "Saved map with path to: " << final_path
                  << " (format: " << format << ", points: "
                  << path.poses_size() << ")";
        }
        return true;
    } catch (const std::exception& e) {
        AERROR << "Failed to save map with path: " << e.what();
        return false;
    }
}

void drawPathOnCostmapImage(
    cv::Mat& image, const map::costmap_2d::Costmap2D& costmap,
    const automsgs::msgs::planning_msgs::Path& path,
    const PgmConverter::RenderParameters& params) {
    if (path.poses().empty()) {
        return;
    }
    const unsigned int width = costmap.getSizeInCellsX();
    const unsigned int height = costmap.getSizeInCellsY();
    const std::vector<cv::Point> coords =
        convertPathToImageCoords(costmap, path, height, width);
    if (coords.size() >= 2) {
        drawPath(image, coords, params);
    }
}

cv::Mat PgmConverter::renderDualPathsToImage(
    const map::costmap_2d::Costmap2D& costmap,
    const automsgs::msgs::planning_msgs::Path& global_plan_path,
    const automsgs::msgs::planning_msgs::Path& executed_path) {
    try {
        cv::Mat image = createImageFromCostmap(costmap);

        RenderParameters reference_params;
        reference_params.path_color_b = 255;
        reference_params.path_color_g = 200;
        reference_params.path_color_r = 0;
        reference_params.path_line_width = 2.0;
        reference_params.draw_start_marker = true;
        reference_params.draw_goal_marker = true;
        drawPathOnCostmapImage(image, costmap, global_plan_path, reference_params);

        RenderParameters executed_params;
        executed_params.path_color_b = 0;
        executed_params.path_color_g = 0;
        executed_params.path_color_r = 255;
        executed_params.path_line_width = 3.0;
        executed_params.draw_start_marker = false;
        executed_params.draw_goal_marker = false;
        drawPathOnCostmapImage(image, costmap, executed_path, executed_params);

        if (!executed_path.poses().empty()) {
            const unsigned int width = costmap.getSizeInCellsX();
            const unsigned int height = costmap.getSizeInCellsY();
            const std::vector<cv::Point> coords = convertPathToImageCoords(
                costmap, executed_path, height, width);
            if (!coords.empty()) {
                cv::circle(image, coords.back(), 6, cv::Scalar(0, 255, 255), -1,
                           cv::LINE_AA);
            }
        }
        return image;
    } catch (const std::exception& e) {
        AERROR << "Failed to render dual paths: " << e.what();
        return {};
    }
}

bool PgmConverter::saveDualPathsToImage(
    const map::costmap_2d::Costmap2D& costmap,
    const automsgs::msgs::planning_msgs::Path& global_plan_path,
    const automsgs::msgs::planning_msgs::Path& executed_path,
    const std::string& output_file_path) {
    try {
        const cv::Mat image =
            renderDualPathsToImage(costmap, global_plan_path, executed_path);
        if (image.empty()) {
            return false;
        }
        const std::string final_path = ensureExtension(output_file_path, "png");
        if (!cv::imwrite(final_path, image)) {
            throw std::runtime_error("OpenCV failed to write image: " +
                                   final_path);
        }
        AINFO << "Saved map with global plan + executed paths to: " << final_path
              << " (global plan poses: " << global_plan_path.poses_size()
              << ", executed poses: " << executed_path.poses_size() << ")";
        return true;
    } catch (const std::exception& e) {
        AERROR << "Failed to save dual paths image: " << e.what();
        return false;
    }
}

}  // namespace utils
}  // namespace map
}  // namespace autonomy
