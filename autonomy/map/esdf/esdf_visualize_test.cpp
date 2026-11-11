/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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
 * @file esdf_visualize_test.cpp
 * @brief Multi-case occupancy / ESDF z-slice visualization (no ROS).
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "autonomy/map/esdf/esdf_map.hpp"

namespace {

using autonomy::map::esdf::EsdfMap;
using autonomy::map::esdf::Options;

constexpr double kSliceHeight = 0.0;
constexpr double kMaxDistanceMetres = 2.0;
constexpr int kPixelScale = 3;

void ObserveOccupied(EsdfMap* map, const Eigen::Vector3d& position,
                     int rounds = 3) {
    for (int i = 0; i < rounds; ++i) {
        map->SetOccupancy(position, true);
        map->UpdateOccupancy(true);
    }
}

void ObserveFree(EsdfMap* map, const Eigen::Vector3d& position, int rounds = 8) {
    for (int i = 0; i < rounds; ++i) {
        map->SetOccupancy(position, false);
        map->UpdateOccupancy(true);
    }
}

void AddWallX(EsdfMap* map, double x, double y_min, double y_max) {
    const double resolution = map->options().resolution;
    for (double y = y_min; y <= y_max + 0.5 * resolution; y += resolution) {
        ObserveOccupied(map, Eigen::Vector3d(x, y, kSliceHeight));
    }
}

void AddWallY(EsdfMap* map, double y, double x_min, double x_max) {
    const double resolution = map->options().resolution;
    for (double x = x_min; x <= x_max + 0.5 * resolution; x += resolution) {
        ObserveOccupied(map, Eigen::Vector3d(x, y, kSliceHeight));
    }
}

void AddBox(EsdfMap* map, double x_min, double y_min, double x_max,
            double y_max) {
    const double resolution = map->options().resolution;
    for (double x = x_min; x <= x_max + 0.5 * resolution; x += resolution) {
        for (double y = y_min; y <= y_max + 0.5 * resolution; y += resolution) {
            ObserveOccupied(map, Eigen::Vector3d(x, y, kSliceHeight));
        }
    }
}

void ClearBox(EsdfMap* map, double x_min, double y_min, double x_max,
              double y_max) {
    const double resolution = map->options().resolution;
    for (double x = x_min; x <= x_max + 0.5 * resolution; x += resolution) {
        for (double y = y_min; y <= y_max + 0.5 * resolution; y += resolution) {
            ObserveFree(map, Eigen::Vector3d(x, y, kSliceHeight));
        }
    }
}

Options VizOptions() {
    Options options;
    options.origin = Eigen::Vector3d(-3.2, -3.2, -0.4);
    options.map_size = Eigen::Vector3d(6.4, 6.4, 0.8);
    options.resolution = 0.08;
    options.min_ray_length = 0.0;
    options.max_ray_length = 20.0;
    return options;
}

cv::Mat OccupancySlice(const EsdfMap& map) {
    const auto& options = map.options();
    const int width = static_cast<int>(
        std::lround(options.map_size.x() / options.resolution));
    const int height = static_cast<int>(
        std::lround(options.map_size.y() / options.resolution));
    cv::Mat image(height, width, CV_8UC3);
    for (int row = 0; row < height; ++row) {
        const double y =
            options.origin.y() + (height - 1 - row + 0.5) * options.resolution;
        for (int col = 0; col < width; ++col) {
            const double x =
                options.origin.x() + (col + 0.5) * options.resolution;
            const int occupancy = map.GetOccupancy(Eigen::Vector3d(x, y, kSliceHeight));
            image.at<cv::Vec3b>(row, col) =
                occupancy == 1 ? cv::Vec3b(36, 36, 36) : cv::Vec3b(248, 248, 248);
        }
    }
    return image;
}

cv::Mat DistanceSlice(const EsdfMap& map) {
    const auto& options = map.options();
    const int width = static_cast<int>(
        std::lround(options.map_size.x() / options.resolution));
    const int height = static_cast<int>(
        std::lround(options.map_size.y() / options.resolution));
    cv::Mat gray(height, width, CV_8UC1);
    for (int row = 0; row < height; ++row) {
        const double y =
            options.origin.y() + (height - 1 - row + 0.5) * options.resolution;
        for (int col = 0; col < width; ++col) {
            const double x =
                options.origin.x() + (col + 0.5) * options.resolution;
            const Eigen::Vector3d position(x, y, kSliceHeight);
            if (map.GetOccupancy(position) == 1) {
                gray.at<std::uint8_t>(row, col) = 0;
                continue;
            }
            const double clipped = std::min(
                std::max(map.GetDistance(position), 0.0), kMaxDistanceMetres);
            gray.at<std::uint8_t>(row, col) = static_cast<std::uint8_t>(
                std::lround(255.0 * clipped / kMaxDistanceMetres));
        }
    }
    cv::Mat color;
    cv::applyColorMap(gray, color, cv::COLORMAP_TURBO);
    for (int row = 0; row < height; ++row) {
        for (int col = 0; col < width; ++col) {
            if (gray.at<std::uint8_t>(row, col) == 0) {
                color.at<cv::Vec3b>(row, col) = cv::Vec3b(16, 16, 16);
            }
        }
    }
    return color;
}

void Annotate(cv::Mat* image, const std::string& title, bool light) {
    cv::putText(*image, title, cv::Point(6, 16), cv::FONT_HERSHEY_SIMPLEX, 0.42,
                light ? cv::Scalar(30, 30, 30) : cv::Scalar(255, 255, 255), 1,
                cv::LINE_AA);
}

cv::Mat Upscale(const cv::Mat& image) {
    cv::Mat out;
    cv::resize(image, out, cv::Size(), kPixelScale, kPixelScale, cv::INTER_NEAREST);
    return out;
}

bool WritePair(const std::string& output_directory, const std::string& name,
               const EsdfMap& map) {
    cv::Mat occupancy = OccupancySlice(map);
    cv::Mat distance = DistanceSlice(map);
    Annotate(&occupancy, name + " occupancy", true);
    Annotate(&distance, name + " ESDF 0-2m", false);
    occupancy = Upscale(occupancy);
    distance = Upscale(distance);
    const std::string occupancy_path =
        output_directory + "/esdf_case_" + name + "_occ.png";
    const std::string distance_path =
        output_directory + "/esdf_case_" + name + "_esdf.png";
    if (!cv::imwrite(occupancy_path, occupancy) ||
        !cv::imwrite(distance_path, distance)) {
        std::fprintf(stderr, "write failed: %s\n", name.c_str());
        return false;
    }
    std::printf("%-16s occupied=%4zu  consistency=%s\n", name.c_str(),
                map.OccupiedPoints().size(),
                map.CheckConsistency() ? "ok" : "FAIL");
    return map.CheckConsistency();
}

using Builder = std::function<void(EsdfMap*)>;

struct Case {
    const char* name;
    Builder build;
};

}  // namespace

TEST(EsdfVisualize, Cases) {
    const std::string output_directory = testing::TempDir();
    const std::vector<Case> cases = {
        {"empty", [](EsdfMap*) {}},
        {"wall",
         [](EsdfMap* map) { AddWallX(map, 0.0, -1.6, 1.6); }},
        {"pillar",
         [](EsdfMap* map) { AddBox(map, 0.7, 0.7, 1.2, 1.2); }},
        {"wall_pillar",
         [](EsdfMap* map) {
             AddWallX(map, 0.0, -1.6, 1.6);
             AddBox(map, 1.6, 0.8, 2.1, 1.3);
         }},
        {"corridor",
         [](EsdfMap* map) {
             AddWallX(map, -0.9, -2.2, 2.2);
             AddWallX(map, 0.9, -2.2, 2.2);
         }},
        {"l_corner",
         [](EsdfMap* map) {
             AddWallX(map, 0.0, -1.6, 0.0);
             AddWallY(map, 0.0, 0.0, 1.6);
         }},
        {"boxes",
         [](EsdfMap* map) {
             AddBox(map, -1.6, -0.4, -0.8, 0.4);
             AddBox(map, 0.8, -1.4, 1.6, -0.6);
         }},
        {"rays",
         [](EsdfMap* map) {
             const Eigen::Vector3d origin(-2.4, 0.0, kSliceHeight);
             for (int i = -8; i <= 8; ++i) {
                 const Eigen::Vector3d hit(1.6, 0.22 * i, kSliceHeight);
                 for (int k = 0; k < 3; ++k) {
                     map->IntegrateRay(origin, hit);
                     map->UpdateOccupancy(true);
                 }
             }
         }},
        {"removed",
         [](EsdfMap* map) {
             AddBox(map, -0.4, -0.4, 0.4, 0.4);
             map->UpdateESDF();
             ClearBox(map, -0.4, -0.4, 0.4, 0.4);
         }},
    };

    std::vector<cv::Mat> tiles;
    for (const auto& item : cases) {
        EsdfMap map(VizOptions());
        item.build(&map);
        map.UpdateESDF();
        EXPECT_TRUE(WritePair(output_directory, item.name, map)) << item.name;
        cv::Mat tile = Upscale(DistanceSlice(map));
        Annotate(&tile, item.name, false);
        tiles.push_back(tile);
    }

    ASSERT_FALSE(tiles.empty());
    const int cols = 3;
    const int rows = (static_cast<int>(tiles.size()) + cols - 1) / cols;
    const cv::Size cell = tiles.front().size();
    cv::Mat sheet(rows * cell.height, cols * cell.width, CV_8UC3,
                  cv::Scalar(10, 10, 10));
    for (int i = 0; i < static_cast<int>(tiles.size()); ++i) {
        const int r = i / cols;
        const int c = i % cols;
        tiles[static_cast<std::size_t>(i)].copyTo(
            sheet(cv::Rect(c * cell.width, r * cell.height, cell.width,
                           cell.height)));
    }
    const std::string sheet_path = output_directory + "/esdf_cases.png";
    ASSERT_TRUE(cv::imwrite(sheet_path, sheet)) << sheet_path;
    std::printf("wrote %s\n", sheet_path.c_str());
}
