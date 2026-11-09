/*
 * Copyright 2026 The Navigation Authors (navigation)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "navigation/planning/tasks/frontiernet_frontier_source.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <queue>
#include <utility>

namespace navigation
{
namespace planning
{
namespace
{

/** @brief Official log1p gain-bin edges for the 11-class checkpoint. */
constexpr std::array<double, 11> kGainEdges{1.0,   50.0,  150.0, 300.0, 450.0, 750.0,
                                            1000.0, 1500.0, 2000.0, 2500.0, 3500.0};

/** @brief One image-space frontier cluster. */
struct Cluster
{
    /** @brief Gain-weighted sum of map x. */
    double sum_x{0.0};
    /** @brief Gain-weighted sum of map y. */
    double sum_y{0.0};
    /** @brief Sum of the weights. */
    double weight{0.0};
    /** @brief Number of pixels in the cluster. */
    std::uint32_t pixels{0};
};

/** @brief Gain in m^3 for one class label. */
double LabelGain(int label)
{
    if (label <= 0 || label >= static_cast<int>(kGainEdges.size()))
    {
        return 0.0;
    }
    const double low = std::log1p(kGainEdges[static_cast<std::size_t>(label) - 1]);
    const double high = std::log1p(kGainEdges[static_cast<std::size_t>(label)]);
    return std::expm1(0.5 * (low + high));
}

}  // namespace

std::vector<std::uint8_t> RegionFromDf(const std::vector<float>& df,
                                       double normalizer, double threshold)
{
    std::vector<std::uint8_t> region(df.size(), 0);
    if (normalizer <= 0.0 || threshold <= 0.0)
    {
        return region;
    }
    // d = exp(-df) * normalizer; d < threshold  <=>  df > log(normalizer/threshold)
    const double cutoff = std::log(normalizer / threshold);
    for (std::size_t index = 0; index < df.size(); ++index)
    {
        if (std::isfinite(df[index]) && df[index] > cutoff)
        {
            region[index] = 1;
        }
    }
    return region;
}

std::vector<float> GainFromMasks(const std::vector<float>& masks,
                                 std::uint32_t classes, std::uint32_t size)
{
    std::vector<float> gain;
    const std::size_t pixels = static_cast<std::size_t>(size) * size;
    if (classes == 0 || masks.size() != pixels * classes)
    {
        return gain;
    }
    gain.assign(pixels, 0.0F);
    std::array<double, 11> table{};
    for (std::size_t label = 0; label < table.size(); ++label)
    {
        table[label] = LabelGain(static_cast<int>(label));
    }
    for (std::size_t pixel = 0; pixel < pixels; ++pixel)
    {
        int best = 0;
        float best_value = masks[pixel];
        for (std::uint32_t channel = 1; channel < classes; ++channel)
        {
            const float value = masks[static_cast<std::size_t>(channel) * pixels + pixel];
            if (value > best_value)
            {
                best_value = value;
                best = static_cast<int>(channel);
            }
        }
        gain[pixel] = static_cast<float>(
            best < static_cast<int>(table.size()) ? table[static_cast<std::size_t>(best)]
                                                  : 0.0);
    }
    return gain;
}

bool ProjectPixelToMap(std::uint32_t u, std::uint32_t v, float depth_m,
                       const FrontierCameraModel& camera,
                       const common::Pose2D& robot_pose, double min_depth,
                       double max_depth, double* map_x, double* map_y)
{
    if (map_x == nullptr || map_y == nullptr || camera.fx <= 0.0 ||
        camera.fy <= 0.0)
    {
        return false;
    }
    const double depth = static_cast<double>(depth_m);
    if (!std::isfinite(depth) || depth < min_depth || depth > max_depth)
    {
        return false;
    }
    // Optical frame: x right, y down, z forward.
    const double x_optical = (static_cast<double>(u) + 0.5 - camera.cx) * depth / camera.fx;
    const double y_optical = (static_cast<double>(v) + 0.5 - camera.cy) * depth / camera.fy;
    // Base frame: x forward, y left, z up.
    const double x_base = depth;
    const double y_base = -x_optical;
    (void)y_optical;
    const double c = std::cos(robot_pose.theta());
    const double s = std::sin(robot_pose.theta());
    *map_x = robot_pose.x() + c * x_base - s * y_base;
    *map_y = robot_pose.y() + s * x_base + c * y_base;
    return true;
}

FrontierNetFrontierSource::FrontierNetFrontierSource(
    FrontierNetFrontierOptions options)
    : options_(options)
{
}

bool FrontierNetFrontierSource::LoadModel(const config::ModelManifest& manifest,
                                          std::string* error)
{
    model_.reset();
    auto model = perception::CreateFrontierNetModel(manifest, error);
    if (model == nullptr)
    {
        return false;
    }
    perception::DepthPreprocessorOptions preprocess;
    preprocess.input_size = model->InputSize();
    if (manifest.mean_size() == 3 && manifest.std_size() == 3)
    {
        for (int i = 0; i < 3; ++i)
        {
            preprocess.mean[static_cast<std::size_t>(i)] = manifest.mean(i);
            preprocess.std[static_cast<std::size_t>(i)] = manifest.std(i);
        }
    }
    preprocessor_ = perception::DepthPreprocessor(preprocess);
    model_ = std::move(model);
    return true;
}

bool FrontierNetFrontierSource::Ready() const
{
    return model_ != nullptr && model_->Ready();
}

const FrontierNetFrontierOptions& FrontierNetFrontierSource::Options() const
{
    return options_;
}

std::vector<Waypoint> FrontierNetFrontierSource::Detect(
    const observation::Observation& observation,
    const common::Pose2D& robot_pose)
{
    // Keep the diagnostics alive so the log line reports real latency.
    FrontierNetFrontierStats stats;
    return Detect(observation, robot_pose, &stats);
}

std::vector<Waypoint> FrontierNetFrontierSource::Detect(
    const observation::Observation& observation,
    const common::Pose2D& robot_pose, FrontierNetFrontierStats* stats)
{
    std::vector<Waypoint> waypoints;
    if (stats != nullptr)
    {
        *stats = FrontierNetFrontierStats{};
        stats->model_ready = Ready();
    }
    if (!Ready() || !observation.has_depth() ||
        !observation.has_camera_info())
    {
        return waypoints;
    }
    const auto& image = observation.depth();
    const auto& info = observation.camera_info();
    if (image.width() == 0 || image.height() == 0 || info.k_size() < 9)
    {
        return waypoints;
    }
    std::vector<float> input;
    perception::DepthStats depth_stats;
    bool depth_ok = false;
    bool model_ok = false;
    if (!preprocessor_.Process(image, &input, &depth_stats))
    {
        if (options_.log_frontiers)
        {
            std::fprintf(stderr,
                         "[frontiernet] preprocess failed valid_ratio=%.3f\n",
                         depth_stats.valid_ratio);
        }
        return waypoints;
    }
    const auto started = std::chrono::steady_clock::now();
    perception::FrontierNetOutput output;
    if (!model_->Run(input, &output))
    {
        if (options_.log_frontiers)
        {
            std::fprintf(stderr, "[frontiernet] model run failed\n");
        }
        return waypoints;
    }
    depth_ok = true;
    model_ok = true;
    const auto finished = std::chrono::steady_clock::now();
    if (stats != nullptr)
    {
        stats->depth_ok = true;
        stats->inference_ms =
            std::chrono::duration<double, std::milli>(finished - started).count();
    }
    const std::uint32_t size = output.size;
    const auto region = RegionFromDf(output.df, options_.df_normalizer,
                                     options_.df_threshold);
    const auto gain = GainFromMasks(output.masks, output.classes, size);
    if (region.size() != static_cast<std::size_t>(size) * size ||
        gain.size() != region.size())
    {
        return waypoints;
    }
    std::vector<float> metres;
    if (!perception::DepthPreprocessor::Decode(image, &metres) ||
        metres.size() != static_cast<std::size_t>(image.width()) * image.height())
    {
        return waypoints;
    }
    FrontierCameraModel camera;
    const double scale_x = static_cast<double>(size) / static_cast<double>(image.width());
    const double scale_y = static_cast<double>(size) / static_cast<double>(image.height());
    camera.fx = info.k(0) * scale_x;
    camera.fy = info.k(4) * scale_y;
    camera.cx = info.k(2) * scale_x;
    camera.cy = info.k(5) * scale_y;
    if (camera.fx <= 0.0 || camera.fy <= 0.0)
    {
        return waypoints;
    }
    const auto sample_depth = [&](std::uint32_t u, std::uint32_t v) {
        const auto source_x = std::min(
            static_cast<std::uint32_t>(image.width()) - 1,
            static_cast<std::uint32_t>(static_cast<double>(u) * image.width() / size));
        const auto source_y = std::min(
            static_cast<std::uint32_t>(image.height()) - 1,
            static_cast<std::uint32_t>(static_cast<double>(v) * image.height() / size));
        return metres[static_cast<std::size_t>(source_y) * image.width() + source_x];
    };
    std::vector<std::uint8_t> visited(region.size(), 0);
    std::vector<Cluster> clusters;
    std::size_t frontier_pixels = 0;
    for (std::uint32_t v = 0; v < size; ++v)
    {
        for (std::uint32_t u = 0; u < size; ++u)
        {
            const std::size_t index = static_cast<std::size_t>(v) * size + u;
            if (visited[index] != 0 || region[index] == 0 ||
                gain[index] < static_cast<float>(options_.min_gain))
            {
                continue;
            }
            Cluster cluster;
            std::queue<std::pair<std::uint32_t, std::uint32_t>> queue;
            queue.emplace(u, v);
            visited[index] = 1;
            while (!queue.empty())
            {
                const auto [cu, cv] = queue.front();
                queue.pop();
                const std::size_t current = static_cast<std::size_t>(cv) * size + cu;
                double map_x = 0.0;
                double map_y = 0.0;
                if (ProjectPixelToMap(cu, cv, sample_depth(cu, cv), camera,
                                      robot_pose, options_.min_depth,
                                      options_.max_depth, &map_x, &map_y))
                {
                    const double weight =
                        std::max(1.0, static_cast<double>(gain[current]));
                    cluster.sum_x += weight * map_x;
                    cluster.sum_y += weight * map_y;
                    cluster.weight += weight;
                    ++cluster.pixels;
                    ++frontier_pixels;
                }
                const int offsets[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
                for (const auto& offset : offsets)
                {
                    const int nu = static_cast<int>(cu) + offset[0];
                    const int nv = static_cast<int>(cv) + offset[1];
                    if (nu < 0 || nv < 0 || nu >= static_cast<int>(size) ||
                        nv >= static_cast<int>(size))
                    {
                        continue;
                    }
                    const std::size_t next =
                        static_cast<std::size_t>(nv) * size + static_cast<std::size_t>(nu);
                    if (visited[next] == 0 && region[next] != 0 &&
                        gain[next] >= static_cast<float>(options_.min_gain))
                    {
                        visited[next] = 1;
                        queue.emplace(static_cast<std::uint32_t>(nu),
                                      static_cast<std::uint32_t>(nv));
                    }
                }
            }
            if (cluster.pixels >= options_.min_cluster_pixels && cluster.weight > 0.0)
            {
                clusters.push_back(cluster);
            }
        }
    }
    for (const auto& cluster : clusters)
    {
        common::Pose2D pose;
        pose.set_x(cluster.sum_x / cluster.weight);
        pose.set_y(cluster.sum_y / cluster.weight);
        const double distance = common::Distance(pose, robot_pose);
        if (distance < options_.min_forward_m ||
            (options_.max_range > 0.0 && distance > options_.max_range))
        {
            continue;
        }
        if (options_.target_distance > 0.0 && distance > options_.target_distance)
        {
            const double ratio = options_.target_distance / distance;
            pose.set_x(robot_pose.x() + (pose.x() - robot_pose.x()) * ratio);
            pose.set_y(robot_pose.y() + (pose.y() - robot_pose.y()) * ratio);
        }
        pose.set_theta(std::atan2(pose.y() - robot_pose.y(),
                                  pose.x() - robot_pose.x()));
        Waypoint waypoint;
        waypoint.pose = pose;
        waypoints.push_back(waypoint);
    }
    std::sort(waypoints.begin(), waypoints.end(),
              [&robot_pose](const Waypoint& lhs, const Waypoint& rhs) {
                  return common::Distance(lhs.pose, robot_pose) <
                         common::Distance(rhs.pose, robot_pose);
              });
    std::vector<Waypoint> unique;
    for (const auto& waypoint : waypoints)
    {
        bool duplicate = false;
        for (const auto& kept : unique)
        {
            if (common::Distance(kept.pose, waypoint.pose) < 0.5)
            {
                duplicate = true;
                break;
            }
        }
        if (!duplicate)
        {
            unique.push_back(waypoint);
        }
    }
    if (stats != nullptr)
    {
        stats->frontier_pixels = frontier_pixels;
        stats->clusters = unique.size();
    }
    if (options_.log_frontiers)
    {
        std::fprintf(stderr,
                     "[frontiernet] pixels=%zu clusters=%zu inference=%.1fms "
                     "depth_ok=%d model_ready=%d",
                     frontier_pixels, unique.size(),
                     stats != nullptr ? stats->inference_ms : 0.0,
                     depth_ok ? 1 : 0, model_ok ? 1 : 0);
        if (!unique.empty())
        {
            std::fprintf(stderr, " target=(%.2f, %.2f) d=%.2f", unique.front().pose.x(),
                         unique.front().pose.y(),
                         common::Distance(unique.front().pose, robot_pose));
        }
        std::fprintf(stderr, "\n");
    }
    return unique;
}

}  // namespace planning
}  // namespace navigation
