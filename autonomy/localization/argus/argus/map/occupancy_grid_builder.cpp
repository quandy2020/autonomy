/*
 * Copyright 2026 The Openbot Authors
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

#include "argus/map/occupancy_grid_builder.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace argus {
namespace map {
namespace {

constexpr double kRad2Deg = 180.0 / M_PI;
constexpr double kDeg2Rad = M_PI / 180.0;

Eigen::Vector3d PointBody(const sensor::PointCloudXYZI& cloud,
                          std::size_t i) {
    return Eigen::Vector3d(cloud[i].x, cloud[i].y, cloud[i].z);
}

void FillRaysFromCloud(
    const transform::Rigid3d& T_w_b, const sensor::PointCloudXYZI& cloud_b,
    bool cloud_in_world, const OccupancyGridBuilderOptions& options,
    const Eigen::Vector4d& floor_coeffs, OccupancyGridMap* map,
    std::vector<std::vector<std::pair<double, double>>>* rays) {
    const double min_th = options.min_th_floor;
    const double max_th = options.max_th_floor;
    for (std::size_t i = 0; i < cloud_b.size(); ++i) {
        Eigen::Vector3d pc = PointBody(cloud_b, i);
        if (cloud_in_world) {
            // Recover body for angle / range; world for SetHit.
            pc = T_w_b.inverse() * pc;
        }
        const Eigen::Vector2d p = pc.head<2>();
        const double dis = p.norm();
        if (dis > options.usable_scan_range) {
            continue;
        }

        const Eigen::Vector4d pn(pc.x(), pc.y(), pc.z(), 1.0);
        const double dis_floor = pn.dot(floor_coeffs);
        const double dangle = std::atan2(p.y(), p.x()) * kRad2Deg;
        const int angle =
            static_cast<int>(std::lround(dangle) + 360) % 360;

        if (dis_floor > min_th) {
            if (dis_floor < max_th) {
                (*rays)[angle].emplace_back(dis, dis_floor);
                const Eigen::Vector3d p_world = T_w_b * pc;
                map->SetHitPointUnlocked(static_cast<float>(p_world.x()),
                                         static_cast<float>(p_world.y()), true,
                                         static_cast<float>(dis_floor));
            }
        } else if (dis_floor > -min_th) {
            (*rays)[angle].emplace_back(dis, dis_floor);
        }
    }
}

void SelectRayEndpoints(
    std::vector<std::vector<std::pair<double, double>>>* rays, double min_th,
    double floor_rh, std::vector<Eigen::Vector2d>* angle_distance_height) {
    constexpr double kDefaultRayDistance = -1.0;
    for (int i = 0; i < 360; ++i) {
        auto& bucket = (*rays)[static_cast<std::size_t>(i)];
        if (bucket.size() < 2) {
            (*angle_distance_height)[static_cast<std::size_t>(i)] =
                Eigen::Vector2d(kDefaultRayDistance, floor_rh);
            continue;
        }
        std::sort(bucket.begin(), bucket.end(),
                  [](const std::pair<double, double>& a,
                     const std::pair<double, double>& b) {
                      return a.first < b.first;
                  });
        for (auto iter = bucket.rbegin(); iter != bucket.rend(); ++iter) {
            if (iter->second < min_th) {
                (*angle_distance_height)[static_cast<std::size_t>(i)] =
                    Eigen::Vector2d(iter->first, iter->second);
                continue;
            }
            auto next_iter = iter;
            ++next_iter;
            if (next_iter != bucket.rend()) {
                if (iter->second > min_th && next_iter->second < min_th) {
                    (*angle_distance_height)[static_cast<std::size_t>(i)] =
                        Eigen::Vector2d(iter->first, iter->second);
                    break;
                }
            } else {
                (*angle_distance_height)[static_cast<std::size_t>(i)] =
                    Eigen::Vector2d(iter->first, iter->second);
            }
        }
    }
}

void ApplyRayMisses(const transform::Rigid3d& T_w_b,
                    const std::vector<Eigen::Vector2d>& angle_distance_height,
                    const OccupancyGridBuilderOptions& options,
                    OccupancyGridMap* map) {
    const Eigen::Vector3d orig = T_w_b.translation();
    for (int i = 0; i < 360; ++i) {
        const double angle = static_cast<double>(i) * kDeg2Rad;
        const float r =
            static_cast<float>(angle_distance_height[static_cast<std::size_t>(i)][0]);
        const float h =
            static_cast<float>(angle_distance_height[static_cast<std::size_t>(i)][1]);
        const Eigen::Vector3d p_local(r * std::cos(angle), r * std::sin(angle),
                                      h);
        const Eigen::Vector3d p_world = T_w_b * p_local;

        if (r <= 0.f || r > options.usable_scan_range) {
            if (r < 0.1f) {
                map->SetMissPointUnlocked(
                    static_cast<float>(p_world.x()),
                    static_cast<float>(p_world.y()),
                    static_cast<float>(orig.x()),
                    static_cast<float>(orig.y()), h,
                    static_cast<float>(options.lidar_height));
            }
            continue;
        }
        map->SetMissPointUnlocked(
            static_cast<float>(p_world.x()), static_cast<float>(p_world.y()),
            static_cast<float>(orig.x()), static_cast<float>(orig.y()), h,
            static_cast<float>(options.lidar_height));
    }
}

}  // namespace

OccupancyGridBuilder::OccupancyGridBuilder(
    OccupancyGridBuilderOptions options)
    : options_(std::move(options)) {}

void OccupancyGridBuilder::Reset() {
    map_.reset();
}

void OccupancyGridBuilder::AddScan(const transform::Rigid3d& T_w_b,
                                   const sensor::PointCloudXYZI& cloud_b,
                                   bool cloud_in_world) {
    if (cloud_b.empty()) {
        return;
    }
    if (!EnsureBounds(T_w_b, cloud_b, cloud_in_world)) {
        return;
    }
    ProjectScan(T_w_b, cloud_b, cloud_in_world);
}

bool OccupancyGridBuilder::EnsureBounds(
    const transform::Rigid3d& T_w_b, const sensor::PointCloudXYZI& cloud_b,
    bool cloud_in_world) {
    float min_x = static_cast<float>(T_w_b.translation().x());
    float min_y = static_cast<float>(T_w_b.translation().y());
    float max_x = min_x;
    float max_y = min_y;

    if (map_ != nullptr && !map_->empty()) {
        map_->GetBounds(&min_x, &min_y, &max_x, &max_y);
    }

    const float margin = 1.f;
    for (std::size_t i = 0; i < cloud_b.size(); i += 10) {
        const Eigen::Vector3d p = PointBody(cloud_b, i);
        const Eigen::Vector3d pw = cloud_in_world ? p : (T_w_b * p);
        const Eigen::Vector3d pb =
            cloud_in_world ? (T_w_b.inverse() * p) : p;
        const double range = pb.head<2>().norm();
        if (range > options_.usable_scan_range || range <= 0.01 ||
            !std::isfinite(range)) {
            continue;
        }
        min_x = std::min(min_x, static_cast<float>(pw.x()) - margin);
        min_y = std::min(min_y, static_cast<float>(pw.y()) - margin);
        max_x = std::max(max_x, static_cast<float>(pw.x()) + margin);
        max_y = std::max(max_y, static_cast<float>(pw.y()) + margin);
    }

    if (min_x > max_x || min_y > max_y) {
        return false;
    }

    const float r = options_.map.resolution;
    min_x = std::floor(min_x / r) * r;
    min_y = std::floor(min_y / r) * r;
    max_x = std::ceil(max_x / r) * r;
    max_y = std::ceil(max_y / r) * r;

    if (map_ == nullptr || map_->empty()) {
        map_ = OccupancyGridMap::make_shared(options_.map);
        return map_->Init(min_x, min_y, max_x, max_y);
    }

    float cur_min_x = 0.f;
    float cur_min_y = 0.f;
    float cur_max_x = 0.f;
    float cur_max_y = 0.f;
    map_->GetBounds(&cur_min_x, &cur_min_y, &cur_max_x, &cur_max_y);
    if (min_x < cur_min_x || min_y < cur_min_y || max_x > cur_max_x ||
        max_y > cur_max_y) {
        min_x = std::min(min_x, cur_min_x);
        min_y = std::min(min_y, cur_min_y);
        max_x = std::max(max_x, cur_max_x);
        max_y = std::max(max_y, cur_max_y);
        return map_->Resize(min_x, min_y, max_x, max_y);
    }
    return true;
}

void OccupancyGridBuilder::ProjectScan(
    const transform::Rigid3d& T_w_b, const sensor::PointCloudXYZI& cloud_b,
    bool cloud_in_world) {
    // Floor plane: z + d = 0 with d = -default_floor_height → height = z - floor.
    const Eigen::Vector4d floor_coeffs(0.0, 0.0, 1.0,
                                       -options_.default_floor_height);
    const double floor_rh = floor_coeffs[3];

    // Per-angle buckets (distance, height); sorted after the scan (no map).
    std::vector<std::vector<std::pair<double, double>>> rays(360);
    std::vector<Eigen::Vector2d> angle_distance_height(
        360, Eigen::Vector2d::Zero());

    FillRaysFromCloud(T_w_b, cloud_b, cloud_in_world, options_, floor_coeffs,
                      map_.get(), &rays);
    SelectRayEndpoints(&rays, options_.min_th_floor, floor_rh,
                       &angle_distance_height);
    ApplyRayMisses(T_w_b, angle_distance_height, options_, map_.get());
}

}  // namespace map
}  // namespace argus
