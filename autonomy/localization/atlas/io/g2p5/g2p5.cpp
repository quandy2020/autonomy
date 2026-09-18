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

#include "autonomy/localization/atlas/io/g2p5/g2p5.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "glog/logging.h"
#include "yaml-cpp/yaml.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace autonomy::localization::atlas {
namespace map {
namespace {

constexpr double kRad2Deg = 180.0 / M_PI;
constexpr double kDeg2Rad = M_PI / 180.0;

}  // namespace

G2P5::G2P5() : G2P5(Options{}) {}

G2P5::G2P5(Options options) : options_(std::move(options)) {}

G2P5::~G2P5() { Quit(); }

void G2P5::Quit() {
    quit_flag_ = true;
    frontend_cv_.notify_all();
    backend_redraw_flag_ = true;

    if (frontend_thread_.joinable()) {
        frontend_thread_.join();
    }
    if (draw_backend_map_thread_.joinable()) {
        draw_backend_map_thread_.join();
    }
    started_ = false;
}

void G2P5::PushKeyframe(G2P5KeyframePtr kf) {
    if (!kf) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(kf_mutex_);
        all_keyframes_.emplace_back(kf);
    }

    if (options_.online_mode_ && started_) {
        {
            std::lock_guard<std::mutex> lock(frontend_queue_mutex_);
            frontend_queue_.push_back(kf);
        }
        frontend_cv_.notify_one();
    } else {
        RenderFront(kf);
    }
}

void G2P5::RenderFront(G2P5KeyframePtr kf) {
    {
        std::lock_guard<std::mutex> lock(frontend_mutex_);
        frontend_current_ = kf;
    }

    {
        std::lock_guard<std::mutex> lock(newest_map_mutex_);
        AddKfToMap({kf}, frontend_map_);
        newest_map_ = frontend_map_;
    }

    if (map_update_cb_) {
        map_update_cb_(newest_map_);
    }
}

void G2P5::RedrawGlobalMap() { backend_redraw_flag_ = true; }

void G2P5::FrontendLoop() {
    while (!quit_flag_) {
        G2P5KeyframePtr kf;
        {
            std::unique_lock<std::mutex> lock(frontend_queue_mutex_);
            frontend_cv_.wait_for(lock, std::chrono::milliseconds(100), [&] {
                return quit_flag_ || !frontend_queue_.empty();
            });
            if (quit_flag_) {
                break;
            }
            if (frontend_queue_.empty()) {
                continue;
            }
            kf = frontend_queue_.front();
            frontend_queue_.pop_front();
        }
        RenderFront(std::move(kf));
    }
}

void G2P5::RenderBack() {
    while (!quit_flag_) {
        while (!backend_redraw_flag_ && !quit_flag_) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        if (quit_flag_) {
            break;
        }

        is_busy_ = true;
        backend_redraw_flag_ = false;

        std::vector<G2P5KeyframePtr> all_keyframes;
        {
            std::lock_guard<std::mutex> lock(kf_mutex_);
            all_keyframes = all_keyframes_;
        }

        if (all_keyframes.empty()) {
            is_busy_ = false;
            continue;
        }

        G2P5Map::Options opt;
        opt.resolution_ =
            static_cast<float>(options_.grid_map_resolution_);
        backend_map_ = std::make_shared<G2P5Map>(opt);
        bool abort = false;

        for (const auto& kf : all_keyframes) {
            AddKfToMap({kf}, backend_map_);
            if (backend_redraw_flag_ || quit_flag_) {
                LOG(INFO) << "G2P5: backend redraw aborted";
                abort = true;
                break;
            }
        }

        if (abort) {
            is_busy_ = false;
            continue;
        }

        // Catch up with frontend keyframes added during redraw.
        std::uint64_t cur_idx = all_keyframes.back()->id;
        while (!quit_flag_) {
            G2P5KeyframePtr frontend_kf;
            {
                std::lock_guard<std::mutex> lock(frontend_mutex_);
                frontend_kf = frontend_current_;
            }
            if (!frontend_kf || cur_idx == frontend_kf->id) {
                break;
            }

            const std::uint64_t frontend_idx = frontend_kf->id;
            std::vector<G2P5KeyframePtr> kfs;
            {
                std::lock_guard<std::mutex> lock(kf_mutex_);
                for (const auto& kf : all_keyframes_) {
                    if (kf->id > cur_idx && kf->id <= frontend_idx) {
                        kfs.push_back(kf);
                    }
                }
            }
            if (!kfs.empty()) {
                AddKfToMap(kfs, backend_map_);
            }
            cur_idx = frontend_idx;
        }

        {
            std::lock_guard<std::mutex> lock(newest_map_mutex_);
            frontend_map_ = backend_map_;
            newest_map_ = frontend_map_;
        }

        if (map_update_cb_) {
            map_update_cb_(newest_map_);
        }
        is_busy_ = false;
    }
    LOG(INFO) << "G2P5: backend render quit";
}

bool G2P5::ResizeMap(const std::vector<G2P5KeyframePtr>& kfs,
                     G2P5MapPtr& map) {
    if (!map) {
        G2P5Map::Options opt;
        opt.resolution_ =
            static_cast<float>(options_.grid_map_resolution_);
        map = std::make_shared<G2P5Map>(opt);
    }

    float init_min_x, init_min_y, init_max_x, init_max_y;
    map->GetMinAndMax(init_min_x, init_min_y, init_max_x, init_max_y);
    float min_x = init_min_x;
    float min_y = init_min_y;
    float max_x = init_max_x;
    float max_y = init_max_y;

    for (const auto& kf : kfs) {
        if (quit_flag_) {
            return true;
        }
        const Vec3_t t = kf->T_wb.block<3, 1>(0, 3);
        min_x = std::min(min_x, static_cast<float>(t.x()));
        min_y = std::min(min_y, static_cast<float>(t.y()));
        max_x = std::max(max_x, static_cast<float>(t.x()));
        max_y = std::max(max_y, static_cast<float>(t.y()));

        const auto& cloud = kf->points_body;
        for (std::size_t i = 0; i < cloud.size(); i += 10) {
            const float range = static_cast<float>(cloud[i].norm());
            if (range > options_.usable_scan_range_ || range <= 0.01f ||
                std::isnan(range)) {
                continue;
            }
            const Vec3_t point = TransformBody(kf->T_wb, cloud[i]);
            min_x = std::min(min_x, static_cast<float>(point.x() - 1.0));
            min_y = std::min(min_y, static_cast<float>(point.y() - 1.0));
            max_x = std::max(max_x, static_cast<float>(point.x() + 1.0));
            max_y = std::max(max_y, static_cast<float>(point.y() + 1.0));
        }
    }

    if (min_x > max_x || min_y > max_y) {
        return false;
    }

    if (map->IsEmpty() || min_x < init_min_x || min_y < init_min_y ||
        max_x > init_max_x || max_y > init_max_y) {
        if (!map->IsEmpty()) {
            min_x = (min_x > init_min_x) ? init_min_x : min_x;
            min_y = (min_y > init_min_y) ? init_min_y : min_y;
            max_x = (max_x < init_max_x) ? init_max_x : max_x;
            max_y = (max_y < init_max_y) ? init_max_y : max_y;
        }

        const float r = map->GetGridResolution();
        min_x = static_cast<float>(std::floor(min_x / r)) * r;
        min_y = static_cast<float>(std::floor(min_y / r)) * r;
        max_x = static_cast<float>(std::ceil(max_x / r)) * r;
        max_y = static_cast<float>(std::ceil(max_y / r)) * r;
        map->Resize(min_x, min_y, max_x, max_y);
    }
    return true;
}

bool G2P5::AddKfToMap(const std::vector<G2P5KeyframePtr>& kfs,
                      G2P5MapPtr& map) {
    ResizeMap(kfs, map);
    for (const auto& kf : kfs) {
        Convert3DTo2DScan(kf, map);
    }
    return true;
}

G2P5MapPtr G2P5::GetNewestMap() {
    std::lock_guard<std::mutex> lock(newest_map_mutex_);
    if (newest_map_ == nullptr) {
        return nullptr;
    }
    return newest_map_->MakeDeepCopy();
}

void G2P5::Convert3DTo2DScan(G2P5KeyframePtr kf, G2P5MapPtr& map) {
    if (!kf || !map || map->IsEmpty()) {
        return;
    }

    if (options_.esti_floor_) {
        if (!DetectPlaneCoeffs(kf)) {
            floor_coeffs_ =
                Vec4_t(0, 0, 1, -options_.default_floor_height_);
        } else if (options_.verbose_) {
            LOG(INFO) << "G2P5 floor coeffs: " << floor_coeffs_.transpose();
        }
    } else {
        floor_coeffs_ = Vec4_t(0, 0, 1, -options_.default_floor_height_);
    }

    std::vector<std::map<double, double>> rays(360);
    std::vector<Vec2_t> angle_distance_height(360, Vec2_t::Zero());

    const Mat44_t& Twb = kf->T_wb;
    const double min_th = options_.min_th_floor_;
    const double max_th = options_.max_th_floor_;

    for (const auto& pt : kf->points_body) {
        if (quit_flag_) {
            return;
        }
        const Vec3_t pc = pt;
        const Vec4_t pn(pt.x(), pt.y(), pt.z(), 1.0);
        const Vec2_t p = pc.head<2>();
        const double dis = p.norm();
        if (dis > options_.usable_scan_range_) {
            continue;
        }

        const double dis_floor = pn.dot(floor_coeffs_);
        const double dangle = std::atan2(p[1], p[0]) * kRad2Deg;
        const int angle =
            static_cast<int>(std::round(dangle) + 360) % 360;

        if (dis_floor > min_th) {
            if (dis_floor < max_th) {
                rays[static_cast<std::size_t>(angle)].insert(
                    {dis, dis_floor});
                const Vec3_t p_world = TransformBody(Twb, pc);
                map->SetHitPoint(static_cast<float>(p_world[0]),
                                 static_cast<float>(p_world[1]), true,
                                 static_cast<float>(dis_floor));
            }
        } else if (dis_floor > -min_th) {
            rays[static_cast<std::size_t>(angle)].insert({dis, dis_floor});
        }
    }

    constexpr double default_ray_distance = -1.0;
    const double floor_rh = floor_coeffs_[3];

    for (int i = 0; i < 360; ++i) {
        if (quit_flag_) {
            return;
        }
        auto& ray = rays[static_cast<std::size_t>(i)];
        if (ray.size() < 2) {
            angle_distance_height[static_cast<std::size_t>(i)] =
                Vec2_t(default_ray_distance, floor_rh);
            continue;
        }
        for (auto iter = ray.rbegin(); iter != ray.rend(); ++iter) {
            if (iter->second < options_.min_th_floor_) {
                angle_distance_height[static_cast<std::size_t>(i)] =
                    Vec2_t(iter->first, iter->second);
                continue;
            }
            auto next_iter = iter;
            ++next_iter;
            if (next_iter != ray.rend()) {
                if (iter->second > options_.min_th_floor_ &&
                    next_iter->second < options_.min_th_floor_) {
                    angle_distance_height[static_cast<std::size_t>(i)] =
                        Vec2_t(iter->first, iter->second);
                    break;
                }
            } else {
                angle_distance_height[static_cast<std::size_t>(i)] =
                    Vec2_t(iter->first, iter->second);
            }
        }
    }

    SetWhitePoints(angle_distance_height, kf, map);
}

void G2P5::SetWhitePoints(const std::vector<Vec2_t>& pt2d,
                          G2P5KeyframePtr kf, G2P5MapPtr& map) {
    if (pt2d.size() != 360 || !kf || !map) {
        return;
    }

    const Mat44_t& pose = kf->T_wb;
    const Vec3_t orig = pose.block<3, 1>(0, 3);

    for (int i = 0; i < 360; ++i) {
        if (quit_flag_) {
            return;
        }
        const double angle = static_cast<double>(i) * kDeg2Rad;
        const float r = static_cast<float>(pt2d[static_cast<std::size_t>(i)][0]);
        const float h = static_cast<float>(pt2d[static_cast<std::size_t>(i)][1]);

        const Vec3_t p_local(r * std::cos(angle), r * std::sin(angle), h);
        const Vec3_t p_world = TransformBody(pose, p_local);

        if (r <= 0.f || r > options_.usable_scan_range_) {
            if (r < 0.1f) {
                map->SetMissPoint(
                    static_cast<float>(p_world[0]),
                    static_cast<float>(p_world[1]),
                    static_cast<float>(orig[0]),
                    static_cast<float>(orig[1]), h,
                    static_cast<float>(options_.lidar_height_));
            }
            continue;
        }

        map->SetMissPoint(static_cast<float>(p_world[0]),
                          static_cast<float>(p_world[1]),
                          static_cast<float>(orig[0]),
                          static_cast<float>(orig[1]), h,
                          static_cast<float>(options_.lidar_height_));
    }
}

bool G2P5::DetectPlaneCoeffs(G2P5KeyframePtr kf) {
    if (!kf) {
        return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.reserve(kf->points_body.size());
    const float z_max = static_cast<float>(options_.lidar_height_ +
                                           options_.default_floor_height_);
    for (const auto& pt : kf->points_body) {
        if (pt.z() < z_max) {
            cloud->points.emplace_back(static_cast<float>(pt.x()),
                                       static_cast<float>(pt.y()),
                                       static_cast<float>(pt.z()));
        }
    }
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = false;

    if (cloud->size() < 200) {
        return false;
    }

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.25);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (coefficients->values.size() < 4) {
        return false;
    }
    if (coefficients->values[2] < 0.99f) {
        LOG(ERROR) << "G2P5: floor is not horizontal";
        return false;
    }
    if (inliers->indices.size() < 100) {
        LOG(ERROR) << "G2P5: not enough floor inliers: "
                   << inliers->indices.size();
        return false;
    }

    for (int i = 0; i < 4; ++i) {
        floor_coeffs_[i] = coefficients->values[static_cast<std::size_t>(i)];
    }
    return true;
}

void G2P5::Init(const std::string& yaml_path) {
    if (started_) {
        return;
    }

    if (!yaml_path.empty()) {
        try {
            const YAML::Node yaml = YAML::LoadFile(yaml_path);
            if (yaml["g2p5"]) {
                const auto& g = yaml["g2p5"];
                options_.esti_floor_ =
                    g["esti_floor"].as<bool>(options_.esti_floor_);
                options_.min_th_floor_ =
                    g["min_th_floor"].as<double>(options_.min_th_floor_);
                options_.max_th_floor_ =
                    g["max_th_floor"].as<double>(options_.max_th_floor_);
                options_.lidar_height_ =
                    g["lidar_height"].as<double>(options_.lidar_height_);
                options_.grid_map_resolution_ =
                    g["grid_map_resolution"].as<double>(
                        options_.grid_map_resolution_);
                options_.default_floor_height_ =
                    g["floor_height"].as<double>(
                        options_.default_floor_height_);
            }
        } catch (const std::exception& e) {
            LOG(WARNING) << "G2P5: yaml load failed: " << e.what();
        }
    }

    quit_flag_ = false;
    G2P5Map::Options opt;
    opt.resolution_ = static_cast<float>(options_.grid_map_resolution_);
    frontend_map_ = std::make_shared<G2P5Map>(opt);

    if (options_.online_mode_) {
        frontend_thread_ = std::thread([this]() { FrontendLoop(); });
    }
    draw_backend_map_thread_ = std::thread([this]() { RenderBack(); });
    started_ = true;
}

}  // namespace map
}  // namespace autonomy::localization::atlas
