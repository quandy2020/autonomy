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

#pragma once

//! mapping/map_incremental — Mapping owns the live IVox; LidarSensor borrows
//! a non-owning pointer via LidarSensor::set_ivox(&MapIncremental::ivox()).
//! Insert path: LidarBridge → UpdateLidar → IntegrateScan (corrected T_wb).
//! Optional TiledMap* mirrors world points for chunked PCD persistence.

#include "autonomy/localization/atlas/mapping/ivox/ivox.hpp"
#include "autonomy/localization/atlas/mapping/tiled_map.hpp"
#include "autonomy/localization/atlas/type.hpp"

#include <cmath>
#include <vector>

namespace autonomy::localization::atlas {
namespace mapping {

class MapIncremental {
public:
    struct Options {
        //! lightning MapIncremental mid-point density gate (default on).
        bool selective_insert = true;
        //! Neighbors required before density veto (lightning NUM_MATCH_POINTS).
        int num_match_points = 5;
        //! Cap points added when no neighbor found (new FOV flood during spin).
        int max_insert_no_nn = 600;
        //! Hard cap on inserted points per scan.
        int max_insert_per_scan = 2500;
        //! Lidar→IMU extrinsic for PointBodyToWorld (lightning offset_*).
        Mat44_t T_imu_lidar = Mat44_t::Identity();
    };

    MapIncremental() = default;
    explicit MapIncremental(mapping::IVox::Options opts)
        : ivox_(opts) {}
    MapIncremental(mapping::IVox::Options opts, Options map_opts)
        : ivox_(opts), options_(std::move(map_opts)) {}

    void set_options(Options options) { options_ = std::move(options); }
    [[nodiscard]] const Options& options() const { return options_; }

    void set_tiled_map(TiledMap* tiled) { tiled_map_ = tiled; }
    TiledMap* tiled_map() { return tiled_map_; }
    const TiledMap* tiled_map() const { return tiled_map_; }

    void InsertWorldPoints(const std::vector<Vec3_t>& pts) {
        ivox_.InsertWorldPoints(pts);
        if (tiled_map_) {
            tiled_map_->InsertWorldPoints(pts);
        }
    }

    //! Transform body points to world and insert into IVox (+ optional TiledMap).
    //! When selective_insert: skip points whose voxel is already denser than the
    //! candidate (lightning LaserMapping::MapIncremental).
    int IntegrateScan(const Mat44_t& T_wb, const std::vector<Vec3_t>& points_body) {
        if (points_body.empty()) {
            return 0;
        }
        const Mat33_t R_wb = T_wb.block<3, 3>(0, 0);
        const Vec3_t t_wb = T_wb.block<3, 1>(0, 3);
        const Mat33_t R_il = options_.T_imu_lidar.block<3, 3>(0, 0);
        const Vec3_t t_il = options_.T_imu_lidar.block<3, 1>(0, 3);
        const bool use_ext =
            !R_il.isIdentity(1e-12) || t_il.norm() > 1e-12;
        const double res = std::max(1e-3, ivox_.options().resolution);
        const int knn = std::max(1, options_.num_match_points);

        std::vector<Vec3_t> points_world;
        points_world.reserve(points_body.size());
        int no_nn = 0;
        const int max_no_nn = std::max(0, options_.max_insert_no_nn);
        const int max_scan = std::max(1, options_.max_insert_per_scan);

        for (const auto& p : points_body) {
            if (static_cast<int>(points_world.size()) >= max_scan) {
                break;
            }
            if (!p.allFinite()) {
                continue;
            }
            // lightning: p_w = R*(R_il*p + t_il) + t
            const Vec3_t p_imu = use_ext ? Vec3_t(R_il * p + t_il) : p;
            const Vec3_t pw = R_wb * p_imu + t_wb;
            if (!options_.selective_insert || ivox_.num_points() == 0) {
                points_world.push_back(pw);
                continue;
            }

            std::vector<Vec3_t> near;
            if (!ivox_.GetClosestPoints(pw, &near, knn, 5.0 * res) ||
                near.empty()) {
                if (no_nn < max_no_nn) {
                    points_world.push_back(pw);
                    ++no_nn;
                }
                continue;
            }

            // Voxel center (lightning mid-point).
            const Vec3_t center =
                ((pw.array() / res).floor() + 0.5) * res;
            const Vec3_t dis = near[0] - center;
            if (std::fabs(dis.x()) > 0.5 * res &&
                std::fabs(dis.y()) > 0.5 * res &&
                std::fabs(dis.z()) > 0.5 * res) {
                // Nearest is far from center → always add (no downsample).
                points_world.push_back(pw);
                continue;
            }

            bool need_add = true;
            const double dist = (pw - center).squaredNorm();
            if (static_cast<int>(near.size()) >= knn) {
                for (int i = 0; i < knn; ++i) {
                    if ((near[static_cast<std::size_t>(i)] - center)
                            .squaredNorm() < dist + 1e-12) {
                        need_add = false;
                        break;
                    }
                }
            }
            if (need_add) {
                points_world.push_back(pw);
            }
        }

        ivox_.InsertWorldPoints(points_world);
        if (tiled_map_) {
            tiled_map_->InsertWorldPoints(points_world);
        }
        return static_cast<int>(points_world.size());
    }

    //! Clear live map and rebuild from keyframe body clouds at corrected T_wb
    //! (call after lidar pose-graph Optimize).
    template <typename KeyframeRange>
    int RebuildFromKeyframes(const KeyframeRange& keyframes) {
        Clear();
        // KF clouds are already downsampled; skip selective so rebuild is dense.
        const bool prev_sel = options_.selective_insert;
        const int prev_scan = options_.max_insert_per_scan;
        const int prev_nn = options_.max_insert_no_nn;
        options_.selective_insert = false;
        options_.max_insert_per_scan = 100000;
        options_.max_insert_no_nn = 100000;
        int n = 0;
        for (const auto& kf : keyframes) {
            n += IntegrateScan(kf.T_wb, kf.cloud_body);
        }
        options_.selective_insert = prev_sel;
        options_.max_insert_per_scan = prev_scan;
        options_.max_insert_no_nn = prev_nn;
        return n;
    }

    void Clear() {
        ivox_.Clear();
        if (tiled_map_) {
            tiled_map_->Clear();
        }
    }

    mapping::IVox& ivox() { return ivox_; }
    const mapping::IVox& ivox() const { return ivox_; }

private:
    //! Canonical live local lidar map (owned here, not by LidarSensor).
    mapping::IVox ivox_;
    Options options_;
    //! Optional chunked map (owned by Pipeline); non-owning.
    TiledMap* tiled_map_ = nullptr;
};

}  // namespace mapping
}  // namespace autonomy::localization::atlas
