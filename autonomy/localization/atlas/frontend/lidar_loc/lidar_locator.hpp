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

//! Pure-lidar localization against a prior tiled/PCD map (NOT a second SLAM).
//! NDT-aligns scans to local tiles via TiledMap::LoadOnPose; pose authority
//! still flows through LocalEstimator when wired.

#include "autonomy/localization/atlas/mapping/tiled_map.hpp"
#include "autonomy/localization/atlas/type.hpp"

#include <memory>
#include <string>
#include <vector>

namespace autonomy::localization::atlas {
namespace frontend {

class LidarLocator {
public:
    struct Options {
        double load_radius_m = 80.0;
        float ndt_resolution = 1.0f;
        float ndt_step_size = 0.1f;
        float ndt_trans_eps = 0.01f;
        int ndt_max_iter = 30;
        float voxel_leaf = 0.5f;
        double min_score = 0.0;  // optional; 0 = accept any
        std::size_t min_map_points = 500;
        std::size_t min_scan_points = 50;
        //! After successful Align, write scan into TiledMap dyn layer.
        bool update_dynamic_cloud = false;
        //! Min translation (m) since last dyn update (OR with update_kf_time_s).
        double update_kf_dis_m = 2.0;
        //! Min time (s) since last dyn update.
        double update_kf_time_s = 10.0;
        //! PCL NDT fitness upper bound (lower=better); update only if score ≤.
        double update_max_fitness = 2.5;
        //! Body-frame z band for dyn insert (Lightning pass-through).
        double dyn_z_min = 0.5;
        double dyn_z_max = 30.0;
        //! Dyn retention when enabling layer via update (short|medium|permanent).
        mapping::DynPolicy dyn_policy = mapping::DynPolicy::kMedium;
    };

    LidarLocator() = default;
    explicit LidarLocator(Options options) : options_(std::move(options)) {}

    void set_options(Options options) { options_ = std::move(options); }
    [[nodiscard]] const Options& options() const { return options_; }

    //! Borrow pipeline TiledMap (non-owning) or own one after LoadMap.
    void set_tiled_map(mapping::TiledMap* map) { map_ = map; }
    mapping::TiledMap* tiled_map() { return map_; }

    //! Load TiledMap index.yaml+PCD dir (or create owned map).
    bool LoadMap(const std::string& dir);

    //! NDT align body-frame scan to local map around guess; write T_wb.
    //! Returns false if map empty / NDT unavailable / align failed.
    bool Align(const std::vector<Vec3_t>& points_body,
               const Mat44_t& T_wb_guess, Mat44_t* T_wb_out,
               double* score_out = nullptr);

    //! Incremental dyn map after successful loc (gated by distance/time/score).
    //! Returns number of dyn points inserted (0 if skipped).
    int MaybeUpdateDynamic(double t_sec, const Mat44_t& T_wb,
                           const std::vector<Vec3_t>& points_body,
                           double fitness_score);

    [[nodiscard]] bool map_ready() const {
        return map_ != nullptr && map_->num_tiles() > 0;
    }

private:
    Options options_;
    mapping::TiledMap* map_ = nullptr;
    std::unique_ptr<mapping::TiledMap> owned_map_;
    bool have_last_dyn_upd_ = false;
    double last_dyn_upd_t_ = -1.0;
    Vec3_t last_dyn_upd_pos_ = Vec3_t::Zero();
};

}  // namespace frontend
}  // namespace autonomy::localization::atlas
