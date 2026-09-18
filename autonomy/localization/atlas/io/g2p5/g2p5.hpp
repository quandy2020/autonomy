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

#include "autonomy/localization/atlas/io/g2p5/g2p5_map.hpp"
#include "autonomy/localization/atlas/type.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace autonomy::localization::atlas {
namespace map {

//! Atlas keyframe for G2P5 (no lightning Keyframe::Ptr).
struct G2P5Keyframe {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::uint64_t id = 0;
    Mat44_t T_wb = Mat44_t::Identity();
    std::vector<Vec3_t> points_body;  // lidar body frame
};
using G2P5KeyframePtr = std::shared_ptr<G2P5Keyframe>;

/**
 * 2.5D occupancy from lidar rays (display / nav export, not localization).
 *
 * Front: PushKeyframe → incremental map. Back: RedrawGlobalMap after loop.
 */
class G2P5 {
public:
    struct Options {
        Options() {}

        bool online_mode_ = true;
        bool esti_floor_ = false;

        double lidar_height_ = 0.0;
        double default_floor_height_ = -1.0;
        double min_th_floor_ = 0.5;
        double max_th_floor_ = 1.2;
        float usable_scan_range_ = 50.0f;

        double grid_map_resolution_ = 0.1;
        bool verbose_ = false;
    };

    G2P5();
    explicit G2P5(Options options);
    ~G2P5();

    G2P5(const G2P5&) = delete;
    G2P5& operator=(const G2P5&) = delete;

    using MapUpdateCallback = std::function<void(G2P5MapPtr map)>;

    //! Start frontend/backend threads; optionally load Options from yaml.
    void Init(const std::string& yaml_path = "");

    void SetMapUpdateCallback(MapUpdateCallback func) {
        map_update_cb_ = std::move(func);
    }

    void Quit();

    void PushKeyframe(G2P5KeyframePtr kf);
    void RedrawGlobalMap();

    G2P5MapPtr GetNewestMap();

    bool SetParallelRendering(bool enable_parallel = false) {
        parallel_render_ = enable_parallel;
        return true;
    }

    bool IsBusy() const { return is_busy_; }

    const Options& options() const { return options_; }
    Options& mutable_options() { return options_; }

private:
    bool AddKfToMap(const std::vector<G2P5KeyframePtr>& kfs, G2P5MapPtr& map);
    bool ResizeMap(const std::vector<G2P5KeyframePtr>& kfs, G2P5MapPtr& map);
    void RenderFront(G2P5KeyframePtr kf);
    void RenderBack();
    void FrontendLoop();
    void Convert3DTo2DScan(G2P5KeyframePtr kf, G2P5MapPtr& map);
    bool DetectPlaneCoeffs(G2P5KeyframePtr kf);
    void SetWhitePoints(const std::vector<Vec2_t>& ang_distance_height,
                        G2P5KeyframePtr kf, G2P5MapPtr& map);

    static Vec3_t TransformBody(const Mat44_t& T_wb, const Vec3_t& p) {
        return T_wb.block<3, 3>(0, 0) * p + T_wb.block<3, 1>(0, 3);
    }

    Options options_;

    std::atomic_bool parallel_render_{false};
    std::atomic_bool is_busy_{false};
    MapUpdateCallback map_update_cb_;
    std::atomic_bool quit_flag_{false};
    std::atomic_bool started_{false};

    std::mutex kf_mutex_;
    std::vector<G2P5KeyframePtr> all_keyframes_;

    std::mutex newest_map_mutex_;
    G2P5MapPtr newest_map_;

    std::mutex frontend_mutex_;
    G2P5MapPtr frontend_map_;
    G2P5KeyframePtr frontend_current_;

    std::thread frontend_thread_;
    std::mutex frontend_queue_mutex_;
    std::condition_variable frontend_cv_;
    std::deque<G2P5KeyframePtr> frontend_queue_;

    std::thread draw_backend_map_thread_;
    std::atomic_bool backend_redraw_flag_{false};
    G2P5MapPtr backend_map_;

    Vec4_t floor_coeffs_ = Vec4_t(0, 0, 1.0, 1.0);
};

}  // namespace map
}  // namespace autonomy::localization::atlas
