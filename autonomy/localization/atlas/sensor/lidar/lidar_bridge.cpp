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

#include "autonomy/localization/atlas/sensor/lidar/lidar_bridge.hpp"

#include "autonomy/localization/atlas/util/map_publisher.hpp"
#include "autonomy/localization/atlas/system.hpp"
#include "autonomy/localization/atlas/type.hpp"
#include "autonomy/localization/atlas/viz_bridge.hpp"

#include <automsgs/msgs/sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>

#include "glog/logging.h"

namespace autonomy::localization::atlas {
namespace {

bool HasField(const automsgs::msgs::sensor_msgs::PointCloud2& pc2,
              const std::string& name) {
    for (const auto& f : pc2.fields()) {
        if (f.name() == name) {
            return true;
        }
    }
    return false;
}

double StampSec(const automsgs::msgs::sensor_msgs::PointCloud2& msg) {
    if (!msg.has_header() || !msg.header().has_stamp()) {
        return 0.0;
    }
    const auto& s = msg.header().stamp();
    return static_cast<double>(s.sec()) +
           1e-9 * static_cast<double>(s.nanosec());
}

//! Decode optional per-point time field ("t", "time", or "offset_time") into [0,1].
//! `offset_time` is Livox CustomMsg-style (often ns); values are min-max normalized.
bool DecodePointTimes(
    const automsgs::msgs::sensor_msgs::PointCloud2& msg,
    std::size_t n_decoded,
    std::vector<double>* times_rel) {
    if (!times_rel) {
        return false;
    }
    times_rel->clear();
    const bool has_t = HasField(msg, "t");
    const bool has_time = HasField(msg, "time");
    const bool has_offset = HasField(msg, "offset_time");
    if (!has_t && !has_time && !has_offset) {
        return false;
    }
    using automsgs::msgs::sensor_msgs::PointCloud2ConstIterator;
    std::vector<double> raw;
    raw.reserve(n_decoded);
    const std::size_t n =
        static_cast<std::size_t>(msg.width()) *
        static_cast<std::size_t>(msg.height());
    try {
        if (has_t) {
            PointCloud2ConstIterator<float> it(msg, "t");
            for (std::size_t i = 0; i < n && raw.size() < n_decoded;
                 ++i, ++it) {
                raw.push_back(static_cast<double>(*it));
            }
        } else if (has_time) {
            PointCloud2ConstIterator<float> it(msg, "time");
            for (std::size_t i = 0; i < n && raw.size() < n_decoded;
                 ++i, ++it) {
                raw.push_back(static_cast<double>(*it));
            }
        } else {
            // Livox-style offset_time: prefer uint32 ns, fall back to float.
            try {
                PointCloud2ConstIterator<std::uint32_t> it(msg, "offset_time");
                for (std::size_t i = 0; i < n && raw.size() < n_decoded;
                     ++i, ++it) {
                    raw.push_back(static_cast<double>(*it));
                }
            } catch (...) {
                raw.clear();
                PointCloud2ConstIterator<float> it(msg, "offset_time");
                for (std::size_t i = 0; i < n && raw.size() < n_decoded;
                     ++i, ++it) {
                    raw.push_back(static_cast<double>(*it));
                }
            }
        }
    } catch (...) {
        return false;
    }
    if (raw.size() != n_decoded) {
        return false;
    }
    double t_min = std::numeric_limits<double>::infinity();
    double t_max = -std::numeric_limits<double>::infinity();
    for (double v : raw) {
        if (!std::isfinite(v)) {
            continue;
        }
        t_min = std::min(t_min, v);
        t_max = std::max(t_max, v);
    }
    if (!std::isfinite(t_min) || !std::isfinite(t_max) ||
        t_max <= t_min + 1e-12) {
        // Constant / invalid — treat as uniform.
        *times_rel = frontend::lio::SynthesizeUniformTimeRel(n_decoded);
        return !times_rel->empty();
    }
    // If already in [0,1], keep; if looks like seconds/ms/ns, normalize.
    const bool already_unit =
        t_min >= -1e-3 && t_max <= 1.0 + 1e-3;
    times_rel->resize(raw.size());
    for (std::size_t i = 0; i < raw.size(); ++i) {
        double v = raw[i];
        if (!std::isfinite(v)) {
            (*times_rel)[i] = 0.0;
            continue;
        }
        if (already_unit) {
            (*times_rel)[i] = std::clamp(v, 0.0, 1.0);
        } else {
            (*times_rel)[i] = (v - t_min) / (t_max - t_min);
        }
    }
    return true;
}

bool DecodeRings(
    const automsgs::msgs::sensor_msgs::PointCloud2& msg,
    std::size_t n_decoded,
    std::vector<int>* rings) {
    if (!rings || !HasField(msg, "ring")) {
        return false;
    }
    rings->clear();
    rings->reserve(n_decoded);
    using automsgs::msgs::sensor_msgs::PointCloud2ConstIterator;
    const std::size_t n =
        static_cast<std::size_t>(msg.width()) *
        static_cast<std::size_t>(msg.height());
    try {
        try {
            PointCloud2ConstIterator<std::uint16_t> it(msg, "ring");
            for (std::size_t i = 0; i < n && rings->size() < n_decoded;
                 ++i, ++it) {
                rings->push_back(static_cast<int>(*it));
            }
        } catch (...) {
            rings->clear();
            PointCloud2ConstIterator<std::uint8_t> it(msg, "ring");
            for (std::size_t i = 0; i < n && rings->size() < n_decoded;
                 ++i, ++it) {
                rings->push_back(static_cast<int>(*it));
            }
        }
    } catch (...) {
        return false;
    }
    return rings->size() == n_decoded;
}

}  // namespace

LidarBridge::LidarBridge(system* slam, sensor::LidarSensor* lidar,
                         Options options, frontend::LocalEstimator* estimator)
    : slam_(slam),
      lidar_(lidar),
      estimator_(estimator),
      options_(std::move(options)),
      preprocess_(options_.preprocess),
      keyframe_mgr_(options_.keyframe),
      lidar_loop_(options_.lidar_loop) {}

LidarBridge::~LidarBridge() { Stop(); }

bool LidarBridge::Start(const std::shared_ptr<autolink::Node>& node) {
    if (!node || !lidar_) {
        LOG(ERROR) << "LidarBridge: missing node or LidarSensor";
        return false;
    }
    node_ = node;
    running_ = true;
    auto self = this;
    node_->CreateReader<automsgs::msgs::sensor_msgs::PointCloud2>(
        options_.topic,
        [self](
            const std::shared_ptr<automsgs::msgs::sensor_msgs::PointCloud2>& msg) {
            self->OnCloud(msg);
        });
    LOG(INFO) << "LidarBridge: subscribed " << options_.topic
              << " use_lidar_loop=" << options_.use_lidar_loop;
    return true;
}

void LidarBridge::Stop() { running_ = false; }

Mat44_t LidarBridge::CurrentTwc() const {
    if (estimator_) {
        return estimator_->T_wb();
    }
    if (!slam_) {
        return Mat44_t::Identity();
    }
    const auto pub = slam_->get_map_publisher();
    if (!pub) {
        return Mat44_t::Identity();
    }
    const Mat44_t T_cw = pub->get_current_cam_pose();
    return T_cw.inverse();
}

bool LidarBridge::MaybeKeyframe(double t, const Mat44_t& Twb,
                                const std::vector<Vec3_t>& cloud_body) {
    if (cloud_body.empty() || (!options_.use_lidar_loop && !g2p5_)) {
        return false;
    }
    if (!keyframe_mgr_.DecideAndPush(t, Twb, cloud_body)) {
        return false;
    }
    if (g2p5_) {
        auto kf = std::make_shared<map::G2P5Keyframe>();
        kf->id = keyframe_mgr_.keyframes().back().id;
        kf->T_wb = Twb;
        kf->points_body = cloud_body;
        g2p5_->PushKeyframe(std::move(kf));
    }
    return true;
}

void LidarBridge::MaybeLidarLoop(const Mat44_t& Twb,
                                 const std::vector<Vec3_t>& cloud_body) {
    if (!options_.use_lidar_loop || cloud_body.empty()) {
        return;
    }
    const std::uint64_t kf_id = lidar_loop_.AddKeyframe(Twb, cloud_body);
    if (kf_id == std::numeric_limits<std::uint64_t>::max()) {
        return;
    }
    pose_graph_.AddKeyframe(kf_id, Twb);

    backend::LidarLoopResult lr;
    if (!lidar_loop_.Detect(Twb, cloud_body, &lr) || !lr.found) {
        return;
    }
    LOG(INFO) << "LidarBridge: lidar loop hit query=" << lr.query_id
              << " candidate=" << lr.candidate_id
              << " ndt_score=" << lr.ndt_score
              << " inlier=" << lr.inlier_ratio
              << " mean_res=" << lr.mean_residual;

    // EdgeSE3: T_cand^{-1} * T_query ≈ T_delta (lightning Tij_).
    const double score =
        (lr.ndt_score > 0.0) ? lr.ndt_score
                             : std::max(0.1, lr.inlier_ratio);
    pose_graph_.AddLoop(lr.candidate_id, lr.query_id, lr.T_delta, score);
    if (!pose_graph_.Optimize(/*iters=*/20)) {
        return;
    }

    // Sync optimized SE3 poses back into detector KFs (body clouds unchanged).
    for (const auto& kv : pose_graph_.poses()) {
        lidar_loop_.UpdateKeyframePose(kv.first, kv.second);
    }

    const Mat44_t T_corr =
        pose_graph_.GetPoseOr(lr.query_id, Twb * lr.T_delta);

    // Unified Atlas pose path: LocalEstimator State (LO/LIO) and/or LIVO
    // map_publisher so Tracking sees the correction. No vision loop edges.
    if (estimator_) {
        estimator_->Reset(T_corr);
        LOG(INFO) << "LidarBridge: applied lidar pose-graph Reset to estimator";
    }
    if (slam_) {
        if (const auto pub = slam_->get_map_publisher()) {
            const Mat44_t T_cw =
                estimator_ ? estimator_->T_cw() : T_corr.inverse();
            pub->set_current_cam_pose(T_cw);
            LOG(INFO) << "LidarBridge: set map_publisher pose after lidar loop";
        }
    }
    if (on_loop_closed_) {
        on_loop_closed_(T_corr, lr.query_id, lr.candidate_id);
    }
    if (g2p5_) {
        g2p5_->RedrawGlobalMap();
    }
}

void LidarBridge::OnCloud(
    const std::shared_ptr<automsgs::msgs::sensor_msgs::PointCloud2>& msg) {
    if (!running_ || !msg || !lidar_) {
        return;
    }
    if (!HasField(*msg, "x") || !HasField(*msg, "y") || !HasField(*msg, "z")) {
        return;
    }

    using automsgs::msgs::sensor_msgs::PointCloud2ConstIterator;
    const std::size_t n =
        static_cast<std::size_t>(msg->width()) *
        static_cast<std::size_t>(msg->height());
    if (n == 0) {
        return;
    }

    std::vector<Vec3_t> pts;
    pts.reserve(std::min(n, static_cast<std::size_t>(options_.max_points_decode)));
    PointCloud2ConstIterator<float> iter_x(*msg, "x");
    PointCloud2ConstIterator<float> iter_y(*msg, "y");
    PointCloud2ConstIterator<float> iter_z(*msg, "z");
    for (std::size_t i = 0; i < n; ++i, ++iter_x, ++iter_y, ++iter_z) {
        if (pts.size() >= static_cast<std::size_t>(options_.max_points_decode)) {
            break;
        }
        pts.emplace_back(*iter_x, *iter_y, *iter_z);
    }

    // Decode per-point times for Velodyne/Ouster/Livox when fields exist.
    std::vector<double> decoded_times;
    const bool want_times =
        options_.preprocess.use_point_time ||
        options_.preprocess.model == sensor::LidarModel::kVelodyne ||
        options_.preprocess.model == sensor::LidarModel::kOuster ||
        options_.preprocess.model == sensor::LidarModel::kLivox ||
        HasField(*msg, "t") || HasField(*msg, "time") ||
        HasField(*msg, "offset_time");
    if (want_times) {
        if (!DecodePointTimes(*msg, pts.size(), &decoded_times) &&
            options_.preprocess.synthesize_ring_time &&
            HasField(*msg, "ring")) {
            std::vector<int> rings;
            if (DecodeRings(*msg, pts.size(), &rings)) {
                decoded_times = sensor::Preprocess::SynthesizeRingBasedTime(
                    pts, rings, options_.preprocess.scan_rate_hz);
            }
        }
    }

    std::vector<Vec3_t> filtered;
    std::vector<double> times_rel;
    if (!decoded_times.empty() || options_.preprocess.use_point_time) {
        auto timed = preprocess_.RunTimed(pts, decoded_times);
        filtered.reserve(timed.size());
        times_rel.reserve(timed.size());
        for (const auto& tp : timed) {
            filtered.push_back(tp.p);
            times_rel.push_back(tp.t_rel);
        }
    } else {
        filtered = preprocess_.Run(pts);
    }

    // Approximation: no point times → uniform t_rel so trajectory deskew still runs.
    if (times_rel.empty() && filtered.size() > 1) {
        times_rel = frontend::lio::SynthesizeUniformTimeRel(filtered.size());
    }

    const double t = StampSec(*msg);
    const double t_end = t + options_.default_scan_dt;

    // Optional IMU sync + trajectory deskew; empty sync → pass-through.
    std::vector<Vec3_t> cloud_body = filtered;
    if (imu_sensor_) {
        std::deque<sensor::ImuSample> recent;
        imu_sensor_->CopySince(t - 0.5, &recent);
        for (const auto& s : recent) {
            sync_.PushImu(s);
        }
        sync_.PushLidar(t, t_end, filtered, times_rel);
        frontend::lio::MeasureGroup mg;
        if (sync_.TryPop(&mg)) {
            if (estimator_) {
                imu_process_.SetExtrinsic(estimator_->T_imu_lidar());
            }
            // Ensure times for deskew (sync may have empty if upstream empty).
            if (mg.point_time_rel.empty() && mg.points_body.size() > 1) {
                mg.point_time_rel = frontend::lio::SynthesizeUniformTimeRel(
                    mg.points_body.size());
            }

            // IMUInit: accumulate static IMU; skip deskew / PredictImu until ready.
            bool imu_ready = true;
            if (estimator_ && imu_process_.imu_need_init()) {
                imu_ready = imu_process_.TryImuInit(estimator_, mg);
            }
            if (!imu_ready) {
                cloud_body = mg.points_body;
            } else {
                std::vector<frontend::lio::ImuPoseSample> poses;
                if (estimator_) {
                    frontend::lio::BuildImuPoses(estimator_, mg, &poses);
                }
                if (poses.size() >= 2) {
                    cloud_body = frontend::lio::UndistortByImuTrajectory(
                        mg.points_body, mg.point_time_rel, poses,
                        estimator_ ? estimator_->T_imu_lidar()
                                   : Mat44_t::Identity());
                } else {
                    Vec3_t omega = Vec3_t::Zero();
                    if (!mg.imu.empty()) {
                        omega = mg.imu.back().gyro;
                    }
                    const Mat44_t T_begin =
                        estimator_ ? estimator_->T_wb() : CurrentTwc();
                    cloud_body = imu_process_.UndistortScan(
                        mg.points_body, mg.point_time_rel, T_begin, omega);
                }
            }
        }
    }

    const Mat44_t Twb = estimator_ ? estimator_->T_wb() : CurrentTwc();
    lidar_->FeedWithPose(t, Twb, cloud_body);

    if (map_incremental_) {
        map_incremental_->IntegrateScan(Twb, cloud_body);
    }

    // Skip lidar IEKF until IMUInit finishes (gravity / bg not ready).
    const bool lidar_update_ok =
        !imu_sensor_ || !estimator_ || imu_process_.IsImuInited();
    if (lidar_update_ok && estimator_ && lidar_->residual_source()) {
        auto batch = lidar_->residual_source()->Pull(t - 0.05, t + 0.05);
        if (!batch.empty()) {
            estimator_->UpdateLidar(batch);
        }
    }

    const Mat44_t Twb_now = estimator_ ? estimator_->T_wb() : Twb;
    if (MaybeKeyframe(t, Twb_now, cloud_body) && options_.use_lidar_loop) {
        MaybeLidarLoop(Twb_now, cloud_body);
    }

    if (viz_ && estimator_) {
        viz_->PublishWorldPose(t, estimator_->T_cw());
    }

    if (slam_) {
        if (const auto pub = slam_->get_map_publisher()) {
            const Mat44_t T_cw =
                estimator_ ? estimator_->T_cw() : Twb.inverse();
            pub->set_current_cam_pose(T_cw);
        }
    }
}

}  // namespace autonomy::localization::atlas
