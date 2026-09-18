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
#include <cstring>
#include <deque>
#include <limits>
#include <string_view>

#include "autolink/common/log.hpp"

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
        AERROR << "LidarBridge: missing node or LidarSensor";
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
    if (options_.publish_global_cloud) {
        global_cloud_writer_ =
            node_->CreateWriter<automsgs::msgs::sensor_msgs::PointCloud2>(
                options_.global_cloud_topic);
    }
    AINFO << "LidarBridge: subscribed " << options_.topic
          << " use_lidar_loop=" << options_.use_lidar_loop
          << " global_cloud="
          << (options_.publish_global_cloud ? options_.global_cloud_topic
                                            : "off");
    return true;
}

void LidarBridge::Stop() {
    running_ = false;
    global_cloud_writer_.reset();
}

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

void LidarBridge::MaybeLidarLoop(double t, const Mat44_t& Twb,
                                 const std::vector<Vec3_t>& cloud_body) {
    if (!options_.use_lidar_loop || cloud_body.empty()) {
        return;
    }
    const std::uint64_t kf_id = lidar_loop_.AddKeyframe(Twb, cloud_body);
    if (kf_id == std::numeric_limits<std::uint64_t>::max()) {
        return;
    }
    pose_graph_.AddKeyframe(kf_id, Twb);

    AINFO_EVERY(5) << "LidarBridge: loop KF#" << kf_id
                   << " total_kfs=" << lidar_loop_.num_keyframes()
                   << " skip_recent=" << options_.lidar_loop.skip_recent_n
                   << " (need >skip_recent before Detect can hit)";

    backend::LidarLoopResult lr;
    if (!lidar_loop_.Detect(Twb, cloud_body, &lr) || !lr.found) {
        PublishConstraintEdges(t);
        return;
    }
    AINFO << "LidarBridge: lidar loop HIT query=" << lr.query_id
          << " candidate=" << lr.candidate_id
          << " ndt_score=" << lr.ndt_score
          << " inlier=" << lr.inlier_ratio
          << " mean_res=" << lr.mean_residual;

    // Require real NDT + ICP residual gate. ICP-only (ndt_score≤0) rejected:
    // early false hits (query≈15→cand=0) SetPose + rebuild → traj stuck short.
    const auto& lopt = options_.lidar_loop;
    const bool ndt_ok = lr.ndt_score > lopt.ndt_score_thresh;
    const bool icp_ok =
        lr.inlier_ratio >= lopt.inlier_ratio_thresh &&
        lr.mean_residual > 1e-6 &&
        lr.mean_residual <= lopt.mean_residual_thresh;
    if (lr.ndt_score <= 0.0) {
        AWARN << "LidarBridge: reject ICP-only loop inlier="
              << lr.inlier_ratio << " mean_res=" << lr.mean_residual;
        PublishConstraintEdges(t);
        return;
    }
    if (!ndt_ok || !icp_ok) {
        AWARN << "LidarBridge: reject NDT loop score=" << lr.ndt_score
              << " inlier=" << lr.inlier_ratio
              << " mean_res=" << lr.mean_residual
              << " (need score>" << lopt.ndt_score_thresh
              << " + ICP residual)";
        PublishConstraintEdges(t);
        return;
    }

    // EdgeSE3: T_cand^{-1} * T_query ≈ T_delta (lightning Tij_).
    const double score =
        (lr.ndt_score > 0.0) ? lr.ndt_score
                             : std::max(0.1, lr.inlier_ratio);
    pose_graph_.AddLoop(lr.candidate_id, lr.query_id, lr.T_delta, score);
    if (!pose_graph_.Optimize(/*iters=*/20)) {
        PublishConstraintEdges(t);
        return;
    }

    // Sync optimized SE3 poses back into detector KFs (body clouds unchanged).
    for (const auto& kv : pose_graph_.poses()) {
        lidar_loop_.UpdateKeyframePose(kv.first, kv.second);
    }

    // Rebuild live IVox from corrected keyframes (lightning post-loop map).
    if (map_incremental_) {
        const int n_pts =
            map_incremental_->RebuildFromKeyframes(lidar_loop_.keyframes());
        AINFO << "LidarBridge: rebuilt IVox after loop kfs="
              << lidar_loop_.num_keyframes() << " pts=" << n_pts;
    }

    const Mat44_t T_corr =
        pose_graph_.GetPoseOr(lr.query_id, Twb * lr.T_delta);

    // Unified Atlas pose path: LocalEstimator State (LO/LIO) and/or LIVO
    // map_publisher so Tracking sees the correction. No vision loop edges.
    // SetPose keeps ba/bg (full Reset would wipe IMUInit).
    if (estimator_) {
        estimator_->SetPose(T_corr, /*zero_velocity=*/true);
        AINFO << "LidarBridge: applied lidar pose-graph SetPose to estimator";
    }
    if (slam_) {
        if (const auto pub = slam_->get_map_publisher()) {
            const Mat44_t T_cw =
                estimator_ ? estimator_->T_cw() : T_corr.inverse();
            pub->set_current_cam_pose(T_cw);
            AINFO << "LidarBridge: set map_publisher pose after lidar loop";
        }
        // Soft Tracking kick via existing relocalize API (not GlobalBA merge).
        const Mat44_t T_wc =
            estimator_ ? estimator_->T_wb() : T_corr;
        if (slam_->relocalize_by_pose(T_wc)) {
            AINFO << "LidarBridge: Tracking relocalize_by_pose after lidar loop";
        }
    }
    if (on_loop_closed_) {
        on_loop_closed_(T_corr, lr.query_id, lr.candidate_id);
    }
    if (g2p5_) {
        g2p5_->RedrawGlobalMap();
    }
    PublishConstraintEdges(t);
}

void LidarBridge::MaybeLidarLoc(double t, const std::vector<Vec3_t>& cloud_body,
                                Mat44_t* Twb_inout) {
    if (!options_.use_lidar_loc || !lidar_locator_ || !Twb_inout ||
        cloud_body.empty() || !lidar_locator_->map_ready()) {
        return;
    }
    Mat44_t guess = *Twb_inout;
    if (pose_extrapolator_ && pose_extrapolator_->initialized()) {
        guess = pose_extrapolator_->PoseAt(t);
    }
    Mat44_t T_aligned = Mat44_t::Identity();
    double score = 0.0;
    if (!lidar_locator_->Align(cloud_body, guess, &T_aligned, &score)) {
        return;
    }
    const Vec3_t p0 = guess.block<3, 1>(0, 3);
    const Vec3_t p1 = T_aligned.block<3, 1>(0, 3);
    if ((p1 - p0).norm() > 0.02) {
        loc_edges_.emplace_back(p0, p1);
        constexpr std::size_t kMaxLocEdges = 200;
        if (loc_edges_.size() > kMaxLocEdges) {
            loc_edges_.erase(loc_edges_.begin(),
                             loc_edges_.begin() +
                                 static_cast<std::ptrdiff_t>(
                                     loc_edges_.size() - kMaxLocEdges));
        }
    }
    *Twb_inout = T_aligned;
    if (pose_extrapolator_) {
        pose_extrapolator_->SetLidarPose(t, T_aligned);
    }
    // Product path: snap pose, keep ba/bg (do not full Reset).
    if (estimator_) {
        estimator_->SetPose(T_aligned, /*zero_velocity=*/true);
    }
    // Incremental dyn layer (Lightning update_dynamic_cloud).
    const int n_dyn =
        lidar_locator_->MaybeUpdateDynamic(t, T_aligned, cloud_body, score);
    if (n_dyn > 0) {
        AINFO_EVERY(20) << "LidarBridge: incremental dyn map +" << n_dyn
                        << " fitness=" << score;
    }
    if (viz_ && estimator_) {
        viz_->PublishWorldPose(t, estimator_->T_wb());
    }
    PublishConstraintEdges(t);
}

void LidarBridge::PublishGlobalCloud(double timestamp_sec) {
    if (!options_.publish_global_cloud || !global_cloud_writer_ ||
        !map_incremental_) {
        return;
    }
    if (last_global_cloud_pub_t_ >= 0.0 &&
        timestamp_sec - last_global_cloud_pub_t_ <
            options_.global_cloud_period_sec) {
        return;
    }
    std::vector<Vec3_t> pts;
    map_incremental_->ivox().ExportWorldPoints(
        &pts, static_cast<std::size_t>(
                  std::max(1, options_.global_cloud_max_points)));
    if (pts.empty()) {
        return;
    }

    automsgs::msgs::sensor_msgs::PointCloud2 cloud;
    const auto sec = static_cast<int32_t>(timestamp_sec);
    auto nanosec = static_cast<uint32_t>(
        std::llround((timestamp_sec - static_cast<double>(sec)) * 1e9));
    if (nanosec >= 1000000000u) {
        cloud.mutable_header()->mutable_stamp()->set_sec(sec + 1);
        cloud.mutable_header()->mutable_stamp()->set_nanosec(nanosec -
                                                             1000000000u);
    } else {
        cloud.mutable_header()->mutable_stamp()->set_sec(sec);
        cloud.mutable_header()->mutable_stamp()->set_nanosec(nanosec);
    }
    cloud.mutable_header()->set_frame_id(options_.global_cloud_frame);
    cloud.set_height(1);
    cloud.set_width(static_cast<uint32_t>(pts.size()));
    cloud.set_is_dense(true);
    cloud.set_is_bigendian(false);
    cloud.set_point_step(16);
    cloud.set_row_step(cloud.point_step() * cloud.width());
    const char* names[] = {"x", "y", "z", "rgb"};
    for (int i = 0; i < 4; ++i) {
        auto* field = cloud.add_fields();
        field->set_name(names[i]);
        field->set_offset(static_cast<uint32_t>(i * 4));
        field->set_datatype(automsgs::msgs::sensor_msgs::PointField::FLOAT32);
        field->set_count(1);
    }

    constexpr uint32_t kWhiteRgb = 0x00FFFFFFu;
    std::vector<float> data;
    data.reserve(pts.size() * 4);
    for (const auto& p : pts) {
        data.push_back(static_cast<float>(p.x()));
        data.push_back(static_cast<float>(p.y()));
        data.push_back(static_cast<float>(p.z()));
        float rgb_as_float = 0.f;
        std::memcpy(&rgb_as_float, &kWhiteRgb, sizeof(float));
        data.push_back(rgb_as_float);
    }
    cloud.set_data(reinterpret_cast<const char*>(data.data()),
                   data.size() * sizeof(float));
    global_cloud_writer_->Write(cloud);
    last_global_cloud_pub_t_ = timestamp_sec;
    AINFO_EVERY(20) << "LidarBridge: published global cloud n=" << pts.size()
                    << " topic=" << options_.global_cloud_topic
                    << " frame=" << options_.global_cloud_frame;
}

void LidarBridge::PublishConstraintEdges(double timestamp_sec) {
    if (!viz_) {
        return;
    }
    // Throttle unless forced (timestamp_sec<=0) or after loop/loc.
    if (timestamp_sec > 0.0 && last_constraint_pub_t_ >= 0.0 &&
        timestamp_sec - last_constraint_pub_t_ < 0.5) {
        return;
    }
    std::vector<std::pair<Vec3_t, Vec3_t>> loops;
    std::vector<std::pair<Vec3_t, Vec3_t>> odom;
    for (const auto& s : pose_graph_.LoopSegments()) {
        loops.emplace_back(s.p0, s.p1);
    }
    for (const auto& s : pose_graph_.OdomSegments()) {
        odom.emplace_back(s.p0, s.p1);
    }
    const double t_pub =
        (timestamp_sec > 0.0) ? timestamp_sec
                              : (last_dbg_t_ > 0.0 ? last_dbg_t_ : 0.0);
    viz_->PublishLidarConstraintEdges(t_pub, loops, odom, loc_edges_);
    last_constraint_pub_t_ = t_pub;
    AINFO_EVERY(40) << "LidarBridge: constraint viz loops=" << loops.size()
                    << " odom_kf=" << odom.size()
                    << " loc=" << loc_edges_.size()
                    << " use_lidar_loop=" << options_.use_lidar_loop;
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

    // Habitat / autosim: no per-point times → instantaneous scan (dt=0).
    // Only synthesize uniform t_rel when default_scan_dt > 0 (spinning lidar).
    const double scan_dt = options_.default_scan_dt;
    if (times_rel.empty() && filtered.size() > 1 && scan_dt > 1e-6) {
        times_rel = frontend::lio::SynthesizeUniformTimeRel(filtered.size());
    }

    const double t_msg = StampSec(*msg);
    const double t_end_msg =
        (scan_dt > 1e-6) ? (t_msg + scan_dt) : t_msg;

    // Lightning-style hard sync gate: with IMU, do not ObsModel / IEKF / insert
    // until LidarImuSync pops a covered measure. Prevents raw-scan ghost maps
    // and timestamp mismatch (FeedWithPose(t_msg) on an older mg cloud).
    std::vector<Vec3_t> cloud_body;
    double t = t_msg;
    [[maybe_unused]] double t_end = t_end_msg;
    bool have_measure = false;

    if (imu_sensor_) {
        std::deque<sensor::ImuSample> recent;
        imu_sensor_->CopySince(t_msg - 0.5, &recent);
        for (const auto& s : recent) {
            sync_.PushImu(s);
        }
        sync_.PushLidar(t_msg, t_end_msg, filtered, times_rel);
        frontend::lio::MeasureGroup mg;
        if (!sync_.TryPop(&mg)) {
            AINFO_EVERY(50)
                << "LidarBridge: waiting IMU sync (queued="
                << sync_.lidar_queued() << " dropped="
                << sync_.dropped_lidar_count() << ")";
            return;
        }
        have_measure = true;
        t = mg.lidar_begin_time;
        t_end = mg.lidar_end_time;
        if (estimator_) {
            imu_process_.SetExtrinsic(estimator_->T_imu_lidar());
        }
        // Only invent uniform times when scan window is non-zero.
        if (mg.point_time_rel.empty() && mg.points_body.size() > 1 &&
            (mg.lidar_end_time - mg.lidar_begin_time) > 1e-6) {
            mg.point_time_rel = frontend::lio::SynthesizeUniformTimeRel(
                mg.points_body.size());
        }

        // IMUInit: accumulate static IMU; skip deskew / PredictImu until ready.
        bool imu_ready = true;
        if (estimator_ && imu_process_.imu_need_init()) {
            imu_ready = imu_process_.TryImuInit(estimator_, mg);
        }
        if (!imu_ready) {
            cloud_body = std::move(mg.points_body);
        } else {
            std::vector<frontend::lio::ImuPoseSample> poses;
            if (estimator_) {
                imu_process_.BuildImuPoses(estimator_, &mg, &poses);
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
    } else {
        // LO without IMU: process current cloud immediately.
        cloud_body = std::move(filtered);
        have_measure = true;
    }

    if (!have_measure || cloud_body.empty()) {
        return;
    }

    const Mat44_t Twb0 = estimator_ ? estimator_->T_wb() : CurrentTwc();
    Mat44_t Twb = Twb0;
    MaybeLidarLoc(t, cloud_body, &Twb);

    // Skip lidar IEKF + map insert until IMUInit finishes (gravity / bg).
    const bool lidar_update_ok =
        !imu_sensor_ || !estimator_ || imu_process_.IsImuInited();
    if (!lidar_update_ok) {
        AINFO_EVERY(20) << "LidarBridge: IMUInit pending — skip IEKF/map";
        ++scan_count_;
        return;
    }

    // Residuals against existing map at predict pose (do not insert yet).
    // Keep ObsModel / MapIncremental extrinsic in sync with estimator.
    if (estimator_ && lidar_) {
        auto obs = lidar_->obs_model_options();
        obs.T_imu_lidar = estimator_->T_imu_lidar();
        lidar_->set_obs_model_options(obs);
    }
    if (estimator_ && map_incremental_) {
        auto mo = map_incremental_->options();
        mo.T_imu_lidar = estimator_->T_imu_lidar();
        map_incremental_->set_options(mo);
    }

    int n_residuals = 0;
    int n_residuals_p2p = 0;
    int n_updated = 0;
    double mean_abs_res = 0.0;
    const Mat44_t Twb_pre_lidar = estimator_ ? estimator_->T_wb() : Twb;

    // Lightning IEKF: ObsModel re-associates NN each iteration.
    // Atlas approximates with a few outer rebuilds of point-plane batch.
    constexpr int kMinSurf = 20;
    constexpr int kOuterIters = 3;
    if (estimator_ && lidar_ && lidar_->ivox()) {
        for (int outer = 0; outer < kOuterIters; ++outer) {
            const Mat44_t Twb_cur = estimator_->T_wb();
            auto batch = lidar_->BuildResiduals(Twb_cur, cloud_body);
            n_residuals = static_cast<int>(batch.point_planes.size());
            n_residuals_p2p = static_cast<int>(batch.point_points.size());
            mean_abs_res = 0.0;
            if (!batch.point_planes.empty()) {
                double sum = 0.0;
                const Mat33_t R0 = Twb_cur.block<3, 3>(0, 0);
                const Vec3_t t0 = Twb_cur.block<3, 1>(0, 3);
                for (const auto& r : batch.point_planes) {
                    const Vec3_t pw = R0 * r.point_body + t0;
                    sum += std::abs(r.normal_world.dot(pw) + r.d);
                }
                mean_abs_res =
                    sum / static_cast<double>(batch.point_planes.size());
            }
            if (n_residuals < kMinSurf) {
                if (outer == 0) {
                    AWARN_EVERY(10)
                        << "LidarBridge: skip IEKF, surf=" << n_residuals
                        << " < " << kMinSurf << " (p2p=" << n_residuals_p2p
                        << ")";
                }
                break;
            }
            if (batch.empty()) {
                break;
            }
            n_updated = estimator_->UpdateLidar(batch);
            if (n_updated == 0) {
                break;
            }
        }
        // Keep residual buffer in sync for any downstream consumers.
        lidar_->FeedWithPose(t, estimator_->T_wb(), cloud_body);
        if (n_updated > 0) {
            const Mat44_t Twb_post = estimator_->T_wb();
            const Vec3_t dp =
                Twb_post.block<3, 1>(0, 3) - Twb_pre_lidar.block<3, 1>(0, 3);
            const Mat33_t dR = Twb_post.block<3, 3>(0, 0) *
                               Twb_pre_lidar.block<3, 3>(0, 0).transpose();
            const double dR_deg =
                Eigen::AngleAxisd(dR).angle() * (180.0 / 3.14159265358979323846);
            if (dp.norm() > 0.5 || dR_deg > 5.0) {
                AWARN_EVERY(5)
                    << "LIO large lidar step after UpdateLidar: |dp|="
                    << dp.norm() << " dR_deg=" << dR_deg
                    << " iekf=" << n_updated << " mean_res=" << mean_abs_res;
            }
            // Pose-only IEKF: set |v| from scan FD (lightning couples via
            // full NavState K).
            if (have_last_dbg_pose_ && last_dbg_t_ > 0.0) {
                const double dt_scan = t - last_dbg_t_;
                if (dt_scan > 0.02 && dt_scan < 1.0) {
                    Vec3_t v_lidar =
                        (Twb_post.block<3, 1>(0, 3) -
                         last_dbg_Twb_.block<3, 1>(0, 3)) /
                        dt_scan;
                    v_lidar.z() = 0.0;
                    constexpr double kMaxV = 1.0;
                    if (v_lidar.norm() > kMaxV) {
                        v_lidar *= kMaxV / v_lidar.norm();
                    }
                    estimator_->set_velocity(v_lidar);
                }
            }
        }
    } else if (estimator_ && lidar_) {
        lidar_->FeedWithPose(t, Twb, cloud_body);
    }

    // Lightning MapIncremental always after Update (predict pose if skipped).
    const Mat44_t Twb_now = estimator_ ? estimator_->T_wb() : Twb;

    int n_map_insert = 0;
    if (map_incremental_) {
        n_map_insert = map_incremental_->IntegrateScan(Twb_now, cloud_body);
    }
    PublishGlobalCloud(t);

    // Seed extrapolator after UpdateLidar so smooth_factor has a lidar anchor.
    if (pose_extrapolator_) {
        pose_extrapolator_->SetLidarPose(t, Twb_now);
    }
    if (MaybeKeyframe(t, Twb_now, cloud_body) && options_.use_lidar_loop) {
        MaybeLidarLoop(t, Twb_now, cloud_body);
    } else {
        PublishConstraintEdges(t);
    }

    // Lidar-rate publish (IMU-rate comes from ImuBridge when wired).
    // body_flu VizBridge: publish T_wb (REP-103), not OpenCV T_cw.
    if (viz_ && estimator_) {
        viz_->PublishWorldPose(t, estimator_->T_wb());
    }

    if (slam_) {
        if (const auto pub = slam_->get_map_publisher()) {
            const Mat44_t T_cw =
                estimator_ ? estimator_->T_cw() : Twb.inverse();
            pub->set_current_cam_pose(T_cw);
        }
    }

    ++scan_count_;
    // Motion / pose health for pure-rotation vs translation debugging.
    {
        const Vec3_t p = Twb_now.block<3, 1>(0, 3);
        const Mat33_t R = Twb_now.block<3, 3>(0, 0);
        const double yaw =
            std::atan2(R(1, 0), R(0, 0)) * (180.0 / 3.14159265358979323846);
        double dp = 0.0;
        double dyaw_deg = 0.0;
        const char* scan_motion = "init";
        if (have_last_dbg_pose_) {
            const Vec3_t p0 = last_dbg_Twb_.block<3, 1>(0, 3);
            const Mat33_t R0 = last_dbg_Twb_.block<3, 3>(0, 0);
            dp = (p - p0).norm();
            const Mat33_t dR = R * R0.transpose();
            const Eigen::AngleAxisd aa(dR);
            dyaw_deg = aa.angle() * (180.0 / 3.14159265358979323846);
            constexpr double kPureRotTrans = 0.05;   // m
            constexpr double kPureRotYaw = 1.0;      // deg
            if (dp < kPureRotTrans && dyaw_deg < kPureRotYaw) {
                scan_motion = "static";
            } else if (dp < kPureRotTrans && dyaw_deg >= kPureRotYaw) {
                scan_motion = "pure_rot";
            } else {
                scan_motion = "translate";
            }
        }
        last_dbg_Twb_ = Twb_now;
        have_last_dbg_pose_ = true;
        last_dbg_t_ = t;

        const Vec3_t vel =
            estimator_ ? estimator_->velocity() : Vec3_t::Zero();
        const std::size_t ivox_pts =
            map_incremental_ ? map_incremental_->ivox().num_points() : 0;
        const std::size_t ivox_vox =
            map_incremental_ ? map_incremental_->ivox().num_voxels() : 0;

        AINFO_EVERY(5)
            << "LIO scan#" << scan_count_ << " t=" << t
            << " motion=" << scan_motion
            << " pose=[" << p.x() << "," << p.y() << "," << p.z() << "]"
            << " yaw_deg=" << yaw << " dp=" << dp << " dR_deg=" << dyaw_deg
            << " |v|=" << vel.norm() << " cloud=" << cloud_body.size()
            << " residuals=" << n_residuals << "+p2p=" << n_residuals_p2p
            << " mean_res=" << mean_abs_res
            << " iekf=" << n_updated
            << " map_ins=" << n_map_insert << " ivox=" << ivox_pts << "/"
            << ivox_vox << " sync=1";

        if (std::string_view(scan_motion) == "pure_rot" && dp > 0.03) {
            AWARN_EVERY(2)
                << "LIO pure_rot position leak: dp=" << dp
                << " dR_deg=" << dyaw_deg
                << " |v|=" << vel.norm() << " pose=[" << p.x() << ","
                << p.y() << "," << p.z() << "]";
        }
    }
}

}  // namespace autonomy::localization::atlas
