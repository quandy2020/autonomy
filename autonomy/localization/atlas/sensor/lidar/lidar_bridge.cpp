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

#include <deque>

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

}  // namespace

LidarBridge::LidarBridge(system* slam, sensor::LidarSensor* lidar,
                         Options options, frontend::LocalEstimator* estimator)
    : slam_(slam),
      lidar_(lidar),
      estimator_(estimator),
      options_(std::move(options)),
      preprocess_(options_.preprocess) {}

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
    LOG(INFO) << "LidarBridge: subscribed " << options_.topic;
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

    std::vector<Vec3_t> filtered;
    std::vector<double> times_rel;
    if (options_.preprocess.use_point_time) {
        // Intensity-as-time stub not decoded here; RunTimed no-ops without times.
        auto timed = preprocess_.RunTimed(pts, /*point_time_rel=*/{});
        filtered.reserve(timed.size());
        times_rel.reserve(timed.size());
        for (const auto& tp : timed) {
            filtered.push_back(tp.p);
            times_rel.push_back(tp.t_rel);
        }
    } else {
        filtered = preprocess_.Run(pts);
    }

    const double t = StampSec(*msg);
    const double t_end = t + options_.default_scan_dt;

    // Optional IMU sync + deskew; empty sync → pass-through filtered cloud.
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
                imu_process_.ProcessPredict(estimator_, mg.imu);
            }
            Vec3_t omega = Vec3_t::Zero();
            if (!mg.imu.empty()) {
                omega = mg.imu.back().gyro;
                if (estimator_) {
                    // Prefer bias-corrected rate when available via last sample.
                    omega = mg.imu.back().gyro;
                }
            }
            const Mat44_t T_begin =
                estimator_ ? estimator_->T_wb() : CurrentTwc();
            cloud_body = imu_process_.UndistortScan(
                mg.points_body, mg.point_time_rel, T_begin, omega);
        }
    }

    const Mat44_t Twb = estimator_ ? estimator_->T_wb() : CurrentTwc();
    lidar_->FeedWithPose(t, Twb, cloud_body);

    if (map_incremental_) {
        map_incremental_->IntegrateScan(Twb, cloud_body);
    }

    if (estimator_ && lidar_->residual_source()) {
        auto batch = lidar_->residual_source()->Pull(t - 0.05, t + 0.05);
        if (!batch.empty()) {
            estimator_->UpdateLidar(batch);
        }
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
