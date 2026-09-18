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

#include "autonomy/localization/atlas/sensor/odom/odom_bridge.hpp"

#include "autonomy/localization/atlas/type.hpp"
#include "autonomy/localization/atlas/viz_bridge.hpp"

#include <Eigen/Geometry>

#include "autolink/common/log.hpp"

namespace autonomy::localization::atlas {
namespace {

Mat44_t PoseToMat44(const automsgs::msgs::geometry_msgs::Pose& pose) {
    Mat44_t T = Mat44_t::Identity();
    if (pose.has_position()) {
        T(0, 3) = pose.position().x();
        T(1, 3) = pose.position().y();
        T(2, 3) = pose.position().z();
    }
    if (pose.has_orientation()) {
        Eigen::Quaterniond q(pose.orientation().w(), pose.orientation().x(),
                             pose.orientation().y(), pose.orientation().z());
        if (q.norm() > 1e-9) {
            q.normalize();
            T.block<3, 3>(0, 0) = q.toRotationMatrix();
        }
    }
    return T;
}

double StampSecFromHeader(const automsgs::msgs::std_msgs::Header& header) {
    if (!header.has_stamp()) {
        return 0.0;
    }
    return static_cast<double>(header.stamp().sec()) +
           1e-9 * static_cast<double>(header.stamp().nanosec());
}

}  // namespace

OdomBridge::OdomBridge(sensor::OdomSensor* odom, Options options,
                       frontend::LocalEstimator* estimator, VizBridge* viz)
    : odom_(odom),
      estimator_(estimator),
      viz_(viz),
      options_(std::move(options)) {}

OdomBridge::~OdomBridge() { Stop(); }

bool OdomBridge::Start(const std::shared_ptr<autolink::Node>& node) {
    if (!node || !odom_) {
        AERROR << "OdomBridge: missing node or OdomSensor";
        return false;
    }
    node_ = node;
    running_ = true;
    auto* self = this;
    if (options_.prefer_odometry) {
        node_->CreateReader<automsgs::msgs::nav_msgs::Odometry>(
            options_.topic,
            [self](const std::shared_ptr<automsgs::msgs::nav_msgs::Odometry>& msg) {
                self->OnOdometry(msg);
            });
        AINFO << "OdomBridge: subscribed Odometry " << options_.topic
              << " seed=" << options_.seed_estimator_pose
              << " apply_delta=" << options_.apply_relative_odom;
    } else {
        node_->CreateReader<automsgs::msgs::geometry_msgs::PoseStamped>(
            options_.topic,
            [self](
                const std::shared_ptr<automsgs::msgs::geometry_msgs::PoseStamped>&
                    msg) { self->OnPoseStamped(msg); });
        AINFO << "OdomBridge: subscribed PoseStamped " << options_.topic
              << " seed=" << options_.seed_estimator_pose
              << " apply_delta=" << options_.apply_relative_odom;
    }
    return true;
}

void OdomBridge::Stop() { running_ = false; }

void OdomBridge::ApplySample(const sensor::OdomSample& sample) {
    odom_->Feed(sample);
    if (!estimator_) {
        return;
    }
    if (!estimator_seeded_) {
        return;
    }
    // LIO seed-only: never chain wheel deltas into the estimator.
    if (!options_.apply_relative_odom) {
        return;
    }
    estimator_->UpdateOdom(sample.T_delta);
    if (viz_) {
        // High-rate odom must NOT refresh map→odom TF / trajectory (lidar owns).
        viz_->PublishWorldPose(sample.timestamp, estimator_->T_wb(),
                               /*update_tf=*/false);
    }
}

void OdomBridge::OnOdometry(
    const std::shared_ptr<automsgs::msgs::nav_msgs::Odometry>& msg) {
    if (!running_ || !msg || !odom_) {
        return;
    }
    // PoseWithCovariance.pose is PoseStamped in this automsgs schema.
    if (!msg->has_pose() || !msg->pose().has_pose() ||
        !msg->pose().pose().has_pose()) {
        return;
    }
    const Mat44_t T = PoseToMat44(msg->pose().pose().pose());
    sensor::OdomSample sample;
    if (msg->has_header()) {
        sample.timestamp = StampSecFromHeader(msg->header());
    }
    if (msg->has_twist() && msg->twist().has_twist()) {
        const auto& tw = msg->twist().twist();
        if (tw.has_linear()) {
            sample.v_mps = tw.linear().x();
        }
        if (tw.has_angular()) {
            sample.yaw_rate_rps = tw.angular().z();
        }
    }
    if (has_last_pose_) {
        sample.T_delta = last_T_.inverse() * T;
    } else {
        sample.T_delta = Mat44_t::Identity();
        last_T_ = T;
        has_last_pose_ = true;
        // Seed on first pose before any delta.
        if (estimator_ && options_.seed_estimator_pose && !estimator_seeded_) {
            estimator_->SetPose(T, /*zero_velocity=*/true);
            estimator_seeded_ = true;
            AINFO << "OdomBridge: seeded LocalEstimator from first odom pose";
            if (viz_) {
                viz_->PublishWorldPose(sample.timestamp, estimator_->T_wb(),
                                       /*update_tf=*/false);
            }
            odom_->Feed(sample);
            return;
        }
    }
    last_T_ = T;
    has_last_pose_ = true;
    ApplySample(sample);
}

void OdomBridge::OnPoseStamped(
    const std::shared_ptr<automsgs::msgs::geometry_msgs::PoseStamped>& msg) {
    if (!running_ || !msg || !odom_ || !msg->has_pose()) {
        return;
    }
    const Mat44_t T = PoseToMat44(msg->pose());
    sensor::OdomSample sample;
    if (msg->has_header()) {
        sample.timestamp = StampSecFromHeader(msg->header());
    }
    if (has_last_pose_) {
        sample.T_delta = last_T_.inverse() * T;
    } else {
        sample.T_delta = Mat44_t::Identity();
        last_T_ = T;
        has_last_pose_ = true;
        if (estimator_ && options_.seed_estimator_pose && !estimator_seeded_) {
            estimator_->SetPose(T, /*zero_velocity=*/true);
            estimator_seeded_ = true;
            AINFO << "OdomBridge: seeded LocalEstimator from first PoseStamped";
            if (viz_) {
                viz_->PublishWorldPose(sample.timestamp, estimator_->T_wb(),
                                       /*update_tf=*/false);
            }
            odom_->Feed(sample);
            return;
        }
    }
    last_T_ = T;
    has_last_pose_ = true;
    ApplySample(sample);
}

}  // namespace autonomy::localization::atlas
