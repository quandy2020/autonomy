/*
 * Copyright 2026 Autodriver contributors
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

#include "autodriver/bridge/pose_feeder.hpp"

#include <utility>

#include "autodriver/common/calibration.hpp"
#include "autodriver/driver_params.hpp"
#include "autodriver/sensor_manager.hpp"
#include "autolink/common/log.hpp"

namespace autodriver {
namespace bridge {
namespace {

using Odometry = automsgs::msgs::nav_msgs::Odometry;

}  // namespace

Eigen::Affine3d Affine3dFromPose(
    const automsgs::msgs::geometry_msgs::Pose& pose) {
    Eigen::Affine3d out = Eigen::Affine3d::Identity();
    out.translation() = Eigen::Vector3d(pose.position().x(), pose.position().y(),
                                        pose.position().z());
    Eigen::Quaterniond q(pose.orientation().w(), pose.orientation().x(),
                         pose.orientation().y(), pose.orientation().z());
    if (q.norm() > 1e-12) {
        q.normalize();
        out.linear() = q.toRotationMatrix();
    }
    return out;
}

std::uint64_t StampToNanoseconds(
    const automsgs::msgs::builtin_interfaces::Time& stamp) {
    return static_cast<std::uint64_t>(stamp.sec()) * 1'000'000'000ULL +
           static_cast<std::uint64_t>(stamp.nanosec());
}

std::unordered_map<std::string, std::vector<PoseFeedTarget>> BuildPoseFeedTargets(
    const Config& config) {
    std::unordered_map<std::string, std::vector<PoseFeedTarget>> out;
    for (const Config::Sensor& sensor : config.sensors) {
        if (!hardware::ParseBool(sensor.params, "enable_compensator", false)) {
            continue;
        }
        std::string channel =
            hardware::GetString(sensor.params, "pose_channel", "");
        if (channel.empty()) {
            channel = config.compensator.pose_channel;
        }
        if (channel.empty()) {
            AWARN << "PoseFeeder: " << sensor.id
                  << " enable_compensator without pose_channel; skipped";
            continue;
        }
        PoseFeedTarget target;
        target.id = sensor.id;
        const std::string ext_path =
            hardware::GetString(sensor.params, "extrinsic_path", "");
        if (!ext_path.empty()) {
            common::Extrinsic ext;
            std::string err;
            if (common::LoadExtrinsicYaml(ext_path, &ext, &err)) {
                target.base_T_lidar = ext.transform;
            } else {
                AWARN << "PoseFeeder: extrinsic_path failed for " << sensor.id
                      << " (" << err << "); assuming odom is world←lidar";
            }
        }
        out[channel].push_back(std::move(target));
    }
    return out;
}

PoseFeeder::~PoseFeeder() { Stop(); }

bool PoseFeeder::Start(autolink::Node* node, SensorManager* manager,
                       const Config& config) {
    Stop();
    manager_ = manager;
    targets_ = BuildPoseFeedTargets(config);
    if (targets_.empty()) {
        return true;
    }
    if (manager_ == nullptr) {
        AERROR << "PoseFeeder: manager required when pose targets exist";
        targets_.clear();
        return false;
    }
    if (node == nullptr) {
        AWARN << "PoseFeeder: no Autolink node; use FeedOdometry to inject";
        return true;
    }
    for (const auto& entry : targets_) {
        const std::string channel = entry.first;
        auto reader = node->CreateReader<Odometry>(
            channel, [this, channel](const std::shared_ptr<Odometry>& msg) {
                OnOdometry(channel, msg);
            });
        if (!reader) {
            AERROR << "PoseFeeder: CreateReader failed for " << channel;
            Stop();
            return false;
        }
        readers_.push_back(std::move(reader));
        AINFO << "PoseFeeder subscribed " << channel << " → "
              << entry.second.size() << " lidar(s)";
    }
    return true;
}

void PoseFeeder::Stop() {
    readers_.clear();
    targets_.clear();
    manager_ = nullptr;
}

void PoseFeeder::FeedOdometry(const std::string& channel,
                              const std::shared_ptr<Odometry>& msg) {
    OnOdometry(channel, msg);
}

void PoseFeeder::OnOdometry(const std::string& channel,
                            const std::shared_ptr<Odometry>& msg) {
    if (!msg || manager_ == nullptr) {
        return;
    }
    const auto it = targets_.find(channel);
    if (it == targets_.end()) {
        return;
    }
    const std::uint64_t time_ns =
        StampToNanoseconds(msg->header().stamp());
    const Eigen::Affine3d odom_pose =
        Affine3dFromPose(msg->pose().pose().pose());
    for (const PoseFeedTarget& target : it->second) {
        const Eigen::Affine3d world_T_lidar = odom_pose * target.base_T_lidar;
        if (!manager_->PushLidarPose(target.id, time_ns, world_T_lidar)) {
            // Lidar may not be attached yet (hotplug); silent skip.
        }
    }
}

}  // namespace bridge
}  // namespace autodriver
