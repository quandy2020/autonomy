/*
 * Copyright 2026 The Openbot Authors (duyongquan)
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

#include "autonomy/task/apps/teleop/point_cloud_obstacle_feeder.hpp"

#include "autonomy/common/logging.hpp"

namespace autonomy::task::teleop {

void PointCloudObstacleFeeder::Configure(
    std::shared_ptr<autolink::Node> node,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap,
    const Options& options) {
    node_ = std::move(node);
    costmap_ = std::move(costmap);
    options_ = options;
    started_ = false;
    reader_.reset();
}

void PointCloudObstacleFeeder::Start() {
    if (started_) {
        return;
    }
    if (!node_ || !costmap_) {
        AERROR << "PointCloudObstacleFeeder: Configure() before Start()";
        return;
    }
    PointCloudObstacleFeeder* self = this;
    reader_ = node_->CreateReader<commsgs::sensor_msgs::PointCloud2>(
        options_.cloud_topic, [self](const std::shared_ptr<
                                     commsgs::sensor_msgs::PointCloud2>& msg) {
            self->OnPointCloud(msg);
        });
    if (!reader_) {
        AERROR << "PointCloudObstacleFeeder: failed to subscribe "
               << options_.cloud_topic;
        return;
    }
    started_ = true;
    AINFO << "PointCloudObstacleFeeder: listening on "
          << options_.cloud_topic;
}

void PointCloudObstacleFeeder::Stop() {
    if (reader_) {
        reader_->Shutdown();
        reader_.reset();
    }
    started_ = false;
}

bool PointCloudObstacleFeeder::IsCloudFresh() const {
    if (last_cloud_time_.time_since_epoch().count() == 0) {
        return false;
    }
    const auto elapsed =
        std::chrono::steady_clock::now() - last_cloud_time_;
    return std::chrono::duration<double>(elapsed).count() <=
           options_.stale_timeout_sec;
}

void PointCloudObstacleFeeder::OnPointCloud(
    const std::shared_ptr<commsgs::sensor_msgs::PointCloud2>& msg) {
    if (!msg || !costmap_) {
        return;
    }
    if (msg->width == 0 && msg->height == 0) {
        return;
    }
    costmap_->feedPointCloud2(*msg);
    last_cloud_time_ = std::chrono::steady_clock::now();
}

}  // namespace autonomy::task::teleop
