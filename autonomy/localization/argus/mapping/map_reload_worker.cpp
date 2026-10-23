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

#include "argus/map/map_reload_worker.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_set>
#include <utility>

namespace argus {
namespace map {
namespace {

int64_t VoxelKey(double x, double y, double z, double res) {
    const double inv = 1.0 / std::max(res, 1e-3);
    const int64_t ix = static_cast<int64_t>(std::floor(x * inv));
    const int64_t iy = static_cast<int64_t>(std::floor(y * inv));
    const int64_t iz = static_cast<int64_t>(std::floor(z * inv));
    // 21-bit packing; adequate for local LTA chunks.
    return (ix & 0x1FFFFF) | ((iy & 0x1FFFFF) << 21) | ((iz & 0x1FFFFF) << 42);
}

}  // namespace

MapReloadWorker::MapReloadWorker(MapReloadConfig config)
    : config_(std::move(config)) {}

void MapReloadWorker::set_config(MapReloadConfig config) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = std::move(config);
}

void MapReloadWorker::SubmitHint(const MapReloadHint& hint) {
    std::lock_guard<std::mutex> lock(mutex_);
    hints_.push_back(hint);
}

sensor::PointCloudXYZI MapReloadWorker::Downsample(
    const sensor::PointCloudXYZI& in) const {
    if (in.empty() || config_.downsample_resolution_m <= 0.0) {
        return in;
    }
    std::unordered_set<int64_t> seen;
    seen.reserve(in.size());
    sensor::PointCloudXYZI out;
    out.reserve(in.size() / 2 + 1);
    const double res = config_.downsample_resolution_m;
    for (const auto& p : in.points) {
        const int64_t key = VoxelKey(p.x, p.y, p.z, res);
        if (!seen.insert(key).second) {
            continue;
        }
        out.push_back(p);
    }
    return out;
}

void MapReloadWorker::EnqueueCorrectedCloud(sensor::PointCloudXYZI cloud_w) {
    if (!config_.enable || cloud_w.empty()) {
        return;
    }
    QueuedCloud item;
    item.cloud = Downsample(cloud_w);
    if (item.cloud.empty()) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push_back(std::move(item));
    while (queue_.size() > config_.max_queued_clouds) {
        queue_.pop_front();
    }
}

void MapReloadWorker::EnqueueTransformed(const sensor::PointCloudXYZI& cloud,
                                         const transform::Rigid3d& delta_w) {
    if (!config_.enable || cloud.empty()) {
        return;
    }
    sensor::PointCloudXYZI world;
    world.reserve(cloud.size());
    for (const auto& p : cloud.points) {
        const Eigen::Vector3d pw =
            delta_w * Eigen::Vector3d(p.x, p.y, p.z);
        sensor::PointType q = p;
        q.x = static_cast<float>(pw.x());
        q.y = static_cast<float>(pw.y());
        q.z = static_cast<float>(pw.z());
        world.push_back(q);
    }
    EnqueueCorrectedCloud(std::move(world));
}

std::size_t MapReloadWorker::Drain(sensor::PointCloudXYZI* out,
                                   std::size_t budget) {
    if (out == nullptr || budget == 0) {
        return 0;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    std::size_t taken = 0;
    out->reserve(out->size() + budget);
    while (!queue_.empty() && taken < budget) {
        QueuedCloud& front = queue_.front();
        const std::size_t available = front.cloud.size() - front.offset;
        if (available == 0) {
            queue_.pop_front();
            continue;
        }
        const std::size_t room = budget - taken;
        const std::size_t n = std::min(available, room);
        for (std::size_t i = 0; i < n; ++i) {
            out->push_back(front.cloud.points[front.offset + i]);
        }
        front.offset += n;
        taken += n;
        if (front.offset >= front.cloud.size()) {
            queue_.pop_front();
        }
    }
    return taken;
}

std::size_t MapReloadWorker::queue_size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
}

std::vector<MapReloadHint> MapReloadWorker::TakeHints() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<MapReloadHint> out;
    out.swap(hints_);
    return out;
}

}  // namespace map
}  // namespace argus
