/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy/map/costmap_2d/filters/keepout_filter.hpp"

#include <algorithm>
#include <cstdint>

#include "autonomy/common/logging.hpp"
#include "autonomy/map/costmap_2d/utils/occ_grid_values.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_math.hpp"
#include "autonomy/map/costmap_2d/filters/filter_values.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/tf2/LinearMath/Transform.h"
#include "autonomy/transform/tf2/exceptions.h"

namespace autonomy {
namespace map {
namespace costmap_2d {
namespace {

commsgs::builtin_interfaces::Time LatestTfTime() {
    commsgs::builtin_interfaces::Time stamp;
    stamp.sec = 0;
    stamp.nanosec = 0;
    return stamp;
}

void CommsgsTransformToTf2(const commsgs::geometry_msgs::Transform& in,
                           transform::tf2::Transform& out) {
    out.setOrigin(transform::tf2::Vector3(in.translation.x, in.translation.y,
                                          in.translation.z));
    transform::tf2::Quaternion rotation(in.rotation.x, in.rotation.y,
                                          in.rotation.z, in.rotation.w);
    out.setRotation(rotation);
}

}  // namespace

KeepoutFilter::KeepoutFilter() : filter_mask_(nullptr), global_frame_("") {}

void KeepoutFilter::initializeFilter(const std::string& filter_info_topic) {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

    filter_info_topic_ = filter_info_topic;
    if (layered_costmap_) {
        global_frame_ = layered_costmap_->getGlobalFrameID();
    }
    mask_missing_warned_ = false;

    AINFO << "KeepoutFilter initialized: filter_info_topic="
          << filter_info_topic_ << " global_frame=" << global_frame_
          << " occupancy_threshold=" << static_cast<int>(occupancy_threshold_);
}

commsgs::planning_msgs::CostmapFilterInfo::SharedPtr
KeepoutFilter::makeDefaultFilterInfo(const std::string& mask_topic) {
    auto info = std::make_shared<commsgs::planning_msgs::CostmapFilterInfo>();
    info->type = KEEPOUT_FILTER;
    info->filter_mask_topic = mask_topic;
    info->base = static_cast<float>(BASE_DEFAULT);
    info->multiplier = static_cast<float>(MULTIPLIER_DEFAULT);
    return info;
}

void KeepoutFilter::applyConfiguration(
    const commsgs::planning_msgs::CostmapFilterInfo::SharedPtr& info,
    const commsgs::map_msgs::OccupancyGrid::SharedPtr& mask) {
    if (info) {
        handleFilterInfo(info);
    }
    if (mask) {
        setFilterMask(mask);
    }
}

bool KeepoutFilter::hasFilterMask() {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());
    return static_cast<bool>(filter_mask_);
}

bool KeepoutFilter::validateFilterMask(
    const commsgs::map_msgs::OccupancyGrid& msg) {
    if (msg.info.width == 0 || msg.info.height == 0) {
        AERROR << "KeepoutFilter: mask has zero width/height";
        return false;
    }
    if (msg.info.resolution <= 0.0) {
        AERROR << "KeepoutFilter: mask resolution must be positive";
        return false;
    }
    const size_t expected =
        static_cast<size_t>(msg.info.width) * static_cast<size_t>(msg.info.height);
    if (msg.data.size() < expected) {
        AERROR << "KeepoutFilter: mask data size " << msg.data.size()
               << " < expected " << expected;
        return false;
    }
    return true;
}

std::string KeepoutFilter::resolveMaskFrame() const {
    if (!filter_mask_) {
        return global_frame_;
    }
    const std::string& frame = filter_mask_->header.frame_id;
    if (frame.empty()) {
        return global_frame_;
    }
    return frame;
}

void KeepoutFilter::handleFilterInfo(
    const commsgs::planning_msgs::CostmapFilterInfo::SharedPtr& msg) {
    filterInfoCallback(msg);
}

void KeepoutFilter::setFilterMask(
    const commsgs::map_msgs::OccupancyGrid::SharedPtr& msg) {
    maskCallback(msg);
}

void KeepoutFilter::filterInfoCallback(
    const commsgs::planning_msgs::CostmapFilterInfo::SharedPtr msg) {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

    if (!msg) {
        AWARN << "KeepoutFilter: null filter info message";
        return;
    }

    if (msg->type != KEEPOUT_FILTER) {
        AERROR << "KeepoutFilter: unsupported filter type " << msg->type
               << " (expected KEEPOUT_FILTER=" << static_cast<int>(KEEPOUT_FILTER)
               << ")";
        return;
    }

    if (msg->base != static_cast<float>(BASE_DEFAULT) ||
        msg->multiplier != static_cast<float>(MULTIPLIER_DEFAULT)) {
        AWARN << "KeepoutFilter: base/multiplier should be "
              << BASE_DEFAULT << " and " << MULTIPLIER_DEFAULT
              << " for keepout filters";
    }

    if (!mask_topic_.empty() && mask_topic_ != msg->filter_mask_topic) {
        AWARN << "KeepoutFilter: mask topic changed from " << mask_topic_
              << " to " << msg->filter_mask_topic << ", clearing old mask";
        filter_mask_.reset();
    }

    mask_topic_ = msg->filter_mask_topic;
    setMaskTopic(mask_topic_);

    AINFO << "KeepoutFilter: received filter info, mask_topic=" << mask_topic_;
}

void KeepoutFilter::maskCallback(
    const commsgs::map_msgs::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

    if (!msg) {
        AWARN << "KeepoutFilter: null filter mask message";
        return;
    }

    if (!validateFilterMask(*msg)) {
        return;
    }

    if (!filter_mask_) {
        AINFO << "KeepoutFilter: received filter mask ("
              << msg->info.width << "x" << msg->info.height << ", frame="
              << msg->header.frame_id << ")";
    } else {
        AWARN << "KeepoutFilter: updating filter mask (frame="
              << msg->header.frame_id << ")";
    }

    filter_mask_ = msg;
    mask_missing_warned_ = false;

    if (mask_topic_.empty()) {
        mask_topic_ = "keepout_filter_mask";
        setMaskTopic(mask_topic_);
    }
}

unsigned char KeepoutFilter::keepoutCostFromMaskCell(unsigned int mx,
                                                     unsigned int my) const {
    if (!filter_mask_ || filter_mask_->data.empty()) {
        return NO_INFORMATION;
    }

    const unsigned int index = my * filter_mask_->info.width + mx;
    if (index >= filter_mask_->data.size()) {
        return NO_INFORMATION;
    }

    const int8_t occ = filter_mask_->data[index];
    if (occ == utils::OCC_GRID_UNKNOWN) {
        return NO_INFORMATION;
    }
    if (occ < occupancy_threshold_) {
        return FREE_SPACE;
    }
    return LETHAL_OBSTACLE;
}

bool KeepoutFilter::lookupGlobalToMaskTransform(
    const std::string& mask_frame, transform::tf2::Transform& out) const {
    auto* tf_buffer = autonomy::transform::Buffer::Instance();
    if (!tf_buffer) {
        AWARN << "KeepoutFilter: TF buffer unavailable";
        return false;
    }

    try {
        const float timeout = static_cast<float>(
            SecondsFromDuration(transform_tolerance_));
        const auto stamped_transform = tf_buffer->lookupTransform(
            mask_frame, global_frame_, LatestTfTime(), timeout);
        CommsgsTransformToTf2(stamped_transform.transform, out);
        return true;
    } catch (const transform::tf2::TransformException& ex) {
        AWARN << "KeepoutFilter: TF " << global_frame_ << " -> " << mask_frame
              << " failed: " << ex.what();
        return false;
    }
}

void KeepoutFilter::computeIterationBounds(Costmap2D& master_grid, int min_i,
                                          int min_j, int max_i, int max_j,
                                          const std::string& mask_frame,
                                          int& mg_min_x, int& mg_min_y,
                                          int& mg_max_x, int& mg_max_y) const {
    if (!filter_mask_) {
        mg_min_x = min_i;
        mg_min_y = min_j;
        mg_max_x = max_i;
        mg_max_y = max_j;
        return;
    }

    if (mask_frame != global_frame_) {
        mg_min_x = min_i;
        mg_min_y = min_j;
        mg_max_x = max_i;
        mg_max_y = max_j;
        return;
    }

    const double half_cell_size = 0.5 * filter_mask_->info.resolution;
    double wx = filter_mask_->info.origin.position.x + half_cell_size;
    double wy = filter_mask_->info.origin.position.y + half_cell_size;
    master_grid.worldToMapNoBounds(wx, wy, mg_min_x, mg_min_y);
    if (mg_min_x >= max_i || mg_min_y >= max_j) {
        mg_min_x = min_i;
        mg_min_y = min_j;
        mg_max_x = min_i;
        mg_max_y = min_j;
        return;
    }
    mg_min_x = std::max(min_i, mg_min_x);
    mg_min_y = std::max(min_j, mg_min_y);

    wx = filter_mask_->info.origin.position.x +
         filter_mask_->info.width * filter_mask_->info.resolution +
         half_cell_size;
    wy = filter_mask_->info.origin.position.y +
         filter_mask_->info.height * filter_mask_->info.resolution +
         half_cell_size;
    master_grid.worldToMapNoBounds(wx, wy, mg_max_x, mg_max_y);
    if (mg_max_x <= min_i || mg_max_y <= min_j) {
        mg_min_x = min_i;
        mg_min_y = min_j;
        mg_max_x = min_i;
        mg_max_y = min_j;
        return;
    }
    mg_max_x = std::min(max_i, mg_max_x);
    mg_max_y = std::min(max_j, mg_max_y);
}

void KeepoutFilter::process(Costmap2D& master_grid, int min_i, int min_j,
                            int max_i, int max_j,
                            const commsgs::geometry_msgs::Pose2D& /*pose*/) {
    if (!filter_mask_) {
        if (!mask_missing_warned_) {
            AWARN << "KeepoutFilter: filter mask was not received";
            mask_missing_warned_ = true;
        }
        return;
    }

    if (global_frame_.empty() && layered_costmap_) {
        global_frame_ = layered_costmap_->getGlobalFrameID();
    }

    const std::string mask_frame = resolveMaskFrame();

    transform::tf2::Transform tf2_transform;
    tf2_transform.setIdentity();
    const bool need_tf = !mask_frame.empty() && mask_frame != global_frame_;
    if (need_tf && !lookupGlobalToMaskTransform(mask_frame, tf2_transform)) {
        return;
    }

    int mg_min_x = min_i;
    int mg_min_y = min_j;
    int mg_max_x = max_i;
    int mg_max_y = max_j;
    computeIterationBounds(master_grid, min_i, min_j, max_i, max_j, mask_frame,
                         mg_min_x, mg_min_y, mg_max_x, mg_max_y);

    if (mg_min_x >= max_i || mg_min_y >= max_j || mg_max_x <= min_i ||
        mg_max_y <= min_j) {
        return;
    }

    const unsigned int mg_min_x_u = static_cast<unsigned int>(mg_min_x);
    const unsigned int mg_min_y_u = static_cast<unsigned int>(mg_min_y);
    const unsigned int mg_max_x_u = static_cast<unsigned int>(mg_max_x);
    const unsigned int mg_max_y_u = static_cast<unsigned int>(mg_max_y);

    unsigned char* master_array = master_grid.getCharMap();
    for (unsigned int i = mg_min_x_u; i < mg_max_x_u; ++i) {
        for (unsigned int j = mg_min_y_u; j < mg_max_y_u; ++j) {
            const unsigned int index = master_grid.getIndex(i, j);
            const unsigned char old_data = master_array[index];

            double gl_wx = 0.0;
            double gl_wy = 0.0;
            master_grid.mapToWorld(i, j, gl_wx, gl_wy);

            double msk_wx = gl_wx;
            double msk_wy = gl_wy;
            if (need_tf) {
                transform::tf2::Vector3 point(gl_wx, gl_wy, 0.0);
                point = tf2_transform * point;
                msk_wx = point.x();
                msk_wy = point.y();
            }

            unsigned int mx = 0;
            unsigned int my = 0;
            if (!worldToMask(filter_mask_, msk_wx, msk_wy, mx, my)) {
                continue;
            }

            const unsigned char data = keepoutCostFromMaskCell(mx, my);
            if (data == NO_INFORMATION || data == FREE_SPACE) {
                continue;
            }
            if (data > old_data || old_data == NO_INFORMATION) {
                master_array[index] = data;
            }
        }
    }
}

void KeepoutFilter::resetFilter() {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());
    filter_mask_.reset();
    mask_missing_warned_ = false;
}

bool KeepoutFilter::isActive() {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());
    return static_cast<bool>(filter_mask_);
}

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy
