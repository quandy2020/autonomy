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

#include "autonomy/map/costmap_2d/filters/speed_filter.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/map/costmap_2d/filters/filter_values.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {

SpeedFilter::SpeedFilter()
    : filter_mask_(nullptr),
      global_frame_(""),
      base_(BASE_DEFAULT),
      multiplier_(MULTIPLIER_DEFAULT),
      percentage_(false),
      speed_limit_(NO_SPEED_LIMIT),
      speed_limit_prev_(NO_SPEED_LIMIT) {}

void SpeedFilter::initializeFilter(const std::string& filter_info_topic) {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

    filter_info_topic_ = filter_info_topic;
    if (layered_costmap_) {
        global_frame_ = layered_costmap_->getGlobalFrameID();
    }

    base_ = BASE_DEFAULT;
    multiplier_ = MULTIPLIER_DEFAULT;
    percentage_ = false;
    filter_info_received_ = false;
    mask_missing_warned_ = false;
    unknown_cell_warned_ = false;
    speed_limit_ = NO_SPEED_LIMIT;
    speed_limit_prev_ = NO_SPEED_LIMIT;

    AINFO << "SpeedFilter initialized: filter_info_topic=" << filter_info_topic_
          << " global_frame=" << global_frame_;
}

commsgs::map_msgs::CostmapFilterInfo::SharedPtr
SpeedFilter::makeDefaultFilterInfo(const std::string& mask_topic, bool percentage,
                                   float base, float multiplier) {
    auto info = std::make_shared<commsgs::map_msgs::CostmapFilterInfo>();
    info->type = percentage ? SPEED_FILTER_PERCENT : SPEED_FILTER_ABSOLUTE;
    info->filter_mask_topic = mask_topic;
    info->base = base;
    info->multiplier = multiplier;
    return info;
}

void SpeedFilter::applyConfiguration(
    const commsgs::map_msgs::CostmapFilterInfo::SharedPtr& info,
    const commsgs::map_msgs::OccupancyGrid::SharedPtr& mask) {
    if (info) {
        handleFilterInfo(info);
    }
    if (mask) {
        setFilterMask(mask);
    }
}

void SpeedFilter::handleFilterInfo(
    const commsgs::map_msgs::CostmapFilterInfo::SharedPtr& msg) {
    filterInfoCallback(msg);
}

void SpeedFilter::setFilterMask(
    const commsgs::map_msgs::OccupancyGrid::SharedPtr& msg) {
    maskCallback(msg);
}

void SpeedFilter::setSpeedLimitCallback(SpeedLimitCallback callback) {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());
    speed_limit_callback_ = std::move(callback);
}

double SpeedFilter::getSpeedLimit() {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());
    return speed_limit_;
}

bool SpeedFilter::isPercentageMode() {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());
    return percentage_;
}

bool SpeedFilter::isFilterConfigured() {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());
    return filter_info_received_;
}

bool SpeedFilter::hasFilterMask() {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());
    return static_cast<bool>(filter_mask_);
}

bool SpeedFilter::validateFilterMask(
    const commsgs::map_msgs::OccupancyGrid& msg) {
    if (msg.info.width == 0 || msg.info.height == 0) {
        AERROR << "SpeedFilter: mask has zero width/height";
        return false;
    }
    if (msg.info.resolution <= 0.0) {
        AERROR << "SpeedFilter: mask resolution must be positive";
        return false;
    }
    const size_t expected =
        static_cast<size_t>(msg.info.width) * static_cast<size_t>(msg.info.height);
    if (msg.data.size() < expected) {
        AERROR << "SpeedFilter: mask data size " << msg.data.size()
               << " < expected " << expected;
        return false;
    }
    return true;
}

void SpeedFilter::filterInfoCallback(
    const commsgs::map_msgs::CostmapFilterInfo::SharedPtr msg) {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

    if (!msg) {
        AWARN << "SpeedFilter: null filter info message";
        return;
    }

    if (msg->type != SPEED_FILTER_PERCENT &&
        msg->type != SPEED_FILTER_ABSOLUTE) {
        AERROR << "SpeedFilter: unsupported filter type " << msg->type
               << " (expected SPEED_FILTER_PERCENT="
               << static_cast<int>(SPEED_FILTER_PERCENT)
               << " or SPEED_FILTER_ABSOLUTE="
               << static_cast<int>(SPEED_FILTER_ABSOLUTE) << ")";
        return;
    }

    if (!filter_info_received_) {
        AINFO << "SpeedFilter: received filter info from " << filter_info_topic_;
    } else {
        AWARN << "SpeedFilter: updating filter info from " << filter_info_topic_;
    }

    base_ = static_cast<double>(msg->base);
    multiplier_ = static_cast<double>(msg->multiplier);
    percentage_ = (msg->type == SPEED_FILTER_PERCENT);

    if (percentage_) {
        AINFO << "SpeedFilter: percent mode speed_limit = " << base_ << " + mask * "
              << multiplier_;
    } else {
        AINFO << "SpeedFilter: absolute mode speed_limit = " << base_ << " + mask * "
              << multiplier_ << " (m/s)";
    }

    if (!mask_topic_.empty() && mask_topic_ != msg->filter_mask_topic) {
        AWARN << "SpeedFilter: mask topic changed from " << mask_topic_ << " to "
              << msg->filter_mask_topic << ", clearing old mask";
        filter_mask_.reset();
    }

    mask_topic_ = msg->filter_mask_topic;
    setMaskTopic(mask_topic_);
    filter_info_received_ = true;
}

void SpeedFilter::maskCallback(
    const commsgs::map_msgs::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

    if (!msg) {
        AWARN << "SpeedFilter: null filter mask message";
        return;
    }

    if (!validateFilterMask(*msg)) {
        return;
    }

    if (!filter_mask_) {
        AINFO << "SpeedFilter: received filter mask (" << msg->info.width << "x"
              << msg->info.height << ", frame=" << msg->header.frame_id << ")";
    } else {
        AWARN << "SpeedFilter: updating filter mask (frame=" << msg->header.frame_id
              << ")";
    }

    filter_mask_ = msg;
    mask_missing_warned_ = false;

    if (mask_topic_.empty()) {
        mask_topic_ = "speed_filter_mask";
        setMaskTopic(mask_topic_);
    }
}

std::string SpeedFilter::resolveMaskFrame() const {
    if (!filter_mask_) {
        return global_frame_;
    }
    const std::string& frame = filter_mask_->header.frame_id;
    if (frame.empty()) {
        return global_frame_;
    }
    return frame;
}

double SpeedFilter::computeSpeedLimitFromMaskCell(int8_t speed_mask_data) const {
    if (speed_mask_data == SPEED_MASK_NO_LIMIT) {
        return NO_SPEED_LIMIT;
    }
    if (speed_mask_data == SPEED_MASK_UNKNOWN) {
        return speed_limit_prev_;
    }

    double limit = static_cast<double>(speed_mask_data) * multiplier_ + base_;
    if (percentage_) {
        if (limit < 0.0 || limit > 100.0) {
            AWARN << "SpeedFilter: percent speed limit " << limit
                  << " out of [0, 100], using no-limit";
            return NO_SPEED_LIMIT;
        }
    } else if (limit < 0.0) {
        AWARN << "SpeedFilter: absolute speed limit " << limit
              << " < 0, using no-limit";
        return NO_SPEED_LIMIT;
    }
    return limit;
}

commsgs::nav_msgs::SpeedLimit::SharedPtr SpeedFilter::buildSpeedLimitMessage()
    const {
    auto msg = std::make_shared<commsgs::nav_msgs::SpeedLimit>();
    msg->header.frame_id = global_frame_;
    msg->header.stamp.sec = 0;
    msg->header.stamp.nanosec = 0;
    msg->percentage = percentage_;
    msg->speed_limit = static_cast<float>(speed_limit_);
    return msg;
}

void SpeedFilter::notifySpeedLimitIfChanged() {
    if (speed_limit_ == speed_limit_prev_) {
        return;
    }

    if (speed_limit_ != NO_SPEED_LIMIT) {
        if (percentage_) {
            AINFO << "SpeedFilter: speed limit set to " << speed_limit_ << "%";
        } else {
            AINFO << "SpeedFilter: speed limit set to " << speed_limit_ << " m/s";
        }
    } else {
        AINFO << "SpeedFilter: speed limit cleared (no limit)";
    }

    if (speed_limit_callback_) {
        speed_limit_callback_(buildSpeedLimitMessage());
    }

    speed_limit_prev_ = speed_limit_;
}

void SpeedFilter::process(Costmap2D& /*master_grid*/, int /*min_i*/,
                          int /*min_j*/, int /*max_i*/, int /*max_j*/,
                          const commsgs::geometry_msgs::Pose2D& pose) {
    if (!filter_mask_) {
        if (!mask_missing_warned_) {
            AWARN << "SpeedFilter: filter mask was not received";
            mask_missing_warned_ = true;
        }
        return;
    }

    if (!filter_info_received_) {
        if (!mask_missing_warned_) {
            AWARN << "SpeedFilter: filter info was not received";
            mask_missing_warned_ = true;
        }
        return;
    }

    if (global_frame_.empty() && layered_costmap_) {
        global_frame_ = layered_costmap_->getGlobalFrameID();
    }

    commsgs::geometry_msgs::Pose2D mask_pose;
    const std::string mask_frame = resolveMaskFrame();
    if (!transformPose(global_frame_, pose, mask_frame, mask_pose)) {
        return;
    }

    unsigned int mask_robot_i = 0;
    unsigned int mask_robot_j = 0;
    if (!worldToMask(filter_mask_, mask_pose.x, mask_pose.y, mask_robot_i,
                     mask_robot_j)) {
        return;
    }

    const int8_t speed_mask_data =
        getMaskData(filter_mask_, mask_robot_i, mask_robot_j);
    if (speed_mask_data == SPEED_MASK_UNKNOWN) {
        if (!unknown_cell_warned_) {
            AERROR << "SpeedFilter: unknown cell at mask[" << mask_robot_i << ","
                   << mask_robot_j << "]";
            unknown_cell_warned_ = true;
        }
        return;
    }

    speed_limit_ = computeSpeedLimitFromMaskCell(speed_mask_data);
    notifySpeedLimitIfChanged();
}

void SpeedFilter::resetFilter() {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

    filter_mask_.reset();
    filter_info_received_ = false;
    mask_missing_warned_ = false;
    unknown_cell_warned_ = false;
    speed_limit_ = NO_SPEED_LIMIT;
    speed_limit_prev_ = NO_SPEED_LIMIT;
    base_ = BASE_DEFAULT;
    multiplier_ = MULTIPLIER_DEFAULT;
    percentage_ = false;
}

bool SpeedFilter::isActive() {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());
    return static_cast<bool>(filter_mask_) && filter_info_received_;
}

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy
