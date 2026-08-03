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

#include "autonomy/map/costmap_2d/filters/binary_filter.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/map/costmap_2d/filters/filter_values.hpp"
#include "autonomy/map/costmap_2d/utils/occ_grid_values.hpp"
#include <automsgs/msgs/geometry_msgs/pose2d.pb.h>

namespace autonomy {
namespace map {
namespace costmap_2d {

BinaryFilter::BinaryFilter()
    : filter_mask_(nullptr),
      global_frame_(""),
      mask_frame_(""),
      base_(BASE_DEFAULT),
      multiplier_(MULTIPLIER_DEFAULT),
      flip_threshold_(50.0),
      default_state_(false),
      binary_state_(false) {}

void BinaryFilter::initializeFilter(const std::string& filter_info_topic) {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

    filter_info_topic_ = filter_info_topic;
    if (layered_costmap_) {
        global_frame_ = layered_costmap_->getGlobalFrameID();
    }

    base_ = BASE_DEFAULT;
    multiplier_ = MULTIPLIER_DEFAULT;
    filter_info_received_ = false;
    mask_missing_warned_ = false;
    unknown_cell_warned_ = false;
    outside_mask_warned_ = false;

    AINFO << "BinaryFilter initialized: filter_info_topic=" << filter_info_topic_
          << " global_frame=" << global_frame_
          << " default_state=" << default_state_
          << " flip_threshold=" << flip_threshold_;

    changeState(default_state_);
}

std::shared_ptr<automsgs::msgs::map_msgs::CostmapFilterInfo>
BinaryFilter::makeDefaultFilterInfo(const std::string& mask_topic, float base,
                                    float multiplier) {
    auto info = std::make_shared<automsgs::msgs::map_msgs::CostmapFilterInfo>();
    info->set_type(BINARY_FILTER);
    info->set_filter_mask_topic(mask_topic);
    info->set_base(base);
    info->set_multiplier(multiplier);
    return info;
}

void BinaryFilter::applyConfiguration(
    const std::shared_ptr<automsgs::msgs::map_msgs::CostmapFilterInfo>& info,
    const std::shared_ptr<automsgs::msgs::map_msgs::OccupancyGrid>& mask) {
    if (info) {
        handleFilterInfo(info);
    }
    if (mask) {
        setFilterMask(mask);
    }
}

void BinaryFilter::handleFilterInfo(
    const std::shared_ptr<automsgs::msgs::map_msgs::CostmapFilterInfo>& msg) {
    filterInfoCallback(msg);
}

void BinaryFilter::setFilterMask(
    const std::shared_ptr<automsgs::msgs::map_msgs::OccupancyGrid>& msg) {
    maskCallback(msg);
}

void BinaryFilter::setBinaryStateCallback(BinaryStateCallback callback) {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());
    binary_state_callback_ = std::move(callback);
}

bool BinaryFilter::getBinaryState() {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());
    return binary_state_;
}

void BinaryFilter::setDefaultState(bool default_state) {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());
    default_state_ = default_state;
}

bool BinaryFilter::getDefaultState() {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());
    return default_state_;
}

void BinaryFilter::setFlipThreshold(double threshold) {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());
    flip_threshold_ = threshold;
}

double BinaryFilter::getFlipThreshold() {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());
    return flip_threshold_;
}

bool BinaryFilter::isFilterConfigured() {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());
    return filter_info_received_;
}

bool BinaryFilter::hasFilterMask() {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());
    return static_cast<bool>(filter_mask_);
}

bool BinaryFilter::validateFilterMask(
    const automsgs::msgs::map_msgs::OccupancyGrid& msg) {
    if (msg.info().width() == 0 || msg.info().height() == 0) {
        AERROR << "BinaryFilter: mask has zero width/height";
        return false;
    }
    if (msg.info().resolution() <= 0.0) {
        AERROR << "BinaryFilter: mask resolution must be positive";
        return false;
    }
    const size_t expected =
        static_cast<size_t>(msg.info().width()) * static_cast<size_t>(msg.info().height());
    if (msg.data().size() < expected) {
        AERROR << "BinaryFilter: mask data size " << msg.data().size()
               << " < expected " << expected;
        return false;
    }
    return true;
}

void BinaryFilter::filterInfoCallback(
    const std::shared_ptr<automsgs::msgs::map_msgs::CostmapFilterInfo> msg) {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

    if (!msg) {
        AWARN << "BinaryFilter: null filter info message";
        return;
    }

    if (msg->type() != BINARY_FILTER) {
        AERROR << "BinaryFilter: unsupported filter type " << msg->type()
               << " (expected BINARY_FILTER="
               << static_cast<int>(BINARY_FILTER) << ")";
        return;
    }

    if (!filter_info_received_) {
        AINFO << "BinaryFilter: received filter info from " << filter_info_topic_;
    } else {
        AWARN << "BinaryFilter: updating filter info from " << filter_info_topic_;
    }

    base_ = static_cast<double>(msg->base());
    multiplier_ = static_cast<double>(msg->multiplier());

    if (!mask_topic_.empty() && mask_topic_ != msg->filter_mask_topic()) {
        AWARN << "BinaryFilter: mask topic changed from " << mask_topic_ << " to "
              << msg->filter_mask_topic() << ", clearing old mask";
        filter_mask_.reset();
        mask_frame_.clear();
    }

    mask_topic_ = msg->filter_mask_topic();
    setMaskTopic(mask_topic_);
    filter_info_received_ = true;

    AINFO << "BinaryFilter: binary mode value = " << base_ << " + mask * "
          << multiplier_ << ", flip_threshold=" << flip_threshold_;
}

void BinaryFilter::maskCallback(
    const std::shared_ptr<automsgs::msgs::map_msgs::OccupancyGrid> msg) {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

    if (!msg) {
        AWARN << "BinaryFilter: null filter mask message";
        return;
    }

    if (!validateFilterMask(*msg)) {
        return;
    }

    if (!filter_mask_) {
        AINFO << "BinaryFilter: received filter mask (" << msg->info().width()<< "x"
              << msg->info().height()<< ", frame=" << msg->header().frame_id()<< ")";
    } else {
        AWARN << "BinaryFilter: updating filter mask (frame=" << msg->header().frame_id()<< ")";
    }

    filter_mask_ = msg;
    mask_frame_ = msg->header().frame_id();
    mask_missing_warned_ = false;
    outside_mask_warned_ = false;
    unknown_cell_warned_ = false;

    if (mask_topic_.empty()) {
        mask_topic_ = "binary_filter_mask";
        setMaskTopic(mask_topic_);
    }
}

std::string BinaryFilter::resolveMaskFrame() const {
    if (!mask_frame_.empty()) {
        return mask_frame_;
    }
    if (filter_mask_ && !filter_mask_->header().frame_id().empty()) {
        return filter_mask_->header().frame_id();
    }
    return global_frame_;
}

bool BinaryFilter::evaluateMaskCell(int8_t mask_data) const {
    const double value =
        base_ + static_cast<double>(mask_data) * multiplier_;
    return value > flip_threshold_;
}

void BinaryFilter::changeState(const bool state) {
    if (binary_state_ == state) {
        return;
    }

    binary_state_ = state;
    if (state) {
        AINFO << "BinaryFilter: switched on (non-default state)";
    } else {
        AINFO << "BinaryFilter: switched off (default state)";
    }

    if (binary_state_callback_) {
        binary_state_callback_(state);
    }
}

void BinaryFilter::process(Costmap2D& /*master_grid*/, int /*min_i*/,
                           int /*min_j*/, int /*max_i*/, int /*max_j*/,
                           const automsgs::msgs::geometry_msgs::Pose2D& pose) {
    if (!filter_mask_) {
        if (!mask_missing_warned_) {
            AWARN << "BinaryFilter: filter mask was not received";
            mask_missing_warned_ = true;
        }
        return;
    }

    if (!filter_info_received_) {
        if (!mask_missing_warned_) {
            AWARN << "BinaryFilter: filter info was not received";
            mask_missing_warned_ = true;
        }
        return;
    }

    if (global_frame_.empty() && layered_costmap_) {
        global_frame_ = layered_costmap_->getGlobalFrameID();
    }

    automsgs::msgs::geometry_msgs::Pose2D mask_pose;
    const std::string mask_frame = resolveMaskFrame();
    if (!transformPose(global_frame_, pose, mask_frame, mask_pose)) {
        return;
    }

    unsigned int mask_robot_i = 0;
    unsigned int mask_robot_j = 0;
    if (!worldToMask(filter_mask_, mask_pose.x(), mask_pose.y(), mask_robot_i,
                     mask_robot_j)) {
        if (!outside_mask_warned_) {
            AWARN << "BinaryFilter: robot is outside filter mask, resetting to "
                     "default state";
            outside_mask_warned_ = true;
        }
        if (binary_state_ != default_state_) {
            changeState(default_state_);
        }
        return;
    }

    outside_mask_warned_ = false;

    const int8_t mask_data =
        getMaskData(filter_mask_, mask_robot_i, mask_robot_j);
    if (mask_data == utils::OCC_GRID_UNKNOWN) {
        if (!unknown_cell_warned_) {
            AWARN << "BinaryFilter: unknown cell at mask[" << mask_robot_i << ","
                  << mask_robot_j << "]";
            unknown_cell_warned_ = true;
        }
        return;
    }

    unknown_cell_warned_ = false;

    if (evaluateMaskCell(mask_data)) {
        if (binary_state_ == default_state_) {
            changeState(!default_state_);
        }
    } else if (binary_state_ != default_state_) {
        changeState(default_state_);
    }
}

void BinaryFilter::resetFilter() {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

    AINFO << "BinaryFilter: resetting filter";
    filter_mask_.reset();
    mask_frame_.clear();
    filter_info_received_ = false;
    mask_missing_warned_ = false;
    unknown_cell_warned_ = false;
    outside_mask_warned_ = false;
    base_ = BASE_DEFAULT;
    multiplier_ = MULTIPLIER_DEFAULT;
    changeState(default_state_);
}

bool BinaryFilter::isActive() {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());
    return static_cast<bool>(filter_mask_) && filter_info_received_;
}

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy
