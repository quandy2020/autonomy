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

#include "autonomy/map/costmap_2d/filters/costmap_filter.hpp"

#include <cmath>

#include "autonomy/common/logging.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_math.hpp"
#include "autonomy/map/costmap_2d/utils/occ_grid_values.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/tf2/exceptions.h"
#include <automsgs/msgs/geometry_msgs/pose2d.pb.h>

namespace autonomy {
namespace map {
namespace costmap_2d {
namespace {

void TransformPoint(const automsgs::msgs::geometry_msgs::Transform& transform,
                    double x, double y, double z, double& out_x, double& out_y,
                    double& out_z) {
    const auto& q = transform.rotation();
    const auto& t = transform.translation();
    const double qx = q.x();
    const double qy = q.y();
    const double qz = q.z();
    const double qw = q.w();
    const double ix = qw * x + qy * z - qz * y;
    const double iy = qw * y + qz * x - qx * z;
    const double iz = qw * z + qx * y - qy * x;
    const double iw = -qx * x - qy * y - qz * z;
    out_x = ix * qw + iw * -qx + iy * -qz - iz * -qy + t.x();
    out_y = iy * qw + iw * -qy + iz * -qx - ix * -qz + t.y();
    out_z = iz * qw + iw * -qz + ix * -qy - iy * -qx + t.z();
}

automsgs::msgs::builtin_interfaces::Time LatestTfTime() {
    automsgs::msgs::builtin_interfaces::Time stamp;
    stamp.set_sec(0);
    stamp.set_nanosec(0);
    return stamp;
}

}  // namespace

CostmapFilter::CostmapFilter()
    : filter_info_topic_(""),
      mask_topic_(""),
      transform_tolerance_(DurationFromSeconds(0.1)) {
    access_ = new mutex_t();
    latest_pose_.set_x(0.0);
    latest_pose_.set_y(0.0);
    latest_pose_.set_theta(0.0);
}

CostmapFilter::~CostmapFilter() {
    delete access_;
}

void CostmapFilter::onInitialize() {
    enabled_ = true;
    current_ = true;
    transform_tolerance_ = DurationFromSeconds(0.1);

    const auto* layer_options = getOptions();
    if (layer_options && layer_options->has_static_layer()) {
        const double tol = layer_options->static_layer().transform_tolerance();
        if (tol > 0.0) {
            transform_tolerance_ = DurationFromSeconds(tol);
        }
    }

    if (filter_info_topic_.empty() && !name_.empty()) {
        filter_info_topic_ = name_ + "/filter_info";
    }

    AINFO << "CostmapFilter '" << name_
          << "' initialized: enabled=" << enabled_
          << " filter_info_topic=" << filter_info_topic_
          << " transform_tolerance="
          << SecondsFromDuration(transform_tolerance_) << "s";
}

void CostmapFilter::activate() {
    if (!filter_info_topic_.empty()) {
        initializeFilter(filter_info_topic_);
    } else {
        initializeFilter(name_ + "/filter_info");
    }
}

void CostmapFilter::deactivate() {
    resetFilter();
}

void CostmapFilter::reset() {
    resetFilter();
    if (!filter_info_topic_.empty()) {
        initializeFilter(filter_info_topic_);
    }
    current_ = false;
}

void CostmapFilter::updateBounds(double robot_x, double robot_y,
                                 double robot_yaw, double* /*min_x*/,
                                 double* /*min_y*/, double* /*max_x*/,
                                 double* /*max_y*/) {
    if (!enabled_) {
        return;
    }

    latest_pose_.set_x(robot_x);
    latest_pose_.set_y(robot_y);
    latest_pose_.set_theta(robot_yaw);
}

void CostmapFilter::updateCosts(Costmap2D& master_grid, int min_i, int min_j,
                                int max_i, int max_j) {
    if (!enabled_) {
        return;
    }

    std::lock_guard<mutex_t> guard(*access_);
    process(master_grid, min_i, min_j, max_i, max_j, latest_pose_);
    current_ = true;
}

bool CostmapFilter::transformPose(
    const std::string global_frame,
    const automsgs::msgs::geometry_msgs::Pose2D& global_pose,
    const std::string mask_frame,
    automsgs::msgs::geometry_msgs::Pose2D& mask_pose) const {
    if (mask_frame.empty() || mask_frame == global_frame) {
        mask_pose = global_pose;
        return true;
    }

    auto* tf_buffer = autonomy::transform::Buffer::Instance();
    if (!tf_buffer) {
        AWARN << "CostmapFilter: TF buffer unavailable for " << global_frame
              << " -> " << mask_frame;
        return false;
    }

    try {
        const float timeout = static_cast<float>(
            SecondsFromDuration(transform_tolerance_));
        const auto stamped_transform = tf_buffer->lookupTransform(
            mask_frame, global_frame, LatestTfTime(), timeout);

        double mx = 0.0;
        double my = 0.0;
        double tz = 0.0;
        TransformPoint(stamped_transform.transform(), global_pose.x(),
                       global_pose.y(), 0.0, mx, my, tz);
        mask_pose.set_x(mx);
        mask_pose.set_y(my);
        mask_pose.set_theta(global_pose.theta());
        return true;
    } catch (const transform::tf2::TransformException& ex) {
        AWARN << "CostmapFilter: failed to transform pose from " << global_frame
              << " to " << mask_frame << ": " << ex.what();
        return false;
    }
}

bool CostmapFilter::worldToMask(
    std::shared_ptr<const automsgs::msgs::map_msgs::OccupancyGrid> filter_mask, double wx,
    double wy, unsigned int& mx, unsigned int& my) const {
    if (!filter_mask || filter_mask->info().resolution()<= 0.0) {
        return false;
    }

    const double origin_x = filter_mask->info().origin().position().x();
    const double origin_y = filter_mask->info().origin().position().y();
    const double resolution = filter_mask->info().resolution();
    const unsigned int size_x = filter_mask->info().width();
    const unsigned int size_y = filter_mask->info().height();

    if (wx < origin_x || wy < origin_y) {
        return false;
    }

    mx = static_cast<unsigned int>((wx - origin_x) / resolution);
    my = static_cast<unsigned int>((wy - origin_y) / resolution);
    if (mx >= size_x || my >= size_y) {
        return false;
    }

    return true;
}

unsigned char CostmapFilter::getMaskCost(
    std::shared_ptr<const automsgs::msgs::map_msgs::OccupancyGrid> filter_mask,
    const unsigned int mx, const unsigned int& my) const {
    if (!filter_mask || filter_mask->data().empty()) {
        return NO_INFORMATION;
    }

    const unsigned int index = my * filter_mask->info().width() + mx;
    if (index >= filter_mask->data().size()) {
        return NO_INFORMATION;
    }

    const int8_t data = filter_mask->data(index);
    if (data == utils::OCC_GRID_UNKNOWN) {
        return NO_INFORMATION;
    }

    const double scale =
        static_cast<double>(LETHAL_OBSTACLE - FREE_SPACE) /
        static_cast<double>(utils::OCC_GRID_OCCUPIED - utils::OCC_GRID_FREE);
    const double cost =
        static_cast<double>(data) * scale + static_cast<double>(FREE_SPACE);
    return static_cast<unsigned char>(std::lround(cost));
}

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy
