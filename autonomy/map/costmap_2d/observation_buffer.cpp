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

#include "autonomy/map/costmap_2d/observation_buffer.hpp"

#include "autonomy/map/costmap_2d/costmap_math.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <list>
#include <string>
#include <vector>

#include "autonomy/commsgs/point_field_conversion.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/transform/tf2/convert.h"

namespace autonomy {
namespace map {
namespace costmap_2d {
namespace {

using commsgs::sensor_msgs::PointCloud2;
using commsgs::sensor_msgs::PointField;

struct XYZFieldOffsets {
    int x{-1};
    int y{-1};
    int z{-1};
    uint8_t x_datatype{PointField::FLOAT32};
    uint8_t y_datatype{PointField::FLOAT32};
    uint8_t z_datatype{PointField::FLOAT32};
    int point_step{0};
    bool valid() const { return x >= 0 && y >= 0 && z >= 0 && point_step > 0; }
};

XYZFieldOffsets FindXYZOffsets(const PointCloud2& cloud) {
    XYZFieldOffsets offsets;
    offsets.point_step = static_cast<int>(cloud.point_step);
    for (const auto& field : cloud.fields) {
        if (field.name == "x") {
            offsets.x = static_cast<int>(field.offset);
            offsets.x_datatype = field.datatype;
        } else if (field.name == "y") {
            offsets.y = static_cast<int>(field.offset);
            offsets.y_datatype = field.datatype;
        } else if (field.name == "z") {
            offsets.z = static_cast<int>(field.offset);
            offsets.z_datatype = field.datatype;
        }
    }
    return offsets;
}

void TransformPoint(const commsgs::geometry_msgs::Transform& transform,
                    double x, double y, double z, double& out_x, double& out_y,
                    double& out_z) {
    const auto& q = transform.rotation;
    const auto& t = transform.translation;
    const double qx = q.x;
    const double qy = q.y;
    const double qz = q.z;
    const double qw = q.w;
    const double ix = qw * x + qy * z - qz * y;
    const double iy = qw * y + qz * x - qx * z;
    const double iz = qw * z + qx * y - qy * x;
    const double iw = -qx * x - qy * y - qz * z;
    out_x = ix * qw + iw * -qx + iy * -qz - iz * -qy + t.x;
    out_y = iy * qw + iw * -qy + iz * -qx - ix * -qz + t.y;
    out_z = iz * qw + iw * -qz + ix * -qy - iy * -qx + t.z;
}

float TimeoutSeconds(transform::tf2::Duration tolerance) {
    return static_cast<float>(SecondsFromDuration(tolerance));
}

}  // namespace

ObservationBuffer::ObservationBuffer(
    std::string topic_name, double observation_keep_time,
    double expected_update_rate, double min_obstacle_height,
    double max_obstacle_height, double obstacle_max_range,
    double obstacle_min_range, double raytrace_max_range,
    double raytrace_min_range, TfBuffer& tf_buffer, std::string global_frame,
    std::string sensor_frame, transform::tf2::Duration tf_tolerance)
    : tf_buffer_(tf_buffer),
      observation_keep_time_(observation_keep_time),
      expected_update_rate_(expected_update_rate),
      global_frame_(global_frame),
      sensor_frame_(sensor_frame),
      topic_name_(topic_name),
      min_obstacle_height_(min_obstacle_height),
      max_obstacle_height_(max_obstacle_height),
      obstacle_max_range_(obstacle_max_range),
      obstacle_min_range_(obstacle_min_range),
      raytrace_max_range_(raytrace_max_range),
      raytrace_min_range_(raytrace_min_range),
      tf_tolerance_(tf_tolerance) {
    last_updated_steady_ = std::chrono::steady_clock::now();
}

ObservationBuffer::~ObservationBuffer() {}

void ObservationBuffer::bufferCloud(
    const commsgs::sensor_msgs::PointCloud2& cloud) {
    observation_list_.push_front(Observation());
    Observation& observation = observation_list_.front();

    const std::string origin_frame =
        sensor_frame_.empty() ? cloud.header.frame_id : sensor_frame_;
    const float timeout = TimeoutSeconds(tf_tolerance_);

    try {
        if (origin_frame == global_frame_) {
            observation.origin_.x = 0.0;
            observation.origin_.y = 0.0;
            observation.origin_.z = 0.0;
        } else {
            const auto origin_transform = tf_buffer_.lookupTransform(
                global_frame_, origin_frame, cloud.header.stamp, timeout);
            TransformPoint(origin_transform.transform, 0.0, 0.0, 0.0,
                           observation.origin_.x, observation.origin_.y,
                           observation.origin_.z);
        }

        observation.raytrace_max_range_ = raytrace_max_range_;
        observation.raytrace_min_range_ = raytrace_min_range_;
        observation.obstacle_max_range_ = obstacle_max_range_;
        observation.obstacle_min_range_ = obstacle_min_range_;

        const XYZFieldOffsets offsets = FindXYZOffsets(cloud);
        if (!offsets.valid()) {
            observation_list_.pop_front();
            AWARN << "ObservationBuffer " << topic_name_
                  << ": point cloud missing x/y/z fields";
            return;
        }

        commsgs::geometry_msgs::Transform cloud_transform;
        bool need_transform = cloud.header.frame_id != global_frame_;
        if (need_transform) {
            const auto stamped_transform = tf_buffer_.lookupTransform(
                global_frame_, cloud.header.frame_id, cloud.header.stamp,
                timeout);
            cloud_transform = stamped_transform.transform;
        }

        PointCloud2& observation_cloud = *(observation.cloud_);
        observation_cloud = cloud;
        observation_cloud.header.frame_id = global_frame_;
        observation_cloud.data.clear();

        const size_t point_count =
            static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height);
        observation_cloud.data.reserve(point_count * cloud.point_step);

        for (size_t point_index = 0; point_index < point_count; ++point_index) {
            const size_t point_offset = point_index * cloud.point_step;
            const unsigned char* point_data = cloud.data.data() + point_offset;
            double px = commsgs::sensor_msgs::readPointCloud2BufferValue<double>(
                point_data + offsets.x, offsets.x_datatype);
            double py = commsgs::sensor_msgs::readPointCloud2BufferValue<double>(
                point_data + offsets.y, offsets.y_datatype);
            double pz = commsgs::sensor_msgs::readPointCloud2BufferValue<double>(
                point_data + offsets.z, offsets.z_datatype);

            if (need_transform) {
                double tx = 0.0;
                double ty = 0.0;
                double tz = 0.0;
                TransformPoint(cloud_transform, px, py, pz, tx, ty, tz);
                px = tx;
                py = ty;
                pz = tz;
            }

            if (pz < min_obstacle_height_ || pz > max_obstacle_height_) {
                continue;
            }

            observation_cloud.data.insert(
                observation_cloud.data.end(), point_data,
                point_data + cloud.point_step);
        }

        observation_cloud.width =
            observation_cloud.point_step > 0
                ? static_cast<uint32_t>(observation_cloud.data.size() /
                                        observation_cloud.point_step)
                : 0;
        observation_cloud.height = 1;
        observation_cloud.row_step =
            observation_cloud.width * observation_cloud.point_step;
        observation_cloud.is_dense = false;
    } catch (const transform::tf2::TransformException& ex) {
        observation_list_.pop_front();
        AWARN << "ObservationBuffer " << topic_name_
              << " TF error (" << origin_frame << " -> " << global_frame_
              << "): " << ex.what();
        return;
    }

    last_updated_steady_ = std::chrono::steady_clock::now();
    purgeStaleObservations();
}

// returns a copy of the observations
void ObservationBuffer::getObservations(
    std::vector<Observation>& observations) {
    // first... let's make sure that we don't have any stale observations
    purgeStaleObservations();

    // now we'll just copy the observations for the caller
    std::list<Observation>::iterator obs_it;
    for (obs_it = observation_list_.begin(); obs_it != observation_list_.end();
         ++obs_it) {
        observations.push_back(*obs_it);
    }
}

void ObservationBuffer::purgeStaleObservations() {
    if (observation_list_.empty()) {
        return;
    }

    if (observation_keep_time_ <= 0.0) {
        auto next = std::next(observation_list_.begin());
        observation_list_.erase(next, observation_list_.end());
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const auto keep_duration =
        std::chrono::duration<double>(observation_keep_time_);
    for (auto obs_it = observation_list_.begin();
         obs_it != observation_list_.end();) {
        const auto age = now - last_updated_steady_;
        if (age > keep_duration) {
            obs_it = observation_list_.erase(obs_it);
        } else {
            ++obs_it;
        }
    }
}

bool ObservationBuffer::isCurrent() const {
    if (expected_update_rate_ <= 0.0) {
        return true;
    }

    const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_updated_steady_);
    if (elapsed.count() > expected_update_rate_) {
        AWARN << "ObservationBuffer " << topic_name_
              << " not updated for " << elapsed.count()
              << "s (expected every " << expected_update_rate_ << "s)";
        return false;
    }
    return true;
}

void ObservationBuffer::resetLastUpdated() {
    last_updated_steady_ = std::chrono::steady_clock::now();
}

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy