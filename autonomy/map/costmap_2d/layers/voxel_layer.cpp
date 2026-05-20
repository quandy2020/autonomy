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

#include "autonomy/map/costmap_2d/layers/voxel_layer.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "autonomy/commsgs/point_field_conversion.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_math.hpp"
#include "autonomy/map/proto/map_2d_option.pb.h"
#include "autonomy/transform/buffer.hpp"

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

class VoxelClearCell
{
public:
    VoxelClearCell(unsigned char* costmap, VoxelGrid* voxel_grid,
                   unsigned int size_x)
        : costmap_(costmap), voxel_grid_(voxel_grid), size_x_(size_x) {}

    inline void operator()(unsigned int offset) {
        costmap_[offset] = FREE_SPACE;
        if (voxel_grid_ != nullptr) {
            const unsigned int mx = offset % size_x_;
            const unsigned int my = offset / size_x_;
            voxel_grid_->clearVoxelColumn(mx, my);
        }
    }

private:
    unsigned char* costmap_;
    VoxelGrid* voxel_grid_;
    unsigned int size_x_;
};

}  // namespace

void VoxelLayer::onInitialize() {
    enabled_ = true;
    footprint_clearing_enabled_ = true;
    min_obstacle_height_ = 0.0;
    max_obstacle_height_ = 2.0;
    combination_method_ = 1;
    z_resolution_ = 0.2;
    origin_z_ = 0.0;
    size_z_ = 10;
    unknown_threshold_ = 15;
    mark_threshold_ = 0;
    publish_voxel_ = false;
    transform_tolerance_ = DurationFromSeconds(0.3);
    rolling_window_ = layered_costmap_->isRolling();
    global_frame_ = layered_costmap_->getGlobalFrameID();
    default_value_ = FREE_SPACE;
    current_ = true;
    was_reset_ = false;

    const auto* layer_options = getOptions();
    if (layer_options && layer_options->has_voxel_layer()) {
        const auto& voxel_opts = layer_options->voxel_layer();
        enabled_ = voxel_opts.enabled();
        footprint_clearing_enabled_ = voxel_opts.footprint_clearing_enabled();
        if (voxel_opts.max_obstacle_height() > 0.0) {
            max_obstacle_height_ = voxel_opts.max_obstacle_height();
        }
        if (voxel_opts.z_voxels() > 0) {
            size_z_ = voxel_opts.z_voxels();
        }
        if (voxel_opts.z_resolution() > 0.0) {
            z_resolution_ = voxel_opts.z_resolution();
        }
        origin_z_ = voxel_opts.origin_z();
        unknown_threshold_ = voxel_opts.unknown_threshold();
        mark_threshold_ = voxel_opts.mark_threshold();
        publish_voxel_ = voxel_opts.publish_voxel_map();

        auto* tf_buffer = autonomy::transform::Buffer::Instance();
        if (tf_buffer) {
            for (const auto& entry : voxel_opts.sensor_sources()) {
                const std::string& source_name = entry.first;
                const auto& src = entry.second;

                const std::string topic =
                    src.topic().empty() ? source_name : src.topic();
                const std::string data_type =
                    src.data_type().empty() ? "PointCloud2" : src.data_type();
                if (data_type != "PointCloud2" && data_type != "LaserScan") {
                    AWARN << "VoxelLayer source " << source_name
                          << ": unsupported data_type " << data_type;
                    continue;
                }

                double min_height = min_obstacle_height_;
                double max_height = max_obstacle_height_;
                if (src.min_obstacle_height() != 0.0) {
                    min_height = src.min_obstacle_height();
                } else if (src.obstacle_min_height() != 0.0) {
                    min_height = src.obstacle_min_height();
                }
                if (src.max_obstacle_height() != 0.0) {
                    max_height = src.max_obstacle_height();
                } else if (src.obstacle_max_height() != 0.0) {
                    max_height = src.obstacle_max_height();
                }

                bool marking = src.marking();
                bool clearing = src.clearing();
                if (!marking && !clearing) {
                    marking = true;
                }

                const double obstacle_max_range =
                    src.obstacle_max_height() > 0.0 ? src.obstacle_max_height()
                                                    : 2.5;
                const double obstacle_min_range = 0.0;
                const double raytrace_max_range =
                    src.raytrace_max_range() > 0.0 ? src.raytrace_max_range()
                                                   : 3.0;
                const double raytrace_min_range = src.raytrace_min_range();

                auto buffer = std::make_shared<ObservationBuffer>(
                    topic, 0.0, 0.0, min_height, max_height, obstacle_max_range,
                    obstacle_min_range, raytrace_max_range, raytrace_min_range,
                    *tf_buffer, global_frame_, "", transform_tolerance_);
                observation_buffers_.push_back(buffer);
                if (marking) {
                    marking_buffers_.push_back(buffer);
                }
                if (clearing) {
                    clearing_buffers_.push_back(buffer);
                }
            }
        } else {
            AWARN << "VoxelLayer: TF buffer unavailable, sensor buffers not "
                     "created";
        }
    }

    if (layer_options && layer_options->has_static_layer()) {
        const double tol = layer_options->static_layer().transform_tolerance();
        if (tol > 0.0) {
            transform_tolerance_ = DurationFromSeconds(tol);
        }
    }

    unknown_threshold_ += static_cast<int>(VoxelGrid::VOXEL_BITS - size_z_);
    matchSize();

    AINFO << "VoxelLayer initialized: enabled=" << enabled_
          << " z_voxels=" << size_z_ << " z_res=" << z_resolution_
          << " origin_z=" << origin_z_ << " mark_threshold=" << mark_threshold_;
}

VoxelLayer::~VoxelLayer() = default;

void VoxelLayer::matchSize() {
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
    ObstacleLayer::matchSize();
    voxel_grid_.resize(size_x_, size_y_, static_cast<unsigned int>(size_z_));
}

void VoxelLayer::reset() {
    ObstacleLayer::reset();
}

void VoxelLayer::resetMaps() {
    ObstacleLayer::resetMaps();
    voxel_grid_.reset();
}

void VoxelLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                              double* min_x, double* min_y, double* max_x,
                              double* max_y) {
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

    if (rolling_window_) {
        updateOrigin(robot_x - getSizeInMetersX() / 2,
                     robot_y - getSizeInMetersY() / 2);
    }
    if (!enabled_) {
        return;
    }

    useExtraBounds(min_x, min_y, max_x, max_y);
    resetMaps();

    bool current = true;
    std::vector<Observation> observations;
    std::vector<Observation> clearing_observations;

    current = getMarkingObservations(observations) && current;
    current = getClearingObservations(clearing_observations) && current;
    current_ = current;

    for (const Observation& clearing_observation : clearing_observations) {
        raytraceFreespace(clearing_observation, min_x, min_y, max_x, max_y);
    }

    for (const Observation& obs : observations) {
        const PointCloud2& cloud = *(obs.cloud_);
        const XYZFieldOffsets offsets = FindXYZOffsets(cloud);
        if (!offsets.valid()) {
            continue;
        }

        const double sq_obstacle_max_range =
            obs.obstacle_max_range_ * obs.obstacle_max_range_;
        const double sq_obstacle_min_range =
            obs.obstacle_min_range_ * obs.obstacle_min_range_;
        const size_t point_count = static_cast<size_t>(cloud.width) *
                                 static_cast<size_t>(cloud.height);

        for (size_t point_index = 0; point_index < point_count; ++point_index) {
            const size_t point_offset = point_index * cloud.point_step;
            const unsigned char* point_data = cloud.data.data() + point_offset;
            const double px =
                commsgs::sensor_msgs::readPointCloud2BufferValue<double>(
                    point_data + offsets.x, offsets.x_datatype);
            const double py =
                commsgs::sensor_msgs::readPointCloud2BufferValue<double>(
                    point_data + offsets.y, offsets.y_datatype);
            const double pz =
                commsgs::sensor_msgs::readPointCloud2BufferValue<double>(
                    point_data + offsets.z, offsets.z_datatype);

            if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
                continue;
            }
            if (pz > max_obstacle_height_) {
                continue;
            }

            const double sq_dist =
                (px - obs.origin_.x) * (px - obs.origin_.x) +
                (py - obs.origin_.y) * (py - obs.origin_.y) +
                (pz - obs.origin_.z) * (pz - obs.origin_.z);
            if (sq_dist >= sq_obstacle_max_range ||
                sq_dist < sq_obstacle_min_range) {
                continue;
            }

            unsigned int mx = 0;
            unsigned int my = 0;
            unsigned int mz = 0;
            if (pz < origin_z_) {
                if (!worldToMap3D(px, py, origin_z_, mx, my, mz)) {
                    continue;
                }
            } else if (!worldToMap3D(px, py, pz, mx, my, mz)) {
                continue;
            }

            if (voxel_grid_.markVoxelInMap(mx, my, mz,
                                           static_cast<unsigned int>(
                                               mark_threshold_))) {
                const unsigned int index = getIndex(mx, my);
                costmap_[index] = LETHAL_OBSTACLE;
                touch(px, py, min_x, min_y, max_x, max_y);
            }
        }
    }

    updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void VoxelLayer::raytraceFreespace(const Observation& clearing_observation,
                                   double* min_x, double* min_y, double* max_x,
                                   double* max_y) {
    const PointCloud2& cloud = *(clearing_observation.cloud_);
    if (cloud.width == 0 || cloud.height == 0) {
        return;
    }

    const double ox = clearing_observation.origin_.x;
    const double oy = clearing_observation.origin_.y;

    unsigned int x0 = 0;
    unsigned int y0 = 0;
    if (!worldToMap(ox, oy, x0, y0)) {
        return;
    }

    const double map_end_x = origin_x_ + size_x_ * resolution_;
    const double map_end_y = origin_y_ + size_y_ * resolution_;
    touch(ox, oy, min_x, min_y, max_x, max_y);

    const XYZFieldOffsets offsets = FindXYZOffsets(cloud);
    if (!offsets.valid()) {
        return;
    }

    VoxelClearCell marker(costmap_, &voxel_grid_, size_x_);
    const unsigned int cell_raytrace_max_range =
        cellDistance(clearing_observation.raytrace_max_range_);
    const unsigned int cell_raytrace_min_range =
        cellDistance(clearing_observation.raytrace_min_range_);

    const size_t point_count =
        static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height);
    for (size_t point_index = 0; point_index < point_count; ++point_index) {
        const size_t point_offset = point_index * cloud.point_step;
        const unsigned char* point_data = cloud.data.data() + point_offset;
        double wx = commsgs::sensor_msgs::readPointCloud2BufferValue<double>(
            point_data + offsets.x, offsets.x_datatype);
        double wy = commsgs::sensor_msgs::readPointCloud2BufferValue<double>(
            point_data + offsets.y, offsets.y_datatype);

        if (!std::isfinite(wx) || !std::isfinite(wy)) {
            continue;
        }

        double a = wx - ox;
        double b = wy - oy;

        if (std::abs(a) > 1e-9 && wx < origin_x_) {
            const double t = (origin_x_ - ox) / a;
            wx = origin_x_;
            wy = oy + b * t;
        }
        if (std::abs(b) > 1e-9 && wy < origin_y_) {
            const double t = (origin_y_ - oy) / b;
            wx = ox + a * t;
            wy = origin_y_;
        }
        if (std::abs(a) > 1e-9 && wx > map_end_x) {
            const double t = (map_end_x - ox) / a;
            wx = map_end_x - 0.001;
            wy = oy + b * t;
        }
        if (std::abs(b) > 1e-9 && wy > map_end_y) {
            const double t = (map_end_y - oy) / b;
            wx = ox + a * t;
            wy = map_end_y - 0.001;
        }

        unsigned int x1 = 0;
        unsigned int y1 = 0;
        if (!worldToMap(wx, wy, x1, y1)) {
            continue;
        }

        raytraceLine(marker, x0, y0, x1, y1, cell_raytrace_max_range,
                     cell_raytrace_min_range);
        updateRaytraceBounds(ox, oy, wx, wy,
                             clearing_observation.raytrace_max_range_,
                             clearing_observation.raytrace_min_range_, min_x,
                             min_y, max_x, max_y);
    }
}

void VoxelLayer::updateOrigin(double new_origin_x, double new_origin_y) {
    Costmap2D::updateOrigin(new_origin_x, new_origin_y);
    voxel_grid_.resize(size_x_, size_y_, static_cast<unsigned int>(size_z_));
}

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy
