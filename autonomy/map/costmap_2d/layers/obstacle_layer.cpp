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

#include "autonomy/map/costmap_2d/layers/obstacle_layer.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <automsgs/msgs/sensor_msgs/point_field_conversion.hpp>
#include "autonomy/common/logging.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_math.hpp"
#include "autonomy/map/proto/map_2d_option.pb.h"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {
namespace {

using automsgs::msgs::sensor_msgs::LaserScan;
using automsgs::msgs::sensor_msgs::PointCloud2;
using automsgs::msgs::sensor_msgs::PointField;

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
    offsets.point_step = static_cast<int>(cloud.point_step());
    for (const auto& field : cloud.fields()) {
        if (field.name() == "x") {
            offsets.x = static_cast<int>(field.offset());
            offsets.x_datatype = static_cast<uint8_t>(field.datatype());
        } else if (field.name() == "y") {
            offsets.y = static_cast<int>(field.offset());
            offsets.y_datatype = static_cast<uint8_t>(field.datatype());
        } else if (field.name() == "z") {
            offsets.z = static_cast<int>(field.offset());
            offsets.z_datatype = static_cast<uint8_t>(field.datatype());
        }
    }
    return offsets;
}

void ProjectLaserScanToPointCloud2(const LaserScan& scan, PointCloud2& cloud) {
    *cloud.mutable_header() = scan.header();
    cloud.set_height(1);
    cloud.set_width(static_cast<uint32_t>(scan.ranges_size()));
    cloud.set_is_bigendian(false);
    cloud.set_is_dense(false);
    cloud.clear_fields();
    auto add_field = [&cloud](const std::string& name, uint32_t offset) {
        auto* field = cloud.add_fields();
        field->set_name(name);
        field->set_offset(offset);
        field->set_datatype(PointField::FLOAT32);
        field->set_count(1);
    };
    add_field("x", 0);
    add_field("y", 4);
    add_field("z", 8);
    cloud.set_point_step(12);
    cloud.set_row_step(cloud.point_step() * cloud.width());
    cloud.mutable_data()->resize(cloud.row_step());

    float angle = scan.angle_min();
    for (int i = 0; i < scan.ranges_size(); ++i) {
        const float range = scan.ranges(i);
        float* point = reinterpret_cast<float*>(
            &(*cloud.mutable_data())[i * cloud.point_step()]);
        if (!std::isfinite(range) || range < scan.range_min() ||
            range > scan.range_max()) {
            point[0] = std::numeric_limits<float>::quiet_NaN();
            point[1] = std::numeric_limits<float>::quiet_NaN();
            point[2] = std::numeric_limits<float>::quiet_NaN();
        } else {
            point[0] = range * std::cos(angle);
            point[1] = range * std::sin(angle);
            point[2] = 0.0f;
        }
        angle += scan.angle_increment();
    }
}

void MarkObstaclePoints(const PointCloud2& cloud, const Observation& obs,
                        double min_obstacle_height, double max_obstacle_height,
                        unsigned char* costmap,
                        const std::function<bool(double, double, unsigned int&,
                                                 unsigned int&)>& world_to_map,
                        const std::function<unsigned int(unsigned int, unsigned int)>&
                            get_index,
                        const std::function<void(double, double, double*, double*,
                                                 double*, double*)>& touch,
                        double* min_x, double* min_y, double* max_x,
                        double* max_y) {
    const XYZFieldOffsets offsets = FindXYZOffsets(cloud);
    if (!offsets.valid()) {
        return;
    }

    const double sq_obstacle_max_range =
        obs.obstacle_max_range_ * obs.obstacle_max_range_;
    const double sq_obstacle_min_range =
        obs.obstacle_min_range_ * obs.obstacle_min_range_;
    const size_t point_count =
        static_cast<size_t>(cloud.width()) * static_cast<size_t>(cloud.height());

    for (size_t point_index = 0; point_index < point_count; ++point_index) {
        const size_t point_offset = point_index * cloud.point_step();
        const unsigned char* point_data = reinterpret_cast<const unsigned char*>(cloud.data().data()) + point_offset;
        const double px =
            automsgs::msgs::sensor_msgs::readPointCloud2BufferValue<double>(
                point_data + offsets.x, offsets.x_datatype);
        const double py =
            automsgs::msgs::sensor_msgs::readPointCloud2BufferValue<double>(
                point_data + offsets.y, offsets.y_datatype);
        const double pz =
            automsgs::msgs::sensor_msgs::readPointCloud2BufferValue<double>(
                point_data + offsets.z, offsets.z_datatype);

        if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
            continue;
        }
        if (pz < min_obstacle_height || pz > max_obstacle_height) {
            continue;
        }

        const double sq_dist =
            (px - obs.origin_.x()) * (px - obs.origin_.x()) +
            (py - obs.origin_.y()) * (py - obs.origin_.y()) +
            (pz - obs.origin_.z()) * (pz - obs.origin_.z());
        if (sq_dist >= sq_obstacle_max_range || sq_dist < sq_obstacle_min_range) {
            continue;
        }

        unsigned int mx = 0;
        unsigned int my = 0;
        if (!world_to_map(px, py, mx, my)) {
            continue;
        }

        const unsigned int index = get_index(mx, my);
        costmap[index] = LETHAL_OBSTACLE;
        touch(px, py, min_x, min_y, max_x, max_y);
    }
}

}  // namespace

ObstacleLayer::~ObstacleLayer() = default;

void ObstacleLayer::onInitialize() {
    enabled_ = true;
    footprint_clearing_enabled_ = true;
    min_obstacle_height_ = 0.0;
    max_obstacle_height_ = 2.0;
    combination_method_ = 1;
    transform_tolerance_ = DurationFromSeconds(0.3);
    rolling_window_ = layered_costmap_->isRolling();
    global_frame_ = layered_costmap_->getGlobalFrameID();
    default_value_ = FREE_SPACE;
    current_ = true;
    was_reset_ = false;

    const auto* layer_options = getOptions();
    if (layer_options && layer_options->has_obstacle_layer()) {
        const auto& obstacle_opts = layer_options->obstacle_layer();
        enabled_ = obstacle_opts.enabled();
        footprint_clearing_enabled_ =
            obstacle_opts.footprint_clearing_enabled();

        auto* tf_buffer = autonomy::transform::Buffer::Instance();
        if (!tf_buffer) {
            AWARN << "ObstacleLayer: TF buffer unavailable, sensor buffers "
                     "not created";
            matchSize();
            return;
        }

        for (const auto& entry : obstacle_opts.sensor_sources()) {
            const std::string& source_name = entry.first;
            const auto& src = entry.second;

            const std::string topic =
                src.topic().empty() ? source_name : src.topic();
            const std::string data_type =
                src.data_type().empty() ? "PointCloud2" : src.data_type();
            if (data_type != "PointCloud2" && data_type != "LaserScan") {
                AWARN << "ObstacleLayer source " << source_name
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
            // Prefer explicit raytrace_max_range as obstacle mark range.
            // Do NOT reuse obstacle_max_height (meters of height ≠ range).
            const double obstacle_max_range =
                src.raytrace_max_range() > 0.0 ? src.raytrace_max_range()
                                               : 25.0;
            const double obstacle_min_range = 0.0;
            const double raytrace_max_range =
                src.raytrace_max_range() > 0.0 ? src.raytrace_max_range() : 25.0;
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
    }

    if (layer_options && layer_options->has_static_layer()) {
        const double tol = layer_options->static_layer().transform_tolerance();
        if (tol > 0.0) {
            transform_tolerance_ = DurationFromSeconds(tol);
        }
    }

    matchSize();
}

void ObstacleLayer::feedLaserScan(const automsgs::msgs::sensor_msgs::LaserScan& scan) {
    if (!enabled_ || marking_buffers_.empty()) {
        return;
    }
    const auto message = std::make_shared<LaserScan>(scan);
    for (const auto& buffer : marking_buffers_) {
        if (buffer) {
            laserScanCallback(message, buffer);
        }
    }
}

void ObstacleLayer::feedPointCloud2(
    const automsgs::msgs::sensor_msgs::PointCloud2& cloud) {
    if (!enabled_ || marking_buffers_.empty()) {
        return;
    }
    const auto message = std::make_shared<PointCloud2>(cloud);
    for (const auto& buffer : marking_buffers_) {
        if (buffer) {
            pointCloud2Callback(message, buffer);
        }
    }
}

void ObstacleLayer::laserScanCallback(
    std::shared_ptr<const automsgs::msgs::sensor_msgs::LaserScan> message,
    const std::shared_ptr<ObservationBuffer>& buffer) {
    PointCloud2 cloud;
    ProjectLaserScanToPointCloud2(*message, cloud);
    buffer->lock();
    buffer->bufferCloud(cloud);
    buffer->unlock();
}

void ObstacleLayer::laserScanValidInfCallback(
    std::shared_ptr<const automsgs::msgs::sensor_msgs::LaserScan> raw_message,
    const std::shared_ptr<ObservationBuffer>& buffer) {
    constexpr float kEpsilon = 0.0001f;
    LaserScan message = *raw_message;
    for (int i = 0; i < message.ranges_size(); ++i) {
        float range = message.ranges(i);
        if (!std::isfinite(range) && range > 0.0f) {
            message.set_ranges(i, message.range_max() - kEpsilon);
        }
    }
    PointCloud2 cloud;
    ProjectLaserScanToPointCloud2(message, cloud);
    buffer->lock();
    buffer->bufferCloud(cloud);
    buffer->unlock();
}

void ObstacleLayer::pointCloud2Callback(
    std::shared_ptr<const automsgs::msgs::sensor_msgs::PointCloud2> message,
    const std::shared_ptr<ObservationBuffer>& buffer) {
    buffer->lock();
    buffer->bufferCloud(*message);
    buffer->unlock();
}

void ObstacleLayer::updateBounds(double robot_x, double robot_y,
                                 double robot_yaw, double* min_x, double* min_y,
                                 double* max_x, double* max_y) {
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

    auto world_to_map = [this](double wx, double wy, unsigned int& mx,
                               unsigned int& my) {
        return worldToMap(wx, wy, mx, my);
    };
    auto get_index = [this](unsigned int mx, unsigned int my) {
        return getIndex(mx, my);
    };
    auto touch_fn = [this](double wx, double wy, double* minx, double* miny,
                           double* maxx, double* maxy) {
        touch(wx, wy, minx, miny, maxx, maxy);
    };

    for (const Observation& obs : observations) {
        MarkObstaclePoints(*obs.cloud_, obs, min_obstacle_height_,
                           max_obstacle_height_, costmap_, world_to_map,
                           get_index, touch_fn, min_x, min_y,
                           max_x, max_y);
    }

    updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void ObstacleLayer::updateFootprint(double robot_x, double robot_y,
                                    double robot_yaw, double* min_x,
                                    double* min_y, double* max_x,
                                    double* max_y) {
    if (!footprint_clearing_enabled_) {
        return;
    }
    transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(),
                       transformed_footprint_);

    for (unsigned int i = 0; i < transformed_footprint_.size(); i++) {
        touch(transformed_footprint_[i].x(), transformed_footprint_[i].y(), min_x,
              min_y, max_x, max_y);
    }
}

void ObstacleLayer::updateCosts(Costmap2D& master_grid, int min_i, int min_j,
                                int max_i, int max_j) {
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
    if (!enabled_) {
        return;
    }

    if (!current_ && was_reset_) {
        was_reset_ = false;
        current_ = true;
    }

    if (footprint_clearing_enabled_) {
        setConvexPolygonCost(transformed_footprint_, FREE_SPACE);
    }

    switch (combination_method_) {
        case 0:
            updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
            break;
        case 1:
            updateWithMax(master_grid, min_i, min_j, max_i, max_j);
            break;
        case 2:
            updateWithMaxWithoutUnknownOverwrite(master_grid, min_i, min_j,
                                                 max_i, max_j);
            break;
        default:
            break;
    }
}

void ObstacleLayer::addStaticObservation(Observation& obs, bool marking,
                                         bool clearing) {
    if (marking) {
        static_marking_observations_.push_back(obs);
    }
    if (clearing) {
        static_clearing_observations_.push_back(obs);
    }
}

void ObstacleLayer::clearStaticObservations(bool marking, bool clearing) {
    if (marking) {
        static_marking_observations_.clear();
    }
    if (clearing) {
        static_clearing_observations_.clear();
    }
}

bool ObstacleLayer::getMarkingObservations(
    std::vector<Observation>& marking_observations) const {
    bool current = true;
    for (unsigned int i = 0; i < marking_buffers_.size(); ++i) {
        marking_buffers_[i]->lock();
        marking_buffers_[i]->getObservations(marking_observations);
        current = marking_buffers_[i]->isCurrent() && current;
        marking_buffers_[i]->unlock();
    }
    marking_observations.insert(marking_observations.end(),
                                static_marking_observations_.begin(),
                                static_marking_observations_.end());
    return current;
}

bool ObstacleLayer::getClearingObservations(
    std::vector<Observation>& clearing_observations) const {
    bool current = true;
    for (unsigned int i = 0; i < clearing_buffers_.size(); ++i) {
        clearing_buffers_[i]->lock();
        clearing_buffers_[i]->getObservations(clearing_observations);
        current = clearing_buffers_[i]->isCurrent() && current;
        clearing_buffers_[i]->unlock();
    }
    clearing_observations.insert(clearing_observations.end(),
                                 static_clearing_observations_.begin(),
                                 static_clearing_observations_.end());
    return current;
}

void ObstacleLayer::raytraceFreespace(const Observation& clearing_observation,
                                      double* min_x, double* min_y,
                                      double* max_x, double* max_y) {
    const double ox = clearing_observation.origin_.x();
    const double oy = clearing_observation.origin_.y();
    const PointCloud2& cloud = *(clearing_observation.cloud_);

    unsigned int x0 = 0;
    unsigned int y0 = 0;
    if (!worldToMap(ox, oy, x0, y0)) {
        AWARN << "ObstacleLayer: sensor origin (" << ox << ", " << oy
              << ") out of map bounds";
        return;
    }

    const double map_end_x = origin_x_ + size_x_ * resolution_;
    const double map_end_y = origin_y_ + size_y_ * resolution_;
    touch(ox, oy, min_x, min_y, max_x, max_y);

    const XYZFieldOffsets offsets = FindXYZOffsets(cloud);
    if (!offsets.valid()) {
        return;
    }

    const size_t point_count =
        static_cast<size_t>(cloud.width()) * static_cast<size_t>(cloud.height());
    MarkCell marker(costmap_, FREE_SPACE);
    const unsigned int cell_raytrace_max_range = cellDistance(
        clearing_observation.raytrace_max_range_);
    const unsigned int cell_raytrace_min_range = cellDistance(
        clearing_observation.raytrace_min_range_);

    for (size_t point_index = 0; point_index < point_count; ++point_index) {
        const size_t point_offset = point_index * cloud.point_step();
        const unsigned char* point_data = reinterpret_cast<const unsigned char*>(cloud.data().data()) + point_offset;
        double wx = automsgs::msgs::sensor_msgs::readPointCloud2BufferValue<double>(
            point_data + offsets.x, offsets.x_datatype);
        double wy = automsgs::msgs::sensor_msgs::readPointCloud2BufferValue<double>(
            point_data + offsets.y, offsets.y_datatype);

        if (!std::isfinite(wx) || !std::isfinite(wy)) {
            continue;
        }

        double a = wx - ox;
        double b = wy - oy;

        if (wx < origin_x_) {
            const double t = (origin_x_ - ox) / a;
            wx = origin_x_;
            wy = oy + b * t;
        }
        if (wy < origin_y_) {
            const double t = (origin_y_ - oy) / b;
            wx = ox + a * t;
            wy = origin_y_;
        }
        if (wx > map_end_x) {
            const double t = (map_end_x - ox) / a;
            wx = map_end_x - 0.001;
            wy = oy + b * t;
        }
        if (wy > map_end_y) {
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

void ObstacleLayer::activate() {
    resetBuffersLastUpdated();
}

void ObstacleLayer::deactivate() {}

void ObstacleLayer::updateRaytraceBounds(double ox, double oy, double wx,
                                         double wy, double max_range,
                                         double min_range, double* min_x,
                                         double* min_y, double* max_x,
                                         double* max_y) {
    const double dx = wx - ox;
    const double dy = wy - oy;
    const double full_distance = hypot(dx, dy);
    if (full_distance < min_range) {
        return;
    }
    const double scale = std::min(1.0, max_range / full_distance);
    const double ex = ox + dx * scale;
    const double ey = oy + dy * scale;
    touch(ex, ey, min_x, min_y, max_x, max_y);
}

void ObstacleLayer::reset() {
    resetMaps();
    resetBuffersLastUpdated();
    current_ = false;
    was_reset_ = true;
}

void ObstacleLayer::resetBuffersLastUpdated() {
    for (const auto& buffer : observation_buffers_) {
        if (buffer) {
            buffer->resetLastUpdated();
        }
    }
}

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy
