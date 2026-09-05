/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

/**
 * @file grid.cpp
 * @brief Atomic depth fusion and safety classification for Shadow LocalGrid.
 */

#include "autonomy/perception/shadow/grid.hpp"

#include "autonomy/map/grid_map/grid_map_core/buffer_region.hpp"
#include "autonomy/map/grid_map/grid_map_core/iterators/circle_iterator.hpp"
#include "autonomy/map/grid_map/grid_map_core/iterators/grid_map_iterator.hpp"
#include "autonomy/map/grid_map/grid_map_msgs/grid_map_converter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace autonomy {
namespace perception {
namespace shadow {
namespace {

using CameraInfo = automsgs::msgs::sensor_msgs::CameraInfo;
using Image = automsgs::msgs::sensor_msgs::Image;
using Odometry = automsgs::msgs::nav_msgs::Odometry;
using TransformStamped = automsgs::msgs::geometry_msgs::TransformStamped;

constexpr double kNanosecondsPerSecond = 1.0e9;
constexpr size_t kTargetSamplesPerAxis = 160;
constexpr char kElevationLayer[] = "elevation";
constexpr char kVarianceLayer[] = "variance";
constexpr char kObstacleLayer[] = "obstacle";
constexpr char kTraversabilityLayer[] = "traversability";

struct point_3d {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct rigid_transform {
    point_3d translation;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
};

struct validated_inputs {
    size_t bytes_per_pixel = 0;
    rigid_transform transform;
    grid_map::Position robot_position = grid_map::Position::Zero();
    double support_height = 0.0;
};

void set_error(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = "Shadow grid: " + message;
    }
}

bool finite_positive(double value) {
    return std::isfinite(value) && value > 0.0;
}

bool validate_options(const proto::ShadowOptions& options, std::string* error) {
    if (options.map_frame().empty() || options.base_frame().empty() ||
        options.camera_frame().empty()) {
        set_error(error, "map, base, and camera frames must not be empty.");
        return false;
    }
    if (!finite_positive(options.map_length_x()) ||
        !finite_positive(options.map_length_y()) ||
        !finite_positive(options.map_resolution()) ||
        !finite_positive(options.map_roll_threshold()) ||
        !finite_positive(options.cell_ttl_sec()) ||
        !finite_positive(options.max_step_height()) ||
        !finite_positive(options.max_slope_rad()) ||
        !finite_positive(options.obstacle_min_height()) ||
        !finite_positive(options.robot_radius()) ||
        !finite_positive(options.inflation_radius()) ||
        options.inflation_radius() < options.robot_radius()) {
        set_error(error, "mapping options are invalid.");
        return false;
    }
    if (!finite_positive(options.min_depth_m()) ||
        !finite_positive(options.max_depth_m()) ||
        options.max_depth_m() <= options.min_depth_m() ||
        !finite_positive(options.depth_scale())) {
        set_error(error, "depth range and scale are invalid.");
        return false;
    }
    return true;
}

bool validate_depth_image(const Image& depth, size_t* bytes_per_pixel,
                          std::string* error) {
    if (depth.encoding() == "16UC1") {
        *bytes_per_pixel = sizeof(uint16_t);
    } else if (depth.encoding() == "32FC1") {
        *bytes_per_pixel = sizeof(float);
    } else {
        set_error(error, "depth encoding must be '16UC1' or '32FC1'.");
        return false;
    }
    if (depth.width() == 0 || depth.height() == 0) {
        set_error(error, "depth dimensions must be positive.");
        return false;
    }
    if (static_cast<size_t>(depth.width()) >
        std::numeric_limits<size_t>::max() / *bytes_per_pixel) {
        set_error(error, "depth row exceeds addressable size.");
        return false;
    }
    const size_t minimum_step =
        static_cast<size_t>(depth.width()) * *bytes_per_pixel;
    if (depth.step() < minimum_step) {
        set_error(error, "depth image step is too small.");
        return false;
    }
    if (static_cast<size_t>(depth.height()) >
        std::numeric_limits<size_t>::max() /
            static_cast<size_t>(depth.step())) {
        set_error(error, "depth image layout exceeds addressable size.");
        return false;
    }
    const size_t required_size =
        static_cast<size_t>(depth.height()) * depth.step();
    if (depth.data().size() < required_size) {
        set_error(error, "depth image data is too small.");
        return false;
    }
    return true;
}

bool validate_camera(const proto::ShadowOptions& options,
                     const CameraInfo& camera, const Image& depth,
                     std::string* error) {
    if (camera.width() == 0 || camera.height() == 0 ||
        camera.width() != depth.width() || camera.height() != depth.height()) {
        set_error(error, "camera dimensions must be positive and match depth.");
        return false;
    }
    if (camera.k_size() < 9 || !finite_positive(camera.k(0)) ||
        !finite_positive(camera.k(4)) || !std::isfinite(camera.k(2)) ||
        !std::isfinite(camera.k(5))) {
        set_error(error, "camera intrinsics are invalid.");
        return false;
    }
    if (depth.header().frame_id() != options.camera_frame() ||
        camera.header().frame_id() != options.camera_frame()) {
        set_error(error, "depth and camera frames must match camera_frame.");
        return false;
    }
    return true;
}

bool validate_transform(const proto::ShadowOptions& options,
                        const TransformStamped& source,
                        rigid_transform* transform, std::string* error) {
    if (source.header().frame_id() != options.map_frame() ||
        source.child_frame_id() != options.camera_frame()) {
        set_error(error, "camera-to-map transform frames are invalid.");
        return false;
    }
    const auto& translation = source.transform().translation();
    const auto& rotation = source.transform().rotation();
    transform->translation =
        point_3d{translation.x(), translation.y(), translation.z()};
    transform->qx = rotation.x();
    transform->qy = rotation.y();
    transform->qz = rotation.z();
    transform->qw = rotation.w();
    if (!std::isfinite(transform->translation.x) ||
        !std::isfinite(transform->translation.y) ||
        !std::isfinite(transform->translation.z) ||
        !std::isfinite(transform->qx) || !std::isfinite(transform->qy) ||
        !std::isfinite(transform->qz) || !std::isfinite(transform->qw)) {
        set_error(error, "camera-to-map transform is non-finite.");
        return false;
    }
    const double norm = std::sqrt(
        transform->qx * transform->qx + transform->qy * transform->qy +
        transform->qz * transform->qz + transform->qw * transform->qw);
    if (!finite_positive(norm)) {
        set_error(error, "camera-to-map rotation is invalid.");
        return false;
    }
    transform->qx /= norm;
    transform->qy /= norm;
    transform->qz /= norm;
    transform->qw /= norm;
    return true;
}

bool validate_odometry(const proto::ShadowOptions& options,
                       const Odometry& odometry,
                       grid_map::Position* robot_position,
                       double* support_height, std::string* error) {
    if (odometry.header().frame_id() != options.map_frame() ||
        odometry.child_frame_id() != options.base_frame()) {
        set_error(error, "odometry frames are invalid.");
        return false;
    }
    const auto& position = odometry.pose().pose().position();
    if (!std::isfinite(position.x()) || !std::isfinite(position.y()) ||
        !std::isfinite(position.z())) {
        set_error(error, "odometry position is non-finite.");
        return false;
    }
    *robot_position = grid_map::Position(position.x(), position.y());
    *support_height = position.z();
    return true;
}

bool validate_inputs(const proto::ShadowOptions& options, int64_t stamp_ns,
                     uint64_t current_stamp, const Image& depth,
                     const CameraInfo& camera,
                     const TransformStamped& camera_to_map,
                     const Odometry& odometry, validated_inputs* inputs,
                     std::string* error) {
    if (!validate_options(options, error)) {
        return false;
    }
    if (stamp_ns <= 0 || (current_stamp != 0 &&
                          static_cast<uint64_t>(stamp_ns) <= current_stamp)) {
        set_error(error, "timestamp must be positive and strictly increase.");
        return false;
    }
    return validate_depth_image(depth, &inputs->bytes_per_pixel, error) &&
           validate_camera(options, camera, depth, error) &&
           validate_transform(options, camera_to_map, &inputs->transform,
                              error) &&
           validate_odometry(options, odometry, &inputs->robot_position,
                             &inputs->support_height, error);
}

bool validate_roll_shift(const grid_map::GridMap& map,
                         const grid_map::Position& robot_position,
                         std::string* error) {
    const grid_map::Position normalized_shift =
        (robot_position - map.getPosition()) / map.getResolution();
    using IndexScalar = grid_map::Index::Scalar;
    constexpr double kMaxIndex =
        static_cast<double>(std::numeric_limits<IndexScalar>::max());
    for (Eigen::Index axis = 0; axis < normalized_shift.size(); ++axis) {
        const double component = normalized_shift(axis);
        const double rounded_component =
            component + 0.5 * (component > 0.0 ? 1.0 : -1.0);
        // GridMap sign-inverts this value before adding it to a circular-buffer
        // start index, so reserve headroom for every valid start index.
        const double max_start_index =
            static_cast<double>(map.getSize()(axis) - 1);
        const double safe_index_limit = kMaxIndex - max_start_index;
        if (!std::isfinite(rounded_component) || safe_index_limit < 0.0 ||
            rounded_component < -safe_index_limit ||
            rounded_component > safe_index_limit) {
            set_error(error, "odometry shift exceeds GridMap index range.");
            return false;
        }
    }
    return true;
}

grid_map::GridMap make_empty_map(const proto::ShadowOptions& options) {
    grid_map::GridMap map;
    map.setGeometry(
        grid_map::Length(options.map_length_x(), options.map_length_y()),
        options.map_resolution());
    map.setFrameId(options.map_frame());
    map.add("elevation");
    map.add("variance");
    map.add("obstacle");
    map.add("traversability");
    map.setBasicLayers({kElevationLayer, kVarianceLayer, kObstacleLayer,
                        kTraversabilityLayer});
    return map;
}

template <typename MatrixType>
void clear_regions(MatrixType* matrix,
                   const std::vector<grid_map::BufferRegion>& regions) {
    for (const auto& region : regions) {
        const auto& start = region.getStartIndex();
        const auto& size = region.getSize();
        matrix->block(start.x(), start.y(), size.x(), size.y()).setZero();
    }
}

void clear_cell(grid_map::GridMap* map, const grid_map::Index& index) {
    const float unknown = std::numeric_limits<float>::quiet_NaN();
    map->at(kElevationLayer, index) = unknown;
    map->at(kVarianceLayer, index) = unknown;
    map->at(kObstacleLayer, index) = unknown;
    map->at(kTraversabilityLayer, index) = unknown;
}

uint16_t decode_uint16(const char* data, bool big_endian) {
    const auto* bytes = reinterpret_cast<const uint8_t*>(data);
    if (big_endian) {
        return static_cast<uint16_t>((static_cast<uint16_t>(bytes[0]) << 8) |
                                     static_cast<uint16_t>(bytes[1]));
    }
    return static_cast<uint16_t>(static_cast<uint16_t>(bytes[0]) |
                                 (static_cast<uint16_t>(bytes[1]) << 8));
}

float decode_float32(const char* data, bool big_endian) {
    const auto* bytes = reinterpret_cast<const uint8_t*>(data);
    uint32_t bits = 0;
    if (big_endian) {
        bits = (static_cast<uint32_t>(bytes[0]) << 24) |
               (static_cast<uint32_t>(bytes[1]) << 16) |
               (static_cast<uint32_t>(bytes[2]) << 8) |
               static_cast<uint32_t>(bytes[3]);
    } else {
        bits = static_cast<uint32_t>(bytes[0]) |
               (static_cast<uint32_t>(bytes[1]) << 8) |
               (static_cast<uint32_t>(bytes[2]) << 16) |
               (static_cast<uint32_t>(bytes[3]) << 24);
    }
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

float depth_at(const proto::ShadowOptions& options, const Image& depth,
               size_t bytes_per_pixel, uint32_t row, uint32_t column) {
    const char* sample = depth.data().data() +
                         static_cast<size_t>(row) * depth.step() +
                         static_cast<size_t>(column) * bytes_per_pixel;
    if (depth.encoding() == "16UC1") {
        return static_cast<float>(decode_uint16(sample, depth.is_bigendian())) *
               options.depth_scale();
    }
    return decode_float32(sample, depth.is_bigendian());
}

point_3d transform_point(const point_3d& point,
                         const rigid_transform& transform) {
    const double qx = transform.qx;
    const double qy = transform.qy;
    const double qz = transform.qz;
    const double qw = transform.qw;
    point_3d result;
    result.x = transform.translation.x +
               (1.0 - 2.0 * (qy * qy + qz * qz)) * point.x +
               2.0 * (qx * qy - qz * qw) * point.y +
               2.0 * (qx * qz + qy * qw) * point.z;
    result.y = transform.translation.y + 2.0 * (qx * qy + qz * qw) * point.x +
               (1.0 - 2.0 * (qx * qx + qz * qz)) * point.y +
               2.0 * (qy * qz - qx * qw) * point.z;
    result.z = transform.translation.z + 2.0 * (qx * qz - qy * qw) * point.x +
               2.0 * (qy * qz + qx * qw) * point.y +
               (1.0 - 2.0 * (qx * qx + qy * qy)) * point.z;
    return result;
}

template <typename CountMatrix, typename TimestampMatrix>
void expire_cells(int64_t stamp_ns, float ttl_sec, grid_map::GridMap* map,
                  CountMatrix* sample_count, grid_map::Matrix* elevation_m2,
                  TimestampMatrix* last_observed_ns) {
    for (grid_map::GridMapIterator iterator(*map); !iterator.isPastEnd();
         ++iterator) {
        const grid_map::Index index = *iterator;
        const int64_t observed_ns = (*last_observed_ns)(index.x(), index.y());
        if (observed_ns == 0) {
            continue;
        }
        const double age_sec =
            static_cast<double>(stamp_ns - observed_ns) / kNanosecondsPerSecond;
        if (age_sec <= ttl_sec) {
            continue;
        }
        clear_cell(map, index);
        (*sample_count)(index.x(), index.y()) = 0;
        (*elevation_m2)(index.x(), index.y()) = 0.0F;
        (*last_observed_ns)(index.x(), index.y()) = 0;
    }
}

template <typename CountMatrix, typename TimestampMatrix>
bool insert_depth(const proto::ShadowOptions& options, int64_t stamp_ns,
                  const Image& depth, const CameraInfo& camera,
                  const validated_inputs& inputs, grid_map::GridMap* map,
                  CountMatrix* sample_count, grid_map::Matrix* elevation_m2,
                  TimestampMatrix* last_observed_ns, std::string* error) {
    const size_t maximum_dimension =
        std::max(static_cast<size_t>(depth.width()),
                 static_cast<size_t>(depth.height()));
    const size_t stride =
        std::max(size_t{1}, (maximum_dimension + kTargetSamplesPerAxis - 1) /
                                kTargetSamplesPerAxis);
    const double fx = camera.k(0);
    const double fy = camera.k(4);
    const double cx = camera.k(2);
    const double cy = camera.k(5);
    for (size_t row = 0; row < depth.height(); row += stride) {
        for (size_t column = 0; column < depth.width(); column += stride) {
            const float range_m = depth_at(
                options, depth, inputs.bytes_per_pixel,
                static_cast<uint32_t>(row), static_cast<uint32_t>(column));
            if (!std::isfinite(range_m) || range_m < options.min_depth_m() ||
                range_m > options.max_depth_m()) {
                continue;
            }
            const point_3d camera_point{
                (static_cast<double>(column) - cx) * range_m / fx,
                (static_cast<double>(row) - cy) * range_m / fy, range_m};
            const point_3d measured =
                transform_point(camera_point, inputs.transform);
            if (!std::isfinite(measured.x) || !std::isfinite(measured.y) ||
                !std::isfinite(measured.z) ||
                std::abs(measured.z) > std::numeric_limits<float>::max()) {
                set_error(error,
                          "transformed depth point is not representable.");
                return false;
            }
            grid_map::Index index;
            if (!map->getIndex(grid_map::Position(measured.x, measured.y),
                               index)) {
                continue;
            }
            auto& count = (*sample_count)(index.x(), index.y());
            auto& mean = map->at(kElevationLayer, index);
            auto& m2 = (*elevation_m2)(index.x(), index.y());
            if (count == std::numeric_limits<uint32_t>::max()) {
                set_error(error, "elevation sample count is exhausted.");
                return false;
            }
            if (count == 0 || !std::isfinite(mean)) {
                count = 1;
                mean = static_cast<float>(measured.z);
                m2 = 0.0F;
            } else {
                ++count;
                const double delta = measured.z - mean;
                mean = static_cast<float>(mean + delta / count);
                m2 = static_cast<float>(m2 + delta * (measured.z - mean));
            }
            map->at(kVarianceLayer, index) =
                std::max(0.0F, m2 / static_cast<float>(count));
            (*last_observed_ns)(index.x(), index.y()) = stamp_ns;
        }
    }
    return true;
}

float terrain_cost(const proto::ShadowOptions& options,
                   const grid_map::GridMap& map, const grid_map::Index& index) {
    static constexpr std::array<std::array<int, 2>, 8> kNeighborOffsets = {{
        {{-1, -1}},
        {{-1, 0}},
        {{-1, 1}},
        {{0, -1}},
        {{0, 1}},
        {{1, -1}},
        {{1, 0}},
        {{1, 1}},
    }};
    grid_map::Position position;
    if (!map.getPosition(index, position)) {
        return 1.0F;
    }
    const float elevation = map.at(kElevationLayer, index);
    float cost = 0.0F;
    for (const auto& offset : kNeighborOffsets) {
        const double dx = offset[0] * map.getResolution();
        const double dy = offset[1] * map.getResolution();
        grid_map::Index neighbor;
        if (!map.getIndex(
                grid_map::Position(position.x() + dx, position.y() + dy),
                neighbor)) {
            continue;
        }
        const float neighbor_elevation = map.at(kElevationLayer, neighbor);
        if (!std::isfinite(neighbor_elevation)) {
            continue;
        }
        const double height_difference =
            std::abs(static_cast<double>(elevation - neighbor_elevation));
        const double distance = std::hypot(dx, dy);
        const double step_cost = height_difference / options.max_step_height();
        const double slope_cost =
            std::atan2(height_difference, distance) / options.max_slope_rad();
        cost =
            std::max(cost, static_cast<float>(std::max(step_cost, slope_cost)));
    }
    return std::clamp(cost, 0.0F, 1.0F);
}

void derive_safety_layers(const proto::ShadowOptions& options,
                          double support_height, grid_map::GridMap* map) {
    std::vector<grid_map::Position> impassable_positions;
    for (grid_map::GridMapIterator iterator(*map); !iterator.isPastEnd();
         ++iterator) {
        const grid_map::Index index = *iterator;
        const float elevation = map->at(kElevationLayer, index);
        const float variance = map->at(kVarianceLayer, index);
        if (!std::isfinite(elevation) || !std::isfinite(variance)) {
            clear_cell(map, index);
            continue;
        }
        const bool obstacle = static_cast<double>(elevation) - support_height >
                              options.obstacle_min_height();
        map->at(kObstacleLayer, index) = obstacle ? 1.0F : 0.0F;
        map->at(kTraversabilityLayer, index) =
            obstacle ? 1.0F : terrain_cost(options, *map, index);
        if (map->at(kTraversabilityLayer, index) >= 1.0F) {
            grid_map::Position position;
            if (map->getPosition(index, position)) {
                impassable_positions.push_back(position);
            }
        }
    }

    const double hard_radius = options.robot_radius();
    const double inflation_radius = options.inflation_radius();
    for (const auto& obstacle_position : impassable_positions) {
        for (grid_map::CircleIterator iterator(*map, obstacle_position,
                                               inflation_radius);
             !iterator.isPastEnd(); ++iterator) {
            const grid_map::Index index = *iterator;
            auto& traversability = map->at(kTraversabilityLayer, index);
            if (!std::isfinite(traversability)) {
                continue;
            }
            grid_map::Position position;
            if (!map->getPosition(index, position)) {
                continue;
            }
            const double distance = (position - obstacle_position).norm();
            float inflation_cost = 1.0F;
            if (distance > hard_radius && inflation_radius > hard_radius) {
                inflation_cost =
                    static_cast<float>((inflation_radius - distance) /
                                       (inflation_radius - hard_radius));
            }
            traversability = std::max(traversability,
                                      std::clamp(inflation_cost, 0.0F, 1.0F));
        }
    }
}

}  // namespace

LocalGrid::LocalGrid(const proto::ShadowOptions& options)
    : options_(options), map_(make_empty_map(options)) {
    const Eigen::Index rows = map_.getSize().x();
    const Eigen::Index columns = map_.getSize().y();
    sample_count_ = CountMatrix::Zero(rows, columns);
    elevation_m2_ = grid_map::Matrix::Zero(rows, columns);
    last_observed_ns_ = TimestampMatrix::Zero(rows, columns);
}

bool LocalGrid::Update(int64_t stamp_ns, const Image& depth,
                       const CameraInfo& camera,
                       const TransformStamped& camera_to_map,
                       const Odometry& odometry, std::string* error) {
    if (error != nullptr) {
        error->clear();
    }
    validated_inputs inputs;
    if (!validate_inputs(options_, stamp_ns, map_.getTimestamp(), depth, camera,
                         camera_to_map, odometry, &inputs, error)) {
        return false;
    }

    const bool should_roll =
        (inputs.robot_position - map_.getPosition()).norm() >
        options_.map_roll_threshold();
    if (should_roll &&
        !validate_roll_shift(map_, inputs.robot_position, error)) {
        return false;
    }

    grid_map::GridMap candidate_map = map_;
    CountMatrix candidate_count = sample_count_;
    grid_map::Matrix candidate_m2 = elevation_m2_;
    TimestampMatrix candidate_observed_ns = last_observed_ns_;
    if (should_roll) {
        std::vector<grid_map::BufferRegion> new_regions;
        candidate_map.move(inputs.robot_position, new_regions);
        clear_regions(&candidate_count, new_regions);
        clear_regions(&candidate_m2, new_regions);
        clear_regions(&candidate_observed_ns, new_regions);
    }

    expire_cells(stamp_ns, options_.cell_ttl_sec(), &candidate_map,
                 &candidate_count, &candidate_m2, &candidate_observed_ns);
    if (!insert_depth(options_, stamp_ns, depth, camera, inputs, &candidate_map,
                      &candidate_count, &candidate_m2, &candidate_observed_ns,
                      error)) {
        return false;
    }
    derive_safety_layers(options_, inputs.support_height, &candidate_map);
    candidate_map.setFrameId(options_.map_frame());
    candidate_map.setTimestamp(static_cast<grid_map::Time>(stamp_ns));

    map_ = std::move(candidate_map);
    sample_count_ = std::move(candidate_count);
    elevation_m2_ = std::move(candidate_m2);
    last_observed_ns_ = std::move(candidate_observed_ns);
    return true;
}

const grid_map::GridMap& LocalGrid::map() const {
    return map_;
}

bool LocalGrid::ToMessage(automsgs::msgs::map_msgs::GridMap* message,
                          std::string* error) const {
    if (error != nullptr) {
        error->clear();
    }
    if (message == nullptr) {
        set_error(error, "message output is null.");
        return false;
    }
    message->Clear();
    grid_map::GridMapConverter::toMessage(map_, *message);
    return true;
}

void LocalGrid::Clear() {
    map_ = make_empty_map(options_);
    const Eigen::Index rows = map_.getSize().x();
    const Eigen::Index columns = map_.getSize().y();
    sample_count_ = CountMatrix::Zero(rows, columns);
    elevation_m2_ = grid_map::Matrix::Zero(rows, columns);
    last_observed_ns_ = TimestampMatrix::Zero(rows, columns);
}

}  // namespace shadow
}  // namespace perception
}  // namespace autonomy
