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
 * @file planner.cpp
 * @brief Map-frame footprint filtering and seven-term Shadow path ranking.
 */

#include "autonomy/perception/shadow/planner.hpp"

#include "autonomy/map/grid_map/grid_map_core/iterators/circle_iterator.hpp"
#include "autonomy/map/grid_map/grid_map_core/iterators/grid_map_iterator.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <limits>
#include <numeric>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace autonomy {
namespace perception {
namespace shadow {
namespace {

using Odometry = automsgs::msgs::nav_msgs::Odometry;
using Path = automsgs::msgs::nav_msgs::Path;
using Pose = automsgs::msgs::geometry_msgs::Pose;
using PoseStamped = automsgs::msgs::geometry_msgs::PoseStamped;
using Quaternion = automsgs::msgs::geometry_msgs::Quaternion;

constexpr int64_t kNanosecondsPerSecond = 1'000'000'000;
constexpr double kPi = 3.14159265358979323846;
constexpr double kObstacleThreshold = 0.5;
constexpr double kImpassableThreshold = 1.0;
constexpr double kComparisonTolerance = 1.0e-12;
constexpr size_t kCostCount = 7;
constexpr size_t kLearnedCost = 0;
constexpr size_t kClearanceCost = 1;
constexpr size_t kTraversabilityCost = 2;
constexpr size_t kCurvatureCost = 3;
constexpr size_t kProgressCost = 4;
constexpr size_t kDistanceCost = 5;
constexpr size_t kVisibilityCost = 6;
constexpr char kElevationLayer[] = "elevation";
constexpr char kVarianceLayer[] = "variance";
constexpr char kObstacleLayer[] = "obstacle";
constexpr char kTraversabilityLayer[] = "traversability";

struct RobotTransform {
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
};

struct ClearanceField {
    int rows = 0;
    int columns = 0;
    std::vector<double> values;

    size_t Id(const grid_map::Index& index) const {
        return static_cast<size_t>(index.x()) * static_cast<size_t>(columns) +
               static_cast<size_t>(index.y());
    }

    grid_map::Index Index(size_t id) const {
        return grid_map::Index(
            static_cast<int>(id / static_cast<size_t>(columns)),
            static_cast<int>(id % static_cast<size_t>(columns)));
    }
};

struct CandidateEvaluation {
    size_t original_index = 0;
    Path path;
    std::array<double, kCostCount> raw_costs{};
};

void SetError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = "Shadow planner: " + message;
    }
}

bool FinitePositive(double value) {
    return std::isfinite(value) && value > 0.0;
}

double NormalizeAngle(double angle) {
    return std::remainder(angle, 2.0 * kPi);
}

bool NormalizeQuaternion(const Quaternion& source,
                         Eigen::Quaterniond* quaternion) {
    if (!std::isfinite(source.x()) || !std::isfinite(source.y()) ||
        !std::isfinite(source.z()) || !std::isfinite(source.w())) {
        return false;
    }
    *quaternion =
        Eigen::Quaterniond(source.w(), source.x(), source.y(), source.z());
    const double norm = quaternion->norm();
    if (!FinitePositive(norm)) {
        return false;
    }
    quaternion->coeffs() /= norm;
    return true;
}

void SetQuaternion(const Eigen::Quaterniond& quaternion, Quaternion* output) {
    output->set_x(quaternion.x());
    output->set_y(quaternion.y());
    output->set_z(quaternion.z());
    output->set_w(quaternion.w());
}

double Yaw(const Eigen::Quaterniond& quaternion) {
    return std::atan2(2.0 * (quaternion.w() * quaternion.z() +
                             quaternion.x() * quaternion.y()),
                      1.0 - 2.0 * (quaternion.y() * quaternion.y() +
                                   quaternion.z() * quaternion.z()));
}

bool StampPath(int64_t stamp_ns, const std::string& frame_id, Path* path,
               std::string* error) {
    if (stamp_ns <= 0) {
        SetError(error, "timestamp must be positive.");
        return false;
    }
    const int64_t seconds = stamp_ns / kNanosecondsPerSecond;
    if (seconds > std::numeric_limits<int32_t>::max()) {
        SetError(error, "timestamp exceeds the automsgs Time range.");
        return false;
    }
    path->mutable_header()->set_frame_id(frame_id);
    path->mutable_header()->mutable_stamp()->set_sec(
        static_cast<int32_t>(seconds));
    path->mutable_header()->mutable_stamp()->set_nanosec(
        static_cast<uint32_t>(stamp_ns % kNanosecondsPerSecond));
    return true;
}

std::array<double, kCostCount> PlannerWeights(
    const proto::ShadowOptions& options) {
    return {options.learned_weight(),        options.clearance_weight(),
            options.traversability_weight(), options.curvature_weight(),
            options.progress_weight(),       options.distance_weight(),
            options.visibility_weight()};
}

bool ValidateOptions(const proto::ShadowOptions& options,
                     std::array<double, kCostCount>* weights,
                     std::string* error) {
    if (options.map_frame().empty() || options.base_frame().empty()) {
        SetError(error, "map and base frames must not be empty.");
        return false;
    }
    if (!FinitePositive(options.robot_radius()) ||
        !FinitePositive(options.max_linear_speed()) ||
        !FinitePositive(options.max_angular_speed()) ||
        !FinitePositive(options.follow_distance())) {
        SetError(error, "footprint, motion, and following limits are invalid.");
        return false;
    }
    *weights = PlannerWeights(options);
    double weight_sum = 0.0;
    for (const double weight : *weights) {
        if (!std::isfinite(weight) || weight < 0.0) {
            SetError(error, "planner weights must be finite and non-negative.");
            return false;
        }
        weight_sum += weight;
    }
    if (!FinitePositive(weight_sum)) {
        SetError(error, "at least one planner weight must be positive.");
        return false;
    }
    return true;
}

bool ValidateGrid(const proto::ShadowOptions& options,
                  const grid_map::GridMap& grid, std::string* error) {
    if (grid.getFrameId() != options.map_frame()) {
        SetError(error, "grid must use the configured map frame.");
        return false;
    }
    if (!FinitePositive(grid.getResolution()) ||
        !FinitePositive(grid.getLength().x()) ||
        !FinitePositive(grid.getLength().y()) ||
        !std::isfinite(grid.getPosition().x()) ||
        !std::isfinite(grid.getPosition().y()) || grid.getSize().x() <= 0 ||
        grid.getSize().y() <= 0) {
        SetError(error, "grid geometry is invalid.");
        return false;
    }
    for (const char* layer : {kElevationLayer, kVarianceLayer, kObstacleLayer,
                              kTraversabilityLayer}) {
        if (!grid.exists(layer)) {
            SetError(error,
                     std::string("grid is missing layer '") + layer + "'.");
            return false;
        }
    }
    const size_t rows = static_cast<size_t>(grid.getSize().x());
    const size_t columns = static_cast<size_t>(grid.getSize().y());
    if (columns != 0 && rows > std::numeric_limits<size_t>::max() / columns) {
        SetError(error, "grid dimensions exceed addressable storage.");
        return false;
    }
    return true;
}

bool ValidateRobotAndTarget(const proto::ShadowOptions& options,
                            const Odometry& odometry, const PoseStamped& target,
                            RobotTransform* robot_transform, Pose* robot_pose,
                            std::string* error) {
    if (odometry.header().frame_id() != options.map_frame() ||
        odometry.child_frame_id() != options.base_frame()) {
        SetError(error, "odometry frames are invalid.");
        return false;
    }
    if (target.header().frame_id() != options.map_frame()) {
        SetError(error, "target must use the configured map frame.");
        return false;
    }
    const Pose& source_robot_pose = odometry.pose().pose().pose();
    const auto& robot_position = source_robot_pose.position();
    const auto& target_position = target.pose().position();
    if (!std::isfinite(robot_position.x()) ||
        !std::isfinite(robot_position.y()) ||
        !std::isfinite(robot_position.z()) ||
        !std::isfinite(target_position.x()) ||
        !std::isfinite(target_position.y()) ||
        !std::isfinite(target_position.z()) ||
        !NormalizeQuaternion(source_robot_pose.orientation(),
                             &robot_transform->rotation)) {
        SetError(error, "robot or target pose is non-finite or invalid.");
        return false;
    }
    robot_transform->translation = Eigen::Vector3d(
        robot_position.x(), robot_position.y(), robot_position.z());
    *robot_pose = source_robot_pose;
    SetQuaternion(robot_transform->rotation, robot_pose->mutable_orientation());
    return true;
}

bool CellIsSafe(const grid_map::GridMap& grid, const grid_map::Index& index,
                float* traversability = nullptr) {
    const float elevation = grid.at(kElevationLayer, index);
    const float variance = grid.at(kVarianceLayer, index);
    const float obstacle = grid.at(kObstacleLayer, index);
    const float terrain = grid.at(kTraversabilityLayer, index);
    if (!std::isfinite(elevation) || !std::isfinite(variance) ||
        !std::isfinite(obstacle) || !std::isfinite(terrain) ||
        obstacle >= kObstacleThreshold || terrain >= kImpassableThreshold) {
        return false;
    }
    if (traversability != nullptr) {
        *traversability = terrain;
    }
    return true;
}

bool PositionIsSafe(const grid_map::GridMap& grid,
                    const grid_map::Position& position) {
    grid_map::Index index;
    return grid.getIndex(position, index) && CellIsSafe(grid, index);
}

bool BuildClearanceField(const grid_map::GridMap& grid, ClearanceField* field) {
    field->rows = grid.getSize().x();
    field->columns = grid.getSize().y();
    field->values.assign(
        static_cast<size_t>(field->rows) * static_cast<size_t>(field->columns),
        std::numeric_limits<double>::infinity());
    using QueueEntry = std::pair<double, size_t>;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>,
                        std::greater<QueueEntry>>
        queue;
    const double half_cell_diagonal = grid.getResolution() / std::sqrt(2.0);
    const double minimum_x =
        grid.getPosition().x() - grid.getLength().x() / 2.0;
    const double maximum_x =
        grid.getPosition().x() + grid.getLength().x() / 2.0;
    const double minimum_y =
        grid.getPosition().y() - grid.getLength().y() / 2.0;
    const double maximum_y =
        grid.getPosition().y() + grid.getLength().y() / 2.0;

    for (grid_map::GridMapIterator iterator(grid); !iterator.isPastEnd();
         ++iterator) {
        const grid_map::Index index = *iterator;
        grid_map::Position position;
        if (!grid.getPosition(index, position)) {
            return false;
        }
        const double distance =
            CellIsSafe(grid, index)
                ? std::min({position.x() - minimum_x, maximum_x - position.x(),
                            position.y() - minimum_y, maximum_y - position.y()})
                : -half_cell_diagonal;
        const size_t id = field->Id(index);
        field->values[id] = distance;
        queue.emplace(distance, id);
    }

    const double resolution = grid.getResolution();
    while (!queue.empty()) {
        const double distance = queue.top().first;
        const size_t id = queue.top().second;
        queue.pop();
        if (distance > field->values[id]) {
            continue;
        }
        const grid_map::Index index = field->Index(id);
        grid_map::Position position;
        if (!grid.getPosition(index, position)) {
            return false;
        }
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) {
                    continue;
                }
                grid_map::Index neighbor;
                const grid_map::Position neighbor_position(
                    position.x() + static_cast<double>(dx) * resolution,
                    position.y() + static_cast<double>(dy) * resolution);
                if (!grid.getIndex(neighbor_position, neighbor)) {
                    continue;
                }
                const size_t neighbor_id = field->Id(neighbor);
                const double step = (dx == 0 || dy == 0)
                                        ? resolution
                                        : resolution * std::sqrt(2.0);
                const double candidate_distance = distance + step;
                if (candidate_distance < field->values[neighbor_id]) {
                    field->values[neighbor_id] = candidate_distance;
                    queue.emplace(candidate_distance, neighbor_id);
                }
            }
        }
    }
    return true;
}

double ClearanceAt(const grid_map::GridMap& grid, const ClearanceField& field,
                   const grid_map::Position& position, double robot_radius) {
    grid_map::Index index;
    if (!grid.getIndex(position, index)) {
        return 0.0;
    }
    return std::max(0.0, field.values[field.Id(index)] - robot_radius);
}

bool FootprintIsInside(const grid_map::GridMap& grid,
                       const grid_map::Position& center, double robot_radius) {
    return grid.isInside(center) &&
           grid.isInside(
               grid_map::Position(center.x() + robot_radius, center.y())) &&
           grid.isInside(
               grid_map::Position(center.x() - robot_radius, center.y())) &&
           grid.isInside(
               grid_map::Position(center.x(), center.y() + robot_radius)) &&
           grid.isInside(
               grid_map::Position(center.x(), center.y() - robot_radius));
}

bool EvaluateFootprint(const grid_map::GridMap& grid,
                       const ClearanceField& clearance_field,
                       const grid_map::Position& center, double robot_radius,
                       double* traversability_sum, size_t* cell_count,
                       double* minimum_clearance) {
    if (!FootprintIsInside(grid, center, robot_radius)) {
        return false;
    }
    const double half_cell = grid.getResolution() * 0.5;
    const double search_radius =
        robot_radius + grid.getResolution() / std::sqrt(2.0);
    bool visited_cell = false;
    for (grid_map::CircleIterator iterator(grid, center, search_radius);
         !iterator.isPastEnd(); ++iterator) {
        const grid_map::Index index = *iterator;
        grid_map::Position cell_center;
        if (!grid.getPosition(index, cell_center)) {
            return false;
        }
        const double dx =
            std::max(0.0, std::abs(cell_center.x() - center.x()) - half_cell);
        const double dy =
            std::max(0.0, std::abs(cell_center.y() - center.y()) - half_cell);
        if (std::hypot(dx, dy) > robot_radius + kComparisonTolerance) {
            continue;
        }
        float traversability = 0.0F;
        if (!CellIsSafe(grid, index, &traversability)) {
            return false;
        }
        *traversability_sum += traversability;
        ++(*cell_count);
        visited_cell = true;
    }
    if (!visited_cell) {
        return false;
    }
    *minimum_clearance =
        std::min(*minimum_clearance,
                 ClearanceAt(grid, clearance_field, center, robot_radius));
    return true;
}

bool SampleStepCount(double distance, double maximum_step, size_t* steps) {
    if (!std::isfinite(distance) || distance < 0.0 ||
        !FinitePositive(maximum_step)) {
        return false;
    }
    const double count = std::max(1.0, std::ceil(distance / maximum_step));
    if (!std::isfinite(count) ||
        count > static_cast<double>(std::numeric_limits<size_t>::max() - 1)) {
        return false;
    }
    *steps = static_cast<size_t>(count);
    return true;
}

bool VisibilityBlockage(const grid_map::GridMap& grid,
                        const grid_map::Position& terminal,
                        const grid_map::Position& target, double* blockage) {
    size_t steps = 0;
    if (!SampleStepCount((target - terminal).norm(), grid.getResolution() * 0.5,
                         &steps)) {
        return false;
    }
    size_t blocked = 0;
    for (size_t step = 0; step <= steps; ++step) {
        const double ratio =
            static_cast<double>(step) / static_cast<double>(steps);
        const grid_map::Position sample =
            terminal + ratio * (target - terminal);
        if (!PositionIsSafe(grid, sample)) {
            ++blocked;
        }
    }
    *blockage = static_cast<double>(blocked) / static_cast<double>(steps + 1);
    return true;
}

bool TransformCandidate(const proto::ShadowOptions& options,
                        const RobotTransform& robot_transform,
                        const Pose& robot_pose, const Path& candidate,
                        const Path& stamped_empty_path, Path* map_path,
                        std::vector<grid_map::Position>* positions,
                        std::vector<double>* relative_yaws) {
    if (candidate.header().frame_id() != options.base_frame() ||
        candidate.poses().empty()) {
        return false;
    }
    *map_path = stamped_empty_path;
    auto* initial_pose = map_path->add_poses();
    *initial_pose->mutable_header() = map_path->header();
    *initial_pose->mutable_pose() = robot_pose;
    positions->clear();
    relative_yaws->clear();
    positions->emplace_back(robot_transform.translation.x(),
                            robot_transform.translation.y());
    relative_yaws->push_back(0.0);

    for (const PoseStamped& stamped_pose : candidate.poses()) {
        if (!stamped_pose.header().frame_id().empty() &&
            stamped_pose.header().frame_id() != options.base_frame()) {
            return false;
        }
        const Pose& source_pose = stamped_pose.pose();
        const auto& source_position = source_pose.position();
        if (!std::isfinite(source_position.x()) ||
            !std::isfinite(source_position.y()) ||
            !std::isfinite(source_position.z())) {
            return false;
        }
        Eigen::Quaterniond source_rotation;
        if (!NormalizeQuaternion(source_pose.orientation(), &source_rotation)) {
            return false;
        }
        const Eigen::Vector3d base_position(
            source_position.x(), source_position.y(), source_position.z());
        const Eigen::Vector3d map_position =
            robot_transform.translation +
            robot_transform.rotation * base_position;
        Eigen::Quaterniond map_rotation =
            robot_transform.rotation * source_rotation;
        map_rotation.normalize();

        auto* output_pose = map_path->add_poses();
        *output_pose->mutable_header() = map_path->header();
        output_pose->mutable_pose()->mutable_position()->set_x(
            map_position.x());
        output_pose->mutable_pose()->mutable_position()->set_y(
            map_position.y());
        output_pose->mutable_pose()->mutable_position()->set_z(
            map_position.z());
        SetQuaternion(map_rotation,
                      output_pose->mutable_pose()->mutable_orientation());
        positions->emplace_back(map_position.x(), map_position.y());
        relative_yaws->push_back(Yaw(source_rotation));
    }
    return true;
}

bool EvaluateCandidate(const proto::ShadowOptions& options,
                       const grid_map::GridMap& grid,
                       const ClearanceField& clearance_field,
                       const RobotTransform& robot_transform,
                       const Pose& robot_pose, const PoseStamped& target,
                       const Path& candidate, float learned_score,
                       const Path& stamped_empty_path,
                       CandidateEvaluation* evaluation) {
    if (!std::isfinite(learned_score)) {
        return false;
    }
    std::vector<grid_map::Position> positions;
    std::vector<double> relative_yaws;
    if (!TransformCandidate(options, robot_transform, robot_pose, candidate,
                            stamped_empty_path, &evaluation->path, &positions,
                            &relative_yaws)) {
        return false;
    }

    double traversability_sum = 0.0;
    size_t traversability_samples = 0;
    double minimum_clearance = std::numeric_limits<double>::infinity();
    double previous_curvature = 0.0;
    double curvature_change = 0.0;
    const double maximum_sample_step = grid.getResolution() * 0.5;

    for (size_t segment = 1; segment < positions.size(); ++segment) {
        const double base_dx =
            candidate.poses(static_cast<int>(segment - 1))
                .pose()
                .position()
                .x() -
            (segment == 1 ? 0.0
                          : candidate.poses(static_cast<int>(segment - 2))
                                .pose()
                                .position()
                                .x());
        const double base_dy =
            candidate.poses(static_cast<int>(segment - 1))
                .pose()
                .position()
                .y() -
            (segment == 1 ? 0.0
                          : candidate.poses(static_cast<int>(segment - 2))
                                .pose()
                                .position()
                                .y());
        const double linear_speed = std::hypot(base_dx, base_dy);
        const double yaw_change =
            NormalizeAngle(relative_yaws[segment] - relative_yaws[segment - 1]);
        if (!std::isfinite(linear_speed) || !std::isfinite(yaw_change) ||
            linear_speed > options.max_linear_speed() + kComparisonTolerance ||
            std::abs(yaw_change) >
                options.max_angular_speed() + kComparisonTolerance) {
            return false;
        }
        const double curvature =
            yaw_change / std::max(linear_speed, maximum_sample_step);
        curvature_change += std::abs(curvature - previous_curvature);
        previous_curvature = curvature;

        const double segment_distance =
            (positions[segment] - positions[segment - 1]).norm();
        size_t sample_steps = 0;
        if (!SampleStepCount(segment_distance, maximum_sample_step,
                             &sample_steps)) {
            return false;
        }
        for (size_t step = 0; step <= sample_steps; ++step) {
            const double ratio =
                static_cast<double>(step) / static_cast<double>(sample_steps);
            const grid_map::Position sample =
                positions[segment - 1] +
                ratio * (positions[segment] - positions[segment - 1]);
            if (!EvaluateFootprint(grid, clearance_field, sample,
                                   options.robot_radius(), &traversability_sum,
                                   &traversability_samples,
                                   &minimum_clearance)) {
                return false;
            }
        }
    }
    if (traversability_samples == 0 || !std::isfinite(minimum_clearance)) {
        return false;
    }

    const grid_map::Position robot_position = positions.front();
    const grid_map::Position terminal = positions.back();
    const grid_map::Position target_position(target.pose().position().x(),
                                             target.pose().position().y());
    const double initial_target_distance =
        (target_position - robot_position).norm();
    const double terminal_target_distance = (target_position - terminal).norm();
    double visibility_blockage = 0.0;
    if (!std::isfinite(initial_target_distance) ||
        !std::isfinite(terminal_target_distance) ||
        !VisibilityBlockage(grid, terminal, target_position,
                            &visibility_blockage)) {
        return false;
    }
    evaluation->raw_costs[kLearnedCost] = learned_score;
    evaluation->raw_costs[kClearanceCost] =
        grid.getResolution() / (grid.getResolution() + minimum_clearance);
    evaluation->raw_costs[kTraversabilityCost] =
        traversability_sum / static_cast<double>(traversability_samples);
    evaluation->raw_costs[kCurvatureCost] = curvature_change;
    evaluation->raw_costs[kProgressCost] =
        terminal_target_distance - initial_target_distance;
    evaluation->raw_costs[kDistanceCost] =
        std::abs(terminal_target_distance - options.follow_distance());
    evaluation->raw_costs[kVisibilityCost] = visibility_blockage;
    return std::all_of(evaluation->raw_costs.begin(),
                       evaluation->raw_costs.end(),
                       [](double cost) { return std::isfinite(cost); });
}

bool SelectLowestNormalizedCost(
    const std::array<double, kCostCount>& weights,
    const std::vector<CandidateEvaluation>& evaluations, size_t* selected) {
    std::array<double, kCostCount> minimums;
    std::array<double, kCostCount> maximums;
    minimums.fill(std::numeric_limits<double>::infinity());
    maximums.fill(-std::numeric_limits<double>::infinity());
    for (const CandidateEvaluation& evaluation : evaluations) {
        for (size_t cost = 0; cost < kCostCount; ++cost) {
            minimums[cost] =
                std::min(minimums[cost], evaluation.raw_costs[cost]);
            maximums[cost] =
                std::max(maximums[cost], evaluation.raw_costs[cost]);
        }
    }

    const double weight_sum =
        std::accumulate(weights.begin(), weights.end(), 0.0);
    double best_total = std::numeric_limits<double>::infinity();
    size_t best_original_index = std::numeric_limits<size_t>::max();
    bool found = false;
    for (size_t index = 0; index < evaluations.size(); ++index) {
        double weighted_total = 0.0;
        for (size_t cost = 0; cost < kCostCount; ++cost) {
            const double range = maximums[cost] - minimums[cost];
            const double normalized =
                range > kComparisonTolerance
                    ? (evaluations[index].raw_costs[cost] - minimums[cost]) /
                          range
                    : 0.0;
            weighted_total += weights[cost] * normalized;
        }
        const double total = weighted_total / weight_sum;
        if (!std::isfinite(total) || total < 0.0) {
            continue;
        }
        if (!found || total < best_total ||
            (total == best_total &&
             evaluations[index].original_index < best_original_index)) {
            found = true;
            best_total = total;
            best_original_index = evaluations[index].original_index;
            *selected = index;
        }
    }
    return found;
}

}  // namespace

FollowPlanner::FollowPlanner(const proto::ShadowOptions& options)
    : options_(options) {}

bool FollowPlanner::Select(int64_t stamp_ns,
                           const std::vector<Path>& candidates,
                           const std::vector<float>& learned_scores,
                           const grid_map::GridMap& grid,
                           const Odometry& odometry, const PoseStamped& target,
                           Path* output, std::string* error) const {
    if (error != nullptr) {
        error->clear();
    }
    if (output == nullptr) {
        SetError(error, "output path is null.");
        return false;
    }
    output->Clear();

    std::array<double, kCostCount> weights;
    if (!ValidateOptions(options_, &weights, error) ||
        !StampPath(stamp_ns, options_.map_frame(), output, error)) {
        return false;
    }
    if (candidates.size() != learned_scores.size()) {
        SetError(error, "candidate and learned-score counts differ.");
        return false;
    }
    if (!ValidateGrid(options_, grid, error)) {
        return false;
    }
    RobotTransform robot_transform;
    Pose robot_pose;
    if (!ValidateRobotAndTarget(options_, odometry, target, &robot_transform,
                                &robot_pose, error)) {
        return false;
    }

    ClearanceField clearance_field;
    if (!BuildClearanceField(grid, &clearance_field)) {
        SetError(error, "failed to derive grid clearance.");
        return false;
    }

    const Path stamped_empty_path = *output;
    std::vector<CandidateEvaluation> evaluations;
    evaluations.reserve(candidates.size());
    for (size_t index = 0; index < candidates.size(); ++index) {
        CandidateEvaluation evaluation;
        evaluation.original_index = index;
        if (EvaluateCandidate(options_, grid, clearance_field, robot_transform,
                              robot_pose, target, candidates[index],
                              learned_scores[index], stamped_empty_path,
                              &evaluation)) {
            evaluations.push_back(std::move(evaluation));
        }
    }
    if (evaluations.empty()) {
        return true;
    }

    size_t selected = 0;
    if (!SelectLowestNormalizedCost(weights, evaluations, &selected)) {
        return true;
    }
    *output = std::move(evaluations[selected].path);
    return true;
}

}  // namespace shadow
}  // namespace perception
}  // namespace autonomy
