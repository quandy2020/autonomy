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

#include "autonomy/localization/atlas/frontend/lidar/lidar_odometry.hpp"

#include <cmath>
#include <unordered_set>

#include "Eigen/Eigenvalues"

#include "autonomy/localization/atlas/backend/optimizer.hpp"
#include "autonomy/localization/atlas/io/point_cloud_io.hpp"
#include "autonomy/localization/atlas/map/dense/tiled_cloud.hpp"

#include <fstream>

namespace autonomy {
namespace localization {
namespace atlas {
namespace {

constexpr int kMaxPointsPerVoxel = 20;
constexpr int kMinPlanePoints = 5;

}  // namespace

bool LidarOdometry::Init(const AtlasConfig& config) {
    config_ = config;
    OccupancyMap::Options grid;
    grid.resolution = config.occupancy_resolution;
    grid.min_z = config.occupancy_min_z;
    grid.max_z = config.occupancy_max_z;
    grid.max_range = config.occupancy_max_range;
    occupancy_ = OccupancyMap(grid);
    Reset();
    return true;
}

FrontendMode LidarOdometry::Mode() const { return config_.mode; }

void LidarOdometry::Reset() {
    voxels_.clear();
    map_points_ = 0;
    T_wb_ = SE3Identity();
    has_pose_ = false;
    last_stamp_ = 0.0;
    result_ = OdometryResult();
    pending_.clear();
    next_keyframe_id_ = 0;
    last_keyframe_pose_ = SE3Identity();
    has_keyframe_ = false;
    has_prior_ = false;
    prior_sqrt_info_ = 0.0;
    reloc_done_ = false;
    occupancy_.Clear();
    lidar_keyframes_.clear();
}

void LidarOdometry::SetPosePrior(const SE3& T_wb, double sqrt_info) {
    prior_ = T_wb;
    prior_sqrt_info_ = sqrt_info;
    has_prior_ = true;
}

void LidarOdometry::LoadMap(const PointCloud& cloud) {
    voxels_.clear();
    map_points_ = 0;
    for (const auto& point : cloud) {
        InsertPoint(Vec3(point.x, point.y, point.z));
    }
    has_pose_ = map_points_ > 0;
    reloc_done_ = false;
    occupancy_.Clear();
}

LidarOdometry::VoxelKey LidarOdometry::KeyOf(const Vec3& point) const {
    const double voxel = config_.voxel_size > 1e-3 ? config_.voxel_size : 0.5;
    VoxelKey key;
    key.x = static_cast<int>(std::floor(point.x() / voxel));
    key.y = static_cast<int>(std::floor(point.y() / voxel));
    key.z = static_cast<int>(std::floor(point.z() / voxel));
    return key;
}

void LidarOdometry::InsertPoint(const Vec3& point) {
    if (map_points_ >= static_cast<std::size_t>(config_.max_local_map_points)) {
        return;
    }
    auto& cell = voxels_[KeyOf(point)];
    if (static_cast<int>(cell.size()) >= kMaxPointsPerVoxel) {
        return;
    }
    cell.push_back(point);
    ++map_points_;
}

bool LidarOdometry::FitPlane(const Vec3& query, Vec3* plane_point,
                             Vec3* plane_normal) const {
    std::vector<Vec3> neighbors;
    const VoxelKey center = KeyOf(query);
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                VoxelKey key = center;
                key.x += dx;
                key.y += dy;
                key.z += dz;
                const auto found = voxels_.find(key);
                if (found == voxels_.end()) {
                    continue;
                }
                neighbors.insert(neighbors.end(), found->second.begin(),
                                 found->second.end());
            }
        }
    }
    if (static_cast<int>(neighbors.size()) < kMinPlanePoints) {
        return false;
    }
    Vec3 mean = Vec3::Zero();
    for (const Vec3& point : neighbors) {
        mean += point;
    }
    mean /= static_cast<double>(neighbors.size());
    Mat33 covariance = Mat33::Zero();
    for (const Vec3& point : neighbors) {
        const Vec3 d = point - mean;
        covariance += d * d.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Mat33> solver(covariance);
    if (solver.info() != Eigen::Success) {
        return false;
    }
    const Vec3 eigenvalues = solver.eigenvalues();
    if (eigenvalues(1) < 1e-6 || eigenvalues(0) > 0.2 * eigenvalues(1)) {
        return false;
    }
    *plane_point = mean;
    *plane_normal = solver.eigenvectors().col(0).normalized();
    return true;
}

SE3 LidarOdometry::Predict(const SensorData& data) const {
    SE3 predicted = T_wb_;
    if (!data.has_imu || data.imu.empty() || !has_pose_) {
        return predicted;
    }
    Mat33 rotation = predicted.rotation();
    double previous = last_stamp_;
    for (const auto& sample : data.imu) {
        const double stamp = HeaderStampSec(sample.header());
        const double dt = stamp - previous;
        previous = stamp;
        if (dt <= 0.0 || dt > 0.2) {
            continue;
        }
        const Vec3 gyro(sample.angular_velocity().x(),
                        sample.angular_velocity().y(),
                        sample.angular_velocity().z());
        const double angle = gyro.norm() * dt;
        if (angle < 1e-8) {
            continue;
        }
        rotation = rotation * Eigen::AngleAxisd(angle, gyro.normalized())
                                  .toRotationMatrix();
    }
    predicted.linear() = rotation;
    return predicted;
}

void LidarOdometry::MaybeQueueKeyframe(double stamp, const PointCloud& body) {
    const double translation =
        (T_wb_.translation() - last_keyframe_pose_.translation()).norm();
    if (has_keyframe_ && translation < 0.5) {
        return;
    }
    Keyframe frame;
    frame.id = next_keyframe_id_++;
    frame.timestamp = static_cast<TimeStamp>(stamp * 1e9);
    frame.pose_world_body = T_wb_;
    frame.cloud = body;
    pending_.push_back(frame);
    lidar_keyframes_.push_back(frame);
    if (lidar_keyframes_.size() > 500) {
        lidar_keyframes_.erase(lidar_keyframes_.begin());
    }
    last_keyframe_pose_ = T_wb_;
    has_keyframe_ = true;
}

void LidarOdometry::Deskew(PointCloud* scan, const SensorData& data,
                           double stamp) const {
    if (scan == nullptr || !data.has_imu || data.imu.empty()) {
        return;
    }
    Vec3 gyro = Vec3::Zero();
    int count = 0;
    for (const auto& sample : data.imu) {
        gyro += Vec3(sample.angular_velocity().x(), sample.angular_velocity().y(),
                     sample.angular_velocity().z());
        ++count;
    }
    if (count == 0) {
        return;
    }
    gyro /= static_cast<double>(count);
    const double rate = gyro.norm();
    if (rate < 1e-4) {
        return;
    }
    const Vec3 axis = gyro / rate;
    bool has_span = false;
    for (const auto& point : *scan) {
        if (std::fabs(point.timestamp - stamp) > 1e-4) {
            has_span = true;
            break;
        }
    }
    if (!has_span) {
        return;
    }
    for (auto& point : *scan) {
        const double dt = point.timestamp - stamp;
        const double angle = -rate * dt;
        if (std::fabs(angle) < 1e-6) {
            continue;
        }
        const Vec3 rotated =
            Eigen::AngleAxisd(angle, axis) * Vec3(point.x, point.y, point.z);
        point.x = static_cast<float>(rotated.x());
        point.y = static_cast<float>(rotated.y());
        point.z = static_cast<float>(rotated.z());
    }
}

int LidarOdometry::ScorePose(const PointCloud& scan, const SE3& pose) const {
    int score = 0;
    std::unordered_set<VoxelKey, VoxelHash> used;
    for (const auto& point : scan) {
        const Vec3 world = pose * Vec3(point.x, point.y, point.z);
        if (!used.insert(KeyOf(world)).second) {
            continue;
        }
        Vec3 plane_point;
        Vec3 plane_normal;
        if (!FitPlane(world, &plane_point, &plane_normal)) {
            continue;
        }
        if (std::fabs(plane_normal.dot(world - plane_point)) < 0.3) {
            ++score;
        }
    }
    return score;
}

bool LidarOdometry::SearchYaw(const PointCloud& scan, SE3* pose) const {
    if (pose == nullptr || map_points_ == 0) {
        return false;
    }
    std::vector<Vec3> translations;
    if (has_prior_) {
        translations.push_back(prior_.translation());
    } else {
        const double radius =
            config_.reloc_xy_radius > 0.0 ? config_.reloc_xy_radius : 0.0;
        const double step =
            config_.reloc_xy_step > 0.1 ? config_.reloc_xy_step : radius;
        if (radius < 1e-3 || step < 1e-3) {
            translations.push_back(Vec3::Zero());
        } else {
            for (double x = -radius; x <= radius + 1e-6; x += step) {
                for (double y = -radius; y <= radius + 1e-6; y += step) {
                    translations.emplace_back(x, y, 0.0);
                }
            }
        }
    }
    int best_score = -1;
    SE3 best = SE3Identity();
    for (const Vec3& translation : translations) {
        for (int step = 0; step < 24; ++step) {
            const double yaw = step * (2.0 * 3.14159265358979323846 / 24.0);
            SE3 candidate = SE3Identity();
            candidate.linear() =
                Eigen::AngleAxisd(yaw, Vec3::UnitZ()).toRotationMatrix();
            candidate.translation() = translation;
            const int score = ScorePose(scan, candidate);
            if (score > best_score) {
                best_score = score;
                best = candidate;
            }
        }
    }
    if (best_score < 8) {
        return false;
    }
    *pose = best;
    return true;
}

bool LidarOdometry::SaveMap(const std::string& directory) const {
    PointCloud cloud;
    for (const auto& cell : voxels_) {
        for (const Vec3& point : cell.second) {
            PointXYZI sample;
            sample.x = static_cast<float>(point.x());
            sample.y = static_cast<float>(point.y());
            sample.z = static_cast<float>(point.z());
            cloud.push_back(sample);
        }
    }
    if (!SaveTiledCloud(directory, cloud, config_.lidar_chunk_size)) {
        return false;
    }
    automsgs::msgs::map_msgs::OccupancyGrid grid;
    if (!occupancy_.Fill(&grid, "map")) {
        return true;
    }
    std::ofstream out(directory + "/occupancy.pb", std::ios::binary);
    return static_cast<bool>(out) && grid.SerializeToOstream(&out);
}

bool LidarOdometry::LoadMapDirectory(const std::string& directory) {
    PointCloud cloud;
    if (!LoadTiledCloud(directory, &cloud)) {
        return false;
    }
    voxels_.clear();
    map_points_ = 0;
    for (const auto& point : cloud) {
        InsertPoint(Vec3(point.x, point.y, point.z));
    }
    has_pose_ = map_points_ > 0;
    reloc_done_ = false;
    occupancy_.Clear();
    std::ifstream in(directory + "/occupancy.pb", std::ios::binary);
    automsgs::msgs::map_msgs::OccupancyGrid grid;
    if (in && grid.ParseFromIstream(&in)) {
        occupancy_.Load(grid);
    }
    return has_pose_;
}

bool LidarOdometry::FillDenseCloud(PointCloud2* cloud) const {
    if (cloud == nullptr || map_points_ == 0) {
        return false;
    }
    PointCloud points;
    points.reserve(map_points_);
    for (const auto& cell : voxels_) {
        for (const Vec3& point : cell.second) {
            PointXYZI sample;
            sample.x = static_cast<float>(point.x());
            sample.y = static_cast<float>(point.y());
            sample.z = static_cast<float>(point.z());
            points.push_back(sample);
        }
    }
    EncodePointCloud2(points, cloud, "map");
    return true;
}

bool LidarOdometry::FillOccupancyGrid(
    automsgs::msgs::map_msgs::OccupancyGrid* grid) const {
    return occupancy_.Fill(grid, "map");
}

void LidarOdometry::RebuildOccupancy() {
    occupancy_.Clear();
    for (const auto& frame : lidar_keyframes_) {
        occupancy_.Integrate(frame.pose_world_body, frame.cloud);
    }
}

bool LidarOdometry::Process(const SensorData& data) {
    if (!data.has_lidar) {
        return false;
    }
    PointCloud scan;
    if (!DecodePointCloud2(data.lidar, &scan)) {
        return false;
    }
    FilterScan(&scan, config_.lidar_blind, config_.lidar_point_stride,
               config_.lidar_height_min, config_.lidar_height_max);
    if (scan.empty()) {
        return false;
    }
    const double stamp = HeaderStampSec(data.lidar.header());
    Deskew(&scan, data, stamp);
    const bool extend_map = config_.mission == Mission::kMapping;
    const bool need_reloc =
        config_.mission == Mission::kRelocalization && !reloc_done_;

    if (need_reloc) {
        if (map_points_ == 0 || !SearchYaw(scan, &T_wb_)) {
            result_.valid = false;
            return false;
        }
        has_pose_ = true;
    }

    if (!has_pose_) {
        if (!extend_map && map_points_ == 0) {
            return false;
        }
        if (has_prior_) {
            T_wb_ = prior_;
        }
        if (extend_map && map_points_ == 0) {
            for (const auto& point : scan) {
                InsertPoint(T_wb_ * Vec3(point.x, point.y, point.z));
            }
        }
        has_pose_ = true;
        last_stamp_ = stamp;
        result_.timestamp = static_cast<TimeStamp>(stamp * 1e9);
        result_.pose_world_body = T_wb_;
        result_.valid = true;
        occupancy_.Integrate(T_wb_, scan);
        MaybeQueueKeyframe(stamp, scan);
        has_prior_ = false;
        return true;
    }

    SE3 pose = Predict(data);
    std::vector<backend::LidarPlaneFactor> factors;
    std::unordered_set<VoxelKey, VoxelHash> used_voxels;
    factors.reserve(scan.size());
    for (const auto& point : scan) {
        const Vec3 body(point.x, point.y, point.z);
        const Vec3 world = pose * body;
        const VoxelKey key = KeyOf(world);
        if (!used_voxels.insert(key).second) {
            continue;
        }
        Vec3 plane_point;
        Vec3 plane_normal;
        if (!FitPlane(world, &plane_point, &plane_normal)) {
            continue;
        }
        backend::LidarPlaneFactor factor;
        factor.point_body = body;
        factor.plane_point = plane_point;
        factor.plane_normal = plane_normal;
        factor.sqrt_info = config_.ceres_pose_weight;
        factors.push_back(factor);
    }

    const SE3* prior = has_prior_ ? &prior_ : nullptr;
    const int inliers = backend::Optimizer::OptimizeLidarPose(
        &pose, factors, prior, prior_sqrt_info_, config_.ceres_max_iterations);
    has_prior_ = false;
    if (inliers < 5) {
        result_.valid = false;
        return false;
    }

    T_wb_ = pose;
    last_stamp_ = stamp;
    if (extend_map) {
        for (const auto& point : scan) {
            InsertPoint(T_wb_ * Vec3(point.x, point.y, point.z));
        }
    }
    result_.timestamp = static_cast<TimeStamp>(stamp * 1e9);
    result_.pose_world_body = T_wb_;
    result_.valid = true;
    if (need_reloc) {
        reloc_done_ = true;
    }
    occupancy_.Integrate(T_wb_, scan);
    MaybeQueueKeyframe(stamp, scan);
    return true;
}

bool LidarOdometry::GetResult(OdometryResult* out) const {
    if (out == nullptr || !result_.valid) {
        return false;
    }
    *out = result_;
    return true;
}

bool LidarOdometry::ConsumePendingKeyframe(Keyframe* out) {
    if (out == nullptr || pending_.empty()) {
        return false;
    }
    *out = pending_.front();
    pending_.pop_front();
    return true;
}

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
