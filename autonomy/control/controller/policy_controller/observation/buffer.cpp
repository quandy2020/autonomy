/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/control/controller/policy_controller/observation/buffer.hpp"

#include <algorithm>
#include <chrono>

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {
namespace {

double SteadyNowSec() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch())
        .count();
}

}  // namespace

void ObservationBuffer::MarkModality(const std::string& name) {
    modality_last_wall_sec_[name] = SteadyNowSec();
}

void ObservationBuffer::SetPose(
    const automsgs::msgs::geometry_msgs::PoseStamped& pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    pose_ = pose;
    MarkModality("observation_kinematic");
}

void ObservationBuffer::SetVelocity(
    const automsgs::msgs::geometry_msgs::TwistStamped& velocity) {
    std::lock_guard<std::mutex> lock(mutex_);
    velocity_ = velocity;
    MarkModality("observation_kinematic");
}

void ObservationBuffer::SetEgoState(
    const automsgs::msgs::geometry_msgs::PoseStamped& pose,
    const automsgs::msgs::geometry_msgs::TwistStamped& velocity) {
    std::lock_guard<std::mutex> lock(mutex_);
    pose_ = pose;
    velocity_ = velocity;
    MarkModality("observation_kinematic");
}

void ObservationBuffer::SetPath(const automsgs::msgs::nav_msgs::Path& path) {
    auto frame = std::make_shared<automsgs::msgs::nav_msgs::Path>(path);
    std::lock_guard<std::mutex> lock(mutex_);
    path_ = std::move(frame);
    MarkModality("observation_path");
}

void ObservationBuffer::SetLaserScan(
    const automsgs::msgs::sensor_msgs::LaserScan& laser_scan) {
    auto frame =
        std::make_shared<automsgs::msgs::sensor_msgs::LaserScan>(laser_scan);
    std::lock_guard<std::mutex> lock(mutex_);
    laser_scan_ = std::move(frame);
    MarkModality("observation_laser");
}

void ObservationBuffer::SetImage(
    const automsgs::msgs::sensor_msgs::Image& image) {
    auto frame = std::make_shared<automsgs::msgs::sensor_msgs::Image>(image);
    std::lock_guard<std::mutex> lock(mutex_);
    image_ = std::move(frame);
    MarkModality("observation_image");
}

void ObservationBuffer::SetPointCloud(
    const automsgs::msgs::sensor_msgs::PointCloud2& point_cloud) {
    auto frame =
        std::make_shared<automsgs::msgs::sensor_msgs::PointCloud2>(point_cloud);
    std::lock_guard<std::mutex> lock(mutex_);
    point_cloud_ = std::move(frame);
    MarkModality("observation_point_cloud");
}

void ObservationBuffer::SetLidarBev(const TensorFrame& lidar_bev) {
    auto frame = std::make_shared<TensorFrame>(lidar_bev);
    std::lock_guard<std::mutex> lock(mutex_);
    lidar_bev_ = std::move(frame);
    MarkModality(lidar_bev.name.empty() ? "observation_lidar_bev"
                                        : lidar_bev.name);
}

void ObservationBuffer::SetRawTensor(const TensorFrame& raw) {
    if (raw.name.empty()) {
        return;
    }
    auto frame = std::make_shared<TensorFrame>(raw);
    std::lock_guard<std::mutex> lock(mutex_);
    raw_tensors_[raw.name] = std::move(frame);
    MarkModality(raw.name);
}

void ObservationBuffer::UpdateCostmapFrom(
    map::costmap_2d::Costmap2DWrapper* costmap_wrapper, int crop_cells) {
    if (costmap_wrapper == nullptr || costmap_wrapper->getCostmap() == nullptr) {
        return;
    }
    auto* costmap = costmap_wrapper->getCostmap();
    std::lock_guard<::autonomy::map::costmap_2d::Costmap2D::mutex_t> map_lock(*(costmap->getMutex()));
    std::lock_guard<std::mutex> lock(mutex_);

    const unsigned int size_x = costmap->getSizeInCellsX();
    const unsigned int size_y = costmap->getSizeInCellsY();
    if (size_x == 0 || size_y == 0) {
        costmap_.reset();
        return;
    }

    unsigned int map_x = size_x / 2;
    unsigned int map_y = size_y / 2;
    if (pose_.has_value()) {
        costmap->worldToMap(pose_->pose().position().x(),
                            pose_->pose().position().y(), map_x, map_y);
    }

    int edge = crop_cells > 0
                   ? crop_cells
                   : static_cast<int>(std::min(size_x, size_y));
    edge = std::max(8, std::min(edge, static_cast<int>(std::min(size_x, size_y))));
    const int half = edge / 2;
    const int x0 = std::max(0, static_cast<int>(map_x) - half);
    const int y0 = std::max(0, static_cast<int>(map_y) - half);
    const int x1 = std::min(static_cast<int>(size_x), x0 + edge);
    const int y1 = std::min(static_cast<int>(size_y), y0 + edge);
    const int width = x1 - x0;
    const int height = y1 - y0;

    auto crop = std::make_shared<CostmapCrop>();
    crop->width = width;
    crop->height = height;
    crop->resolution = static_cast<float>(costmap->getResolution());
    crop->data.resize(static_cast<std::size_t>(width * height));
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const unsigned char cost = costmap->getCost(
                static_cast<unsigned int>(x0 + x),
                static_cast<unsigned int>(y0 + y));
            crop->data[static_cast<std::size_t>(y * width + x)] =
                static_cast<float>(cost) / 255.0f;
        }
    }
    costmap_ = std::move(crop);
    MarkModality("observation_costmap");
}

ObservationSnapshot ObservationBuffer::Snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    ObservationSnapshot snapshot;
    snapshot.pose = pose_;
    snapshot.velocity = velocity_;
    snapshot.path = path_;
    snapshot.laser_scan = laser_scan_;
    snapshot.image = image_;
    snapshot.point_cloud = point_cloud_;
    snapshot.lidar_bev = lidar_bev_;
    snapshot.raw_tensors = raw_tensors_;
    snapshot.costmap = costmap_;
    snapshot.modality_last_wall_sec = modality_last_wall_sec_;
    if (pose_.has_value()) {
        snapshot.stamp_sec = HeaderStampSeconds(pose_->header());
    } else if (laser_scan_ != nullptr) {
        snapshot.stamp_sec = HeaderStampSeconds(laser_scan_->header());
    } else if (point_cloud_ != nullptr) {
        snapshot.stamp_sec = HeaderStampSeconds(point_cloud_->header());
    }
    return snapshot;
}

bool ObservationBuffer::GetPose(
    automsgs::msgs::geometry_msgs::PoseStamped* pose) const {
    if (pose == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pose_.has_value()) {
        return false;
    }
    *pose = *pose_;
    return true;
}

void ObservationBuffer::Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    pose_.reset();
    velocity_.reset();
    path_.reset();
    laser_scan_.reset();
    image_.reset();
    point_cloud_.reset();
    lidar_bev_.reset();
    raw_tensors_.clear();
    costmap_.reset();
    modality_last_wall_sec_.clear();
}

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
