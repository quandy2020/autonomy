/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

/**
 * @file buffer.hpp
 * @brief Thread-safe latest-frame observation cache.
 */

#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include "autonomy/control/controller/policy_controller/types.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {

/**
 * @class ObservationBuffer
 * @brief Latest-value store for ego state and optional sensor modalities.
 *
 * Heavy messages are kept as @c shared_ptr so @ref Snapshot is an O(1)
 * pointer copy under the mutex.
 */
class ObservationBuffer
{
public:
    void SetPose(const automsgs::msgs::geometry_msgs::PoseStamped& pose);
    void SetVelocity(
        const automsgs::msgs::geometry_msgs::TwistStamped& velocity);

    /**
     * @brief Write pose and velocity under a single lock (control-tick path).
     */
    void SetEgoState(
        const automsgs::msgs::geometry_msgs::PoseStamped& pose,
        const automsgs::msgs::geometry_msgs::TwistStamped& velocity);

    void SetPath(const automsgs::msgs::nav_msgs::Path& path);
    void SetLaserScan(const automsgs::msgs::sensor_msgs::LaserScan& laser_scan);
    void SetImage(const automsgs::msgs::sensor_msgs::Image& image);
    void SetPointCloud(
        const automsgs::msgs::sensor_msgs::PointCloud2& point_cloud);

    /**
     * @brief Inject a precomputed lidar BEV float grid.
     * @note Heavy BEV projection belongs in the exported ONNX graph.
     */
    void SetLidarBev(const TensorFrame& lidar_bev);

    /**
     * @brief Inject a named RAW float tensor (key = frame.name).
     */
    void SetRawTensor(const TensorFrame& raw);

    /**
     * @brief Crop a local costmap patch around the robot into @ref CostmapCrop.
     */
    void UpdateCostmapFrom(map::costmap_2d::Costmap2DWrapper* costmap_wrapper,
                           int crop_cells);

    ObservationSnapshot Snapshot() const;

    bool GetPose(automsgs::msgs::geometry_msgs::PoseStamped* pose) const;

    void Clear();

private:
    void MarkModality(const std::string& name);

    mutable std::mutex mutex_;
    std::optional<automsgs::msgs::geometry_msgs::PoseStamped> pose_;
    std::optional<automsgs::msgs::geometry_msgs::TwistStamped> velocity_;
    std::shared_ptr<const automsgs::msgs::nav_msgs::Path> path_;
    std::shared_ptr<const automsgs::msgs::sensor_msgs::LaserScan> laser_scan_;
    std::shared_ptr<const automsgs::msgs::sensor_msgs::Image> image_;
    std::shared_ptr<const automsgs::msgs::sensor_msgs::PointCloud2> point_cloud_;
    std::shared_ptr<const TensorFrame> lidar_bev_;
    std::unordered_map<std::string, std::shared_ptr<const TensorFrame>>
        raw_tensors_;
    std::shared_ptr<const CostmapCrop> costmap_;
    std::unordered_map<std::string, double> modality_last_wall_sec_;
};

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
