/*
 * Copyright 2016 The Cartographer Authors
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

#pragma once

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "autolink/autolink.hpp"
#include "autolink/timer/timer.hpp"
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>
#include <automsgs/msgs/sensor_msgs/imu.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud.pb.h>
#include <automsgs/msgs/sensor_msgs/multi_echo_laser_scan.pb.h>
#include <automsgs/msgs/sensor_msgs/nav_sat_fix.pb.h>
#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>
#include "autonomy/localization/cartographer/common/fixed_ratio_sampler.hpp"
#include "autonomy/localization/cartographer/common/port.hpp"
#include "autonomy/localization/cartographer/mapping/map_builder_interface.hpp"
#include "autonomy/localization/cartographer/mapping/pose_extrapolator.hpp"
#include "autonomy/localization/cartographer/io/submap_painter.hpp"
#include "autonomy/localization/cartographer/node/map_builder_bridge.hpp"
#include "autonomy/localization/cartographer/node/node_constants.hpp"
#include "autonomy/localization/cartographer/node/node_options.hpp"
#include "autonomy/localization/cartographer/node/trajectory_options.hpp"
#include "autonomy/localization/cartographer/proto/cartographer_services.pb.h"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/transform_broadcaster.hpp"

namespace autonomy {
namespace localization {
namespace cartographer {
namespace node {

class CartographerNode {
public:
    CartographerNode(
        const NodeOptions& node_options,
        std::unique_ptr<::cartographer::mapping::MapBuilderInterface> map_builder);

    ~CartographerNode();

    CartographerNode(const CartographerNode&) = delete;
    CartographerNode& operator=(const CartographerNode&) = delete;

    bool Init(std::shared_ptr<autolink::Node> node);

    void FinishAllTrajectories();
    bool FinishTrajectory(int trajectory_id);
    void RunFinalOptimization();
    void StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options);
    void SerializeState(const std::string& filename,
                        bool include_unfinished_submaps);
    void LoadState(const std::string& state_filename, bool load_frozen_state);

private:
    struct TrajectorySensorSamplers {
        TrajectorySensorSamplers(double rangefinder_sampling_ratio,
                                 double odometry_sampling_ratio,
                                 double fixed_frame_pose_sampling_ratio,
                                 double imu_sampling_ratio,
                                 double landmark_sampling_ratio)
            : rangefinder_sampler(rangefinder_sampling_ratio),
              odometry_sampler(odometry_sampling_ratio),
              fixed_frame_pose_sampler(fixed_frame_pose_sampling_ratio),
              imu_sampler(imu_sampling_ratio),
              landmark_sampler(landmark_sampling_ratio) {}

        ::cartographer::common::FixedRatioSampler rangefinder_sampler;
        ::cartographer::common::FixedRatioSampler odometry_sampler;
        ::cartographer::common::FixedRatioSampler fixed_frame_pose_sampler;
        ::cartographer::common::FixedRatioSampler imu_sampler;
        ::cartographer::common::FixedRatioSampler landmark_sampler;
    };

    std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>
    ComputeExpectedSensorIds(const TrajectoryOptions& options) const;
    int AddTrajectory(const TrajectoryOptions& options);
    void LaunchSubscribers(const TrajectoryOptions& options, int trajectory_id);
    void AddExtrapolator(int trajectory_id, const TrajectoryOptions& options);
    void AddSensorSamplers(int trajectory_id, const TrajectoryOptions& options);

    void PublishSubmapList();
    void PublishLocalTrajectoryData();
    void PublishOccupancyGrid();

    /** Updates occupancy_submap_slices_ (version-cached) and returns it. */
    const std::map<::cartographer::mapping::SubmapId,
                   ::cartographer::io::SubmapSlice>&
    CollectSubmapSlices();

    void HandleOdometryMessage(int trajectory_id, const std::string& sensor_id,
                               const automsgs::msgs::nav_msgs::Odometry& msg);
    void HandleImuMessage(int trajectory_id, const std::string& sensor_id,
                          const automsgs::msgs::sensor_msgs::Imu& msg);
    void HandleLaserScanMessage(int trajectory_id, const std::string& sensor_id,
                                const automsgs::msgs::sensor_msgs::LaserScan& msg);
    void HandleMultiEchoLaserScanMessage(
        int trajectory_id, const std::string& sensor_id,
        const automsgs::msgs::sensor_msgs::MultiEchoLaserScan& msg);
    void HandlePointCloud2Message(int trajectory_id, const std::string& sensor_id,
                                  const automsgs::msgs::sensor_msgs::PointCloud2& msg);
    void HandleNavSatFixMessage(int trajectory_id, const std::string& sensor_id,
                                const automsgs::msgs::sensor_msgs::NavSatFix& msg);
    void HandleLandmarkMessage(int trajectory_id, const std::string& sensor_id,
                               const proto::LandmarkList& msg);

    void HandleSubmapQuery(
        const std::shared_ptr<proto::SubmapQueryRequest>& request,
        std::shared_ptr<proto::SubmapQueryResponse>& response);
    void HandleStartTrajectory(
        const std::shared_ptr<proto::StartTrajectoryRequest>& request,
        std::shared_ptr<proto::StartTrajectoryResponse>& response);
    void HandleFinishTrajectory(
        const std::shared_ptr<proto::FinishTrajectoryRequest>& request,
        std::shared_ptr<proto::FinishTrajectoryResponse>& response);
    void HandleWriteState(
        const std::shared_ptr<proto::WriteStateRequest>& request,
        std::shared_ptr<proto::WriteStateResponse>& response);

    void DispatchSensorMessage(int trajectory_id,
                               ::cartographer::common::Time sensor_time,
                               std::function<void()> handler);

    const NodeOptions node_options_;
    transform::Buffer* tf_buffer_;
    std::shared_ptr<transform::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<autolink::Node> node_;

    std::mutex mutex_;
    std::unique_ptr<MapBuilderBridge> map_builder_bridge_ GUARDED_BY(mutex_);

    std::shared_ptr<autolink::Writer<proto::SubmapList>> submap_list_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::geometry_msgs::PoseStamped>>
        tracked_pose_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::map_msgs::OccupancyGrid>>
        occupancy_grid_writer_;
    /** Cached painted submap textures; skip HandleSubmapQuery when version
     *  unchanged (OccupancyGridNode already does this; embedded path did not). */
    std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>
        occupancy_submap_slices_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::tf2_msgs::TFMessage>>
        tf_writer_;

    std::vector<std::unique_ptr<autolink::Timer>> timers_;

    std::map<int, ::cartographer::mapping::PoseExtrapolator> extrapolators_;
    std::map<int, automsgs::msgs::builtin_interfaces::Time> last_published_tf_stamps_;
    std::unordered_map<int, TrajectorySensorSamplers> sensor_samplers_;
    std::map<int, std::multimap<std::pair<::cartographer::common::Time, uint64_t>,
                                std::function<void()>>>
        sensor_dispatch_queues_;
    std::map<int, uint64_t> sensor_dispatch_sequence_;
};

}  // namespace node
}  // namespace cartographer
}  // namespace localization
}  // namespace autonomy
