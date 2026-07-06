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

#include "autonomy/localization/cartographer/node/cartographer_node.hpp"

#include <algorithm>
#include <utility>

#include <glog/logging.h>

#include "autonomy/commsgs/proto/geometry_msgs.pb.h"
#include "autonomy/localization/cartographer/io/image.hpp"
#include "autonomy/localization/cartographer/io/submap_painter.hpp"
#include "autonomy/localization/cartographer/mapping/id.hpp"
#include "autonomy/localization/cartographer/node/msg_conversion.hpp"
#include "autonomy/localization/cartographer/node/time_conversion.hpp"
#include "autonomy/transform/geometry_msgs/transform_stamped.h"

namespace autonomy {
namespace localization {
namespace cartographer {
namespace node {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;
using TrajectoryState =
    carto::mapping::PoseGraphInterface::TrajectoryState;

namespace {

uint32_t MsFromSeconds(const double seconds) {
    return static_cast<uint32_t>(std::max(seconds * 1000.0, 1.0));
}

void SetTransformInBuffer(
    transform::Buffer* buffer,
    const commsgs::geometry_msgs::TransformStamped& trans,
    const std::string& authority, bool is_static) {
    geometry_msgs::TransformStamped geo_msg;
    geo_msg.header.stamp =
        static_cast<uint64_t>(trans.header.stamp.sec) * 1000000000ULL +
        static_cast<uint64_t>(trans.header.stamp.nanosec);
    geo_msg.header.frame_id = trans.header.frame_id;
    geo_msg.child_frame_id = trans.child_frame_id;
    geo_msg.transform.translation.x = trans.transform.translation.x;
    geo_msg.transform.translation.y = trans.transform.translation.y;
    geo_msg.transform.translation.z = trans.transform.translation.z;
    geo_msg.transform.rotation.x = trans.transform.rotation.x;
    geo_msg.transform.rotation.y = trans.transform.rotation.y;
    geo_msg.transform.rotation.z = trans.transform.rotation.z;
    geo_msg.transform.rotation.w = trans.transform.rotation.w;
    buffer->setTransform(geo_msg, authority, is_static);
}

}  // namespace

CartographerNode::CartographerNode(
    const NodeOptions& node_options,
    std::unique_ptr<::cartographer::mapping::MapBuilderInterface> map_builder)
    : node_options_(node_options),
      tf_buffer_(transform::Buffer::Instance()),
      tf_broadcaster_(std::make_shared<transform::TransformBroadcaster>()) {
    map_builder_bridge_ = std::make_unique<MapBuilderBridge>(
        node_options_, std::move(map_builder), tf_buffer_);
}

CartographerNode::~CartographerNode() { FinishAllTrajectories(); }

bool CartographerNode::Init(std::shared_ptr<autolink::Node> node) {
    node_ = std::move(node);
    if (!node_) {
        LOG(ERROR) << "CartographerNode requires a valid autolink node.";
        return false;
    }

    submap_list_writer_ =
        node_->CreateWriter<proto::SubmapList>(kSubmapListTopic);
    if (node_options_.publish_tracked_pose) {
        tracked_pose_writer_ =
            node_->CreateWriter<commsgs::geometry_msgs::PoseStamped>(
                kTrackedPoseTopic);
    }
    if (node_options_.publish_occupancy_grid) {
        occupancy_grid_writer_ =
            node_->CreateWriter<commsgs::map_msgs::OccupancyGrid>(
                kOccupancyGridTopic);
    }

    node_->CreateService<proto::SubmapQueryRequest, proto::SubmapQueryResponse>(
        kSubmapQueryServiceName,
        [this](const std::shared_ptr<proto::SubmapQueryRequest>& request,
               std::shared_ptr<proto::SubmapQueryResponse>& response) {
            HandleSubmapQuery(request, response);
        });
    node_->CreateService<proto::StartTrajectoryRequest,
                           proto::StartTrajectoryResponse>(
        kStartTrajectoryServiceName,
        [this](const std::shared_ptr<proto::StartTrajectoryRequest>& request,
               std::shared_ptr<proto::StartTrajectoryResponse>& response) {
            HandleStartTrajectory(request, response);
        });
    node_->CreateService<proto::FinishTrajectoryRequest,
                           proto::FinishTrajectoryResponse>(
        kFinishTrajectoryServiceName,
        [this](const std::shared_ptr<proto::FinishTrajectoryRequest>& request,
               std::shared_ptr<proto::FinishTrajectoryResponse>& response) {
            HandleFinishTrajectory(request, response);
        });
    node_->CreateService<proto::WriteStateRequest, proto::WriteStateResponse>(
        kWriteStateServiceName,
        [this](const std::shared_ptr<proto::WriteStateRequest>& request,
               std::shared_ptr<proto::WriteStateResponse>& response) {
            HandleWriteState(request, response);
        });

    node_->CreateReader<commsgs::geometry_msgs::TransformStampeds>(
        kTfTopic,
        [this](const std::shared_ptr<commsgs::geometry_msgs::TransformStampeds>&
                   msg) {
            if (!msg || !tf_buffer_) {
                return;
            }
            for (const auto& transform : msg->transforms) {
                try {
                    SetTransformInBuffer(tf_buffer_, transform,
                                         "cartographer_node", false);
                } catch (const std::exception& ex) {
                    LOG(WARNING) << "Failed to set transform: " << ex.what();
                }
            }
        });

    timers_.emplace_back(std::make_unique<autolink::Timer>(
        MsFromSeconds(node_options_.submap_publish_period_sec),
        [this]() { PublishSubmapList(); }, false));
    if (node_options_.pose_publish_period_sec > 0) {
        timers_.emplace_back(std::make_unique<autolink::Timer>(
            MsFromSeconds(node_options_.pose_publish_period_sec),
            [this]() { PublishLocalTrajectoryData(); }, false));
    }
    if (node_options_.publish_occupancy_grid) {
        timers_.emplace_back(std::make_unique<autolink::Timer>(
            MsFromSeconds(node_options_.occupancy_grid_publish_period_sec),
            [this]() { PublishOccupancyGrid(); }, false));
    }
    for (auto& timer : timers_) {
        timer->Start();
    }
    return true;
}

void CartographerNode::FinishAllTrajectories() {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto trajectory_states = map_builder_bridge_->GetTrajectoryStates();
    for (const auto& entry : trajectory_states) {
        if (entry.second == TrajectoryState::ACTIVE) {
            const int trajectory_id = entry.first;
            LOG(INFO) << "Finishing trajectory with ID '" << trajectory_id
                      << "'...";
            map_builder_bridge_->FinishTrajectory(trajectory_id);
        }
    }
}

bool CartographerNode::FinishTrajectory(const int trajectory_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto trajectory_states = map_builder_bridge_->GetTrajectoryStates();
    if (trajectory_states.count(trajectory_id) == 0 ||
        trajectory_states.at(trajectory_id) != TrajectoryState::ACTIVE) {
        return false;
    }
    map_builder_bridge_->FinishTrajectory(trajectory_id);
    return true;
}

void CartographerNode::RunFinalOptimization() {
    std::lock_guard<std::mutex> lock(mutex_);
    map_builder_bridge_->RunFinalOptimization();
}

void CartographerNode::StartTrajectoryWithDefaultTopics(
    const TrajectoryOptions& options) {
    AddTrajectory(options);
}

void CartographerNode::SerializeState(const std::string& filename,
                                       const bool include_unfinished_submaps) {
    std::lock_guard<std::mutex> lock(mutex_);
    map_builder_bridge_->SerializeState(filename, include_unfinished_submaps);
}

void CartographerNode::LoadState(const std::string& state_filename,
                                 bool load_frozen_state) {
    std::lock_guard<std::mutex> lock(mutex_);
    map_builder_bridge_->LoadState(state_filename, load_frozen_state);
}

std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>
CartographerNode::ComputeExpectedSensorIds(
    const TrajectoryOptions& options) const {
    using SensorId =
        ::cartographer::mapping::TrajectoryBuilderInterface::SensorId;
    using SensorType = SensorId::SensorType;
    std::set<SensorId> expected_topics;
    for (const std::string& topic :
         ComputeRepeatedTopicNames(kLaserScanTopic, options.num_laser_scans)) {
        expected_topics.insert(SensorId{SensorType::RANGE, topic});
    }
    for (const std::string& topic : ComputeRepeatedTopicNames(
             kMultiEchoLaserScanTopic, options.num_multi_echo_laser_scans)) {
        expected_topics.insert(SensorId{SensorType::RANGE, topic});
    }
    for (const std::string& topic : ComputeRepeatedTopicNames(
             kPointCloud2Topic, options.num_point_clouds)) {
        expected_topics.insert(SensorId{SensorType::RANGE, topic});
    }
    if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
        (node_options_.map_builder_options.use_trajectory_builder_2d() &&
         options.trajectory_builder_options.trajectory_builder_2d_options()
             .use_imu_data())) {
        expected_topics.insert(SensorId{SensorType::IMU, kImuTopic});
    }
    if (options.use_odometry) {
        expected_topics.insert(SensorId{SensorType::ODOMETRY, kOdometryTopic});
    }
    if (options.use_nav_sat) {
        expected_topics.insert(
            SensorId{SensorType::FIXED_FRAME_POSE, kNavSatFixTopic});
    }
    if (options.use_landmarks) {
        expected_topics.insert(SensorId{SensorType::LANDMARK, kLandmarkTopic});
    }
    return expected_topics;
}

int CartographerNode::AddTrajectory(const TrajectoryOptions& options) {
    const std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>
        expected_sensor_ids = ComputeExpectedSensorIds(options);
    const int trajectory_id = map_builder_bridge_->AddTrajectory(
        expected_sensor_ids, options);
    AddExtrapolator(trajectory_id, options);
    AddSensorSamplers(trajectory_id, options);
    LaunchSubscribers(options, trajectory_id);
    for (const auto& sensor_id : expected_sensor_ids) {
        subscribed_topics_.insert(sensor_id.id);
    }
    return trajectory_id;
}

void CartographerNode::LaunchSubscribers(const TrajectoryOptions& options,
                                         const int trajectory_id) {
    CartographerNode* self = this;
    for (const std::string& topic :
         ComputeRepeatedTopicNames(kLaserScanTopic, options.num_laser_scans)) {
        node_->CreateReader<commsgs::sensor_msgs::LaserScan>(
            topic,
            [self, trajectory_id, topic](
                const std::shared_ptr<commsgs::sensor_msgs::LaserScan>& msg) {
                if (msg) {
                    self->HandleLaserScanMessage(trajectory_id, topic, *msg);
                }
            });
    }

    for (const std::string& topic : ComputeRepeatedTopicNames(
             kMultiEchoLaserScanTopic, options.num_multi_echo_laser_scans)) {
        node_->CreateReader<commsgs::sensor_msgs::MultiEchoLaserScan>(
            topic,
            [self, trajectory_id, topic](
                const std::shared_ptr<commsgs::sensor_msgs::MultiEchoLaserScan>&
                    msg) {
                if (msg) {
                    self->HandleMultiEchoLaserScanMessage(trajectory_id, topic,
                                                          *msg);
                }
            });
    }

    for (const std::string& topic : ComputeRepeatedTopicNames(
             kPointCloud2Topic, options.num_point_clouds)) {
        node_->CreateReader<commsgs::sensor_msgs::PointCloud2>(
            topic,
            [self, trajectory_id, topic](
                const std::shared_ptr<commsgs::sensor_msgs::PointCloud2>& msg) {
                if (msg) {
                    self->HandlePointCloud2Message(trajectory_id, topic, *msg);
                }
            });
    }

    if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
        (node_options_.map_builder_options.use_trajectory_builder_2d() &&
         options.trajectory_builder_options.trajectory_builder_2d_options()
             .use_imu_data())) {
        node_->CreateReader<commsgs::sensor_msgs::Imu>(
            kImuTopic,
            [self, trajectory_id](
                const std::shared_ptr<commsgs::sensor_msgs::Imu>& msg) {
                if (msg) {
                    self->HandleImuMessage(trajectory_id, kImuTopic, *msg);
                }
            });
    }

    if (options.use_odometry) {
        node_->CreateReader<commsgs::planning_msgs::Odometry>(
            kOdometryTopic,
            [self, trajectory_id](
                const std::shared_ptr<commsgs::planning_msgs::Odometry>& msg) {
                if (msg) {
                    self->HandleOdometryMessage(trajectory_id, kOdometryTopic,
                                                *msg);
                }
            });
    }

    if (options.use_nav_sat) {
        node_->CreateReader<commsgs::sensor_msgs::NavSatFix>(
            kNavSatFixTopic,
            [self, trajectory_id](
                const std::shared_ptr<commsgs::sensor_msgs::NavSatFix>& msg) {
                if (msg) {
                    self->HandleNavSatFixMessage(trajectory_id, kNavSatFixTopic,
                                                 *msg);
                }
            });
    }

    if (options.use_landmarks) {
        node_->CreateReader<proto::LandmarkList>(
            kLandmarkTopic,
            [self, trajectory_id](
                const std::shared_ptr<proto::LandmarkList>& msg) {
                if (msg) {
                    self->HandleLandmarkMessage(trajectory_id, kLandmarkTopic,
                                                *msg);
                }
            });
    }
}

void CartographerNode::AddExtrapolator(const int trajectory_id,
                                       const TrajectoryOptions& options) {
    constexpr double kExtrapolationEstimationTimeSec = 0.001;
    CHECK_EQ(extrapolators_.count(trajectory_id), 0);
    const double gravity_time_constant =
        node_options_.map_builder_options.use_trajectory_builder_3d()
            ? options.trajectory_builder_options
                  .trajectory_builder_3d_options()
                  .imu_gravity_time_constant()
            : options.trajectory_builder_options
                  .trajectory_builder_2d_options()
                  .imu_gravity_time_constant();
    extrapolators_.emplace(
        std::piecewise_construct, std::forward_as_tuple(trajectory_id),
        std::forward_as_tuple(
            carto::common::FromSeconds(kExtrapolationEstimationTimeSec),
            gravity_time_constant));
}

void CartographerNode::AddSensorSamplers(const int trajectory_id,
                                         const TrajectoryOptions& options) {
    CHECK_EQ(sensor_samplers_.count(trajectory_id), 0);
    sensor_samplers_.emplace(
        std::piecewise_construct, std::forward_as_tuple(trajectory_id),
        std::forward_as_tuple(
            options.rangefinder_sampling_ratio, options.odometry_sampling_ratio,
            options.fixed_frame_pose_sampling_ratio, options.imu_sampling_ratio,
            options.landmarks_sampling_ratio));
}

void CartographerNode::PublishSubmapList() {
    if (!submap_list_writer_) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = commsgs::builtin_interfaces::Time::Now();
    submap_list_writer_->Write(
        map_builder_bridge_->GetSubmapList(now));
}

void CartographerNode::PublishLocalTrajectoryData() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& entry : map_builder_bridge_->GetLocalTrajectoryData()) {
        const auto& trajectory_data = entry.second;
        auto& extrapolator = extrapolators_.at(entry.first);

        if (trajectory_data.local_slam_data->time !=
            extrapolator.GetLastPoseTime()) {
            extrapolator.AddPose(trajectory_data.local_slam_data->time,
                                 trajectory_data.local_slam_data->local_pose);
        }

        commsgs::geometry_msgs::TransformStamped stamped_transform;
        const carto::common::Time now = std::max(
            FromCommsgs(commsgs::builtin_interfaces::Time::Now()),
            extrapolator.GetLastExtrapolatedTime());
        stamped_transform.header.stamp =
            node_options_.use_pose_extrapolator
                ? ToCommsgs(now)
                : ToCommsgs(trajectory_data.local_slam_data->time);

        if (last_published_tf_stamps_.count(entry.first) &&
            last_published_tf_stamps_[entry.first] ==
                stamped_transform.header.stamp) {
            continue;
        }
        last_published_tf_stamps_[entry.first] = stamped_transform.header.stamp;

        const Rigid3d tracking_to_local_3d =
            node_options_.use_pose_extrapolator
                ? extrapolator.ExtrapolatePose(now)
                : trajectory_data.local_slam_data->local_pose;
        const Rigid3d tracking_to_local = [&] {
            if (trajectory_data.trajectory_options
                    .publish_frame_projected_to_2d) {
                return carto::transform::Embed3D(
                    carto::transform::Project2D(tracking_to_local_3d));
            }
            return tracking_to_local_3d;
        }();

        const Rigid3d tracking_to_map =
            trajectory_data.local_to_map * tracking_to_local;

        if (trajectory_data.published_to_tracking != nullptr) {
            if (node_options_.publish_to_tf && tf_broadcaster_) {
                std::vector<commsgs::geometry_msgs::TransformStamped>
                    stamped_transforms;
                if (trajectory_data.trajectory_options.provide_odom_frame) {
                    stamped_transform.header.frame_id = node_options_.map_frame;
                    stamped_transform.child_frame_id =
                        trajectory_data.trajectory_options.odom_frame;
                    stamped_transform.transform =
                        ToGeometryMsgTransform(trajectory_data.local_to_map);
                    stamped_transforms.push_back(stamped_transform);

                    stamped_transform.header.frame_id =
                        trajectory_data.trajectory_options.odom_frame;
                    stamped_transform.child_frame_id =
                        trajectory_data.trajectory_options.published_frame;
                    stamped_transform.transform = ToGeometryMsgTransform(
                        tracking_to_local *
                        (*trajectory_data.published_to_tracking));
                    stamped_transforms.push_back(stamped_transform);
                } else {
                    stamped_transform.header.frame_id = node_options_.map_frame;
                    stamped_transform.child_frame_id =
                        trajectory_data.trajectory_options.published_frame;
                    stamped_transform.transform = ToGeometryMsgTransform(
                        tracking_to_map *
                        (*trajectory_data.published_to_tracking));
                    stamped_transforms.push_back(stamped_transform);
                }
                tf_broadcaster_->SendTransform(stamped_transforms);
                for (const auto& transform : stamped_transforms) {
                    if (tf_buffer_) {
                        SetTransformInBuffer(tf_buffer_, transform,
                                             "cartographer_node", true);
                    }
                }
            }
            if (node_options_.publish_tracked_pose && tracked_pose_writer_) {
                commsgs::geometry_msgs::PoseStamped pose_msg;
                pose_msg.header.frame_id = node_options_.map_frame;
                pose_msg.header.stamp = stamped_transform.header.stamp;
                pose_msg.pose = ToGeometryMsgPose(tracking_to_map);
                tracked_pose_writer_->Write(pose_msg);
            }
        }
    }
}

void CartographerNode::PublishOccupancyGrid() {
    if (!occupancy_grid_writer_) {
        return;
    }

    std::map<carto::mapping::SubmapId, carto::io::SubmapSlice> submap_slices;
    proto::SubmapList submap_list;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto now = commsgs::builtin_interfaces::Time::Now();
        submap_list = map_builder_bridge_->GetSubmapList(now);
    }

    for (const auto& submap_msg : submap_list.submap()) {
        const carto::mapping::SubmapId id{submap_msg.trajectory_id(),
                                          submap_msg.submap_index()};
        proto::SubmapQueryRequest request;
        request.set_trajectory_id(id.trajectory_id);
        request.set_submap_index(id.submap_index);
        proto::SubmapQueryResponse response;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            map_builder_bridge_->HandleSubmapQuery(request, &response);
        }
        if (response.status().code() != proto::OK ||
            response.textures().empty()) {
            continue;
        }
        const auto& fetched_texture = response.textures(0);
        carto::io::SubmapSlice& submap_slice = submap_slices[id];
        commsgs::geometry_msgs::Pose pose = FromProtoPose(submap_msg.pose());
        submap_slice.pose = ToRigid3d(pose);
        submap_slice.metadata_version = submap_msg.submap_version();
        submap_slice.version = submap_msg.submap_version();
        submap_slice.width = fetched_texture.width();
        submap_slice.height = fetched_texture.height();
        submap_slice.resolution = fetched_texture.resolution();
        submap_slice.slice_pose =
            ToRigid3d(FromProtoPose(fetched_texture.slice_pose()));
        const std::string compressed_cells = fetched_texture.cells();
        const auto pixels = carto::io::UnpackTextureData(
            compressed_cells, fetched_texture.width(),
            fetched_texture.height());
        submap_slice.cairo_data.clear();
        submap_slice.surface = carto::io::DrawTexture(
            pixels.intensity, pixels.alpha, fetched_texture.width(),
            fetched_texture.height(), &submap_slice.cairo_data);
    }

    if (submap_slices.empty()) {
        return;
    }

    const auto painted_slices = carto::io::PaintSubmapSlices(
        submap_slices, node_options_.occupancy_grid_resolution);
    const auto now = commsgs::builtin_interfaces::Time::Now();
    auto grid = CreateOccupancyGridMsg(
        painted_slices, node_options_.occupancy_grid_resolution,
        node_options_.map_frame, now);
    if (grid) {
        occupancy_grid_writer_->Write(*grid);
    }
}

void CartographerNode::HandleOdometryMessage(
    const int trajectory_id, const std::string& sensor_id,
    const commsgs::planning_msgs::Odometry& msg) {
    auto& sampler = sensor_samplers_.at(trajectory_id);
    if (!sampler.odometry_sampler.Pulse()) {
        return;
    }
    map_builder_bridge_->sensor_bridge(trajectory_id)
        ->HandleOdometryMessage(sensor_id, msg);
}

void CartographerNode::HandleImuMessage(
    const int trajectory_id, const std::string& sensor_id,
    const commsgs::sensor_msgs::Imu& msg) {
    auto& sampler = sensor_samplers_.at(trajectory_id);
    if (!sampler.imu_sampler.Pulse()) {
        return;
    }
    map_builder_bridge_->sensor_bridge(trajectory_id)
        ->HandleImuMessage(sensor_id, msg);
}

void CartographerNode::HandleLaserScanMessage(
    const int trajectory_id, const std::string& sensor_id,
    const commsgs::sensor_msgs::LaserScan& msg) {
    auto& sampler = sensor_samplers_.at(trajectory_id);
    if (!sampler.rangefinder_sampler.Pulse()) {
        return;
    }
    map_builder_bridge_->sensor_bridge(trajectory_id)
        ->HandleLaserScanMessage(sensor_id, msg);
}

void CartographerNode::HandlePointCloud2Message(
    const int trajectory_id, const std::string& sensor_id,
    const commsgs::sensor_msgs::PointCloud2& msg) {
    auto& sampler = sensor_samplers_.at(trajectory_id);
    if (!sampler.rangefinder_sampler.Pulse()) {
        return;
    }
    map_builder_bridge_->sensor_bridge(trajectory_id)
        ->HandlePointCloud2Message(sensor_id, msg);
}

void CartographerNode::HandleMultiEchoLaserScanMessage(
    const int trajectory_id, const std::string& sensor_id,
    const commsgs::sensor_msgs::MultiEchoLaserScan& msg) {
    auto& sampler = sensor_samplers_.at(trajectory_id);
    if (!sampler.rangefinder_sampler.Pulse()) {
        return;
    }
    map_builder_bridge_->sensor_bridge(trajectory_id)
        ->HandleMultiEchoLaserScanMessage(sensor_id, msg);
}

void CartographerNode::HandleNavSatFixMessage(
    const int trajectory_id, const std::string& sensor_id,
    const commsgs::sensor_msgs::NavSatFix& msg) {
    auto& sampler = sensor_samplers_.at(trajectory_id);
    if (!sampler.fixed_frame_pose_sampler.Pulse()) {
        return;
    }
    map_builder_bridge_->sensor_bridge(trajectory_id)
        ->HandleNavSatFixMessage(sensor_id, msg);
}

void CartographerNode::HandleLandmarkMessage(
    const int trajectory_id, const std::string& sensor_id,
    const proto::LandmarkList& msg) {
    auto& sampler = sensor_samplers_.at(trajectory_id);
    if (!sampler.landmark_sampler.Pulse()) {
        return;
    }
    map_builder_bridge_->sensor_bridge(trajectory_id)
        ->HandleLandmarkMessage(sensor_id, msg);
}

void CartographerNode::HandleSubmapQuery(
    const std::shared_ptr<proto::SubmapQueryRequest>& request,
    std::shared_ptr<proto::SubmapQueryResponse>& response) {
    if (!response) {
        response = std::make_shared<proto::SubmapQueryResponse>();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    map_builder_bridge_->HandleSubmapQuery(*request, response.get());
}

void CartographerNode::HandleStartTrajectory(
    const std::shared_ptr<proto::StartTrajectoryRequest>& request,
    std::shared_ptr<proto::StartTrajectoryResponse>& response) {
    if (!response) {
        response = std::make_shared<proto::StartTrajectoryResponse>();
    }
    TrajectoryOptions options;
    try {
        std::tie(std::ignore, options) =
            LoadOptions(request->configuration_directory(),
                        request->configuration_basename());
    } catch (const std::exception& ex) {
        response->mutable_status()->set_code(proto::INVALID_ARGUMENT);
        response->mutable_status()->set_message(ex.what());
        return;
    }
    const int trajectory_id = AddTrajectory(options);
    response->set_trajectory_id(trajectory_id);
    response->mutable_status()->set_code(proto::OK);
    response->mutable_status()->set_message("Success.");
}

void CartographerNode::HandleFinishTrajectory(
    const std::shared_ptr<proto::FinishTrajectoryRequest>& request,
    std::shared_ptr<proto::FinishTrajectoryResponse>& response) {
    if (!response) {
        response = std::make_shared<proto::FinishTrajectoryResponse>();
    }
    if (!FinishTrajectory(request->trajectory_id())) {
        response->mutable_status()->set_code(proto::NOT_FOUND);
        response->mutable_status()->set_message("Trajectory not found.");
        return;
    }
    response->mutable_status()->set_code(proto::OK);
    response->mutable_status()->set_message("Success.");
}

void CartographerNode::HandleWriteState(
    const std::shared_ptr<proto::WriteStateRequest>& request,
    std::shared_ptr<proto::WriteStateResponse>& response) {
    if (!response) {
        response = std::make_shared<proto::WriteStateResponse>();
    }
    SerializeState(request->filename(), request->include_unfinished_submaps());
    response->mutable_status()->set_code(proto::OK);
    response->mutable_status()->set_message("Success.");
}

}  // namespace node
}  // namespace cartographer
}  // namespace localization
}  // namespace autonomy
