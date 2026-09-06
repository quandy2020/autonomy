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
 * @file shadow_component.hpp
 * @brief Four-input autolink component for safe selected-person following.
 */

#ifndef AUTONOMY_PERCEPTION_SHADOW_SHADOW_COMPONENT_HPP_
#define AUTONOMY_PERCEPTION_SHADOW_SHADOW_COMPONENT_HPP_

#include "autonomy/perception/shadow/detector.hpp"
#include "autonomy/perception/shadow/grid.hpp"
#include "autonomy/perception/shadow/localizer.hpp"
#include "autonomy/perception/shadow/planner.hpp"
#include "autonomy/perception/shadow/policy.hpp"
#include "autonomy/perception/shadow/tracker.hpp"
#include "autonomy/transform/autolink_tf_listener.hpp"
#include "autonomy/transform/buffer.hpp"

#include "autolink/component/component.hpp"

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/map_msgs/grid_map.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/std_msgs/string.pb.h>
#include <automsgs/msgs/vision_msgs/detection2d_array.pb.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

namespace autonomy {
namespace perception {
namespace shadow {

class ShadowComponentTestApi;

/**
 * @brief Coordinates Shadow perception, localization, mapping, and planning.
 *
 * Inputs are fixed in RGB, aligned-depth, CameraInfo, and Odometry order.
 * Target selection arrives asynchronously on a separate String channel. The
 * component publishes paths only; low-level velocity control remains outside
 * Shadow.
 */
class ShadowComponent final
    : public autolink::Component<automsgs::msgs::sensor_msgs::Image,
                                 automsgs::msgs::sensor_msgs::Image,
                                 automsgs::msgs::sensor_msgs::CameraInfo,
                                 automsgs::msgs::nav_msgs::Odometry>
{
public:
    using Image = automsgs::msgs::sensor_msgs::Image;
    using CameraInfo = automsgs::msgs::sensor_msgs::CameraInfo;
    using Odometry = automsgs::msgs::nav_msgs::Odometry;
    using Selection = automsgs::msgs::std_msgs::String;
    using Detection2DArray = automsgs::msgs::vision_msgs::Detection2DArray;
    using PoseStamped = automsgs::msgs::geometry_msgs::PoseStamped;
    using TransformStamped = automsgs::msgs::geometry_msgs::TransformStamped;
    using Path = automsgs::msgs::nav_msgs::Path;
    using GridMap = automsgs::msgs::map_msgs::GridMap;

    ~ShadowComponent() override;

    /** @brief Loads the profile and creates all long-lived resources. */
    bool Init() override;

    /** @brief Processes one synchronized RGB-D, calibration, and pose tuple. */
    bool Proc(const std::shared_ptr<Image>& rgb,
              const std::shared_ptr<Image>& depth,
              const std::shared_ptr<CameraInfo>& camera_info,
              const std::shared_ptr<Odometry>& odometry) override;

protected:
    /** @brief Releases transport, TF, inference, and accumulated state. */
    void Clear() override;

private:
    friend class ShadowComponentTestApi;

    struct FrameOutputs {
        Detection2DArray detections;
        PoseStamped target;
        Path path;
        GridMap grid;
        bool has_detections = false;
        bool has_target = false;
        bool has_path = false;
        bool has_grid = false;

        void Clear() {
            detections.Clear();
            target.Clear();
            path.Clear();
            grid.Clear();
            has_detections = false;
            has_target = false;
            has_path = false;
            has_grid = false;
        }
    };

    enum class InputStatus {
        kValid,
        kInvalid,
        kStale,
    };

    using ClockFunction = std::function<uint64_t()>;
    using SelectionFunction = std::function<void(const std::string&)>;
    using TransformLookup =
        std::function<bool(const Image&, TransformStamped*, std::string*)>;
    using ProcessFunction = std::function<bool(
        const Image&, const Image&, const CameraInfo&, const Odometry&,
        const TransformStamped&, FrameOutputs*, std::string*)>;
    using DetectionPublisher = std::function<bool(const Detection2DArray&)>;
    using TargetPublisher = std::function<bool(const PoseStamped&)>;
    using PathPublisher = std::function<bool(const Path&)>;
    using GridPublisher = std::function<bool(const GridMap&)>;

    void HandleSelection(const std::shared_ptr<Selection>& selection);
    void ApplyPendingSelection();
    bool ResolveAutomaticSelection(const Image& depth, const CameraInfo& camera,
                                   std::string* error);
    bool LookupCameraToMap(const Image& rgb, TransformStamped* transform,
                           std::string* error) const;
    InputStatus ValidateInputs(const Image& rgb, const Image& depth,
                               const CameraInfo& camera,
                               const Odometry& odometry, uint64_t now_ns,
                               int64_t* stamp_ns, std::string* error) const;
    bool ValidateTransform(const TransformStamped& transform,
                           int64_t rgb_stamp_ns, std::string* error) const;
    bool ProcessFrame(const Image& rgb, const Image& depth,
                      const CameraInfo& camera, const Odometry& odometry,
                      const TransformStamped& camera_to_map,
                      FrameOutputs* outputs, std::string* error);
    void StampEmptyPath(const Image& rgb, Path* path) const;
    bool PublishOutputs(const FrameOutputs& outputs) const;

    proto::ShadowOptions options_;
    std::unique_ptr<YoloDetector> detector_;
    std::unique_ptr<PersonTracker> tracker_;
    std::unique_ptr<TargetLocalizer> localizer_;
    std::unique_ptr<LocalGrid> local_grid_;
    std::unique_ptr<YopoPolicy> policy_;
    std::unique_ptr<FollowPlanner> planner_;
    std::unique_ptr<transform::AutolinkTfListener> tf_listener_;
    transform::Buffer* tf_buffer_ = nullptr;

    std::shared_ptr<autolink::Reader<Selection>> selection_reader_;
    std::shared_ptr<autolink::Writer<Detection2DArray>> detections_writer_;
    std::shared_ptr<autolink::Writer<PoseStamped>> target_writer_;
    std::shared_ptr<autolink::Writer<Path>> path_writer_;
    std::shared_ptr<autolink::Writer<GridMap>> grid_writer_;

    mutable std::mutex selection_mutex_;
    bool selection_pending_ = false;
    bool automatic_selection_pending_ = false;
    std::string pending_selection_;

    ClockFunction now_;
    SelectionFunction select_;
    TransformLookup lookup_transform_;
    ProcessFunction process_;
    DetectionPublisher publish_detections_;
    TargetPublisher publish_target_;
    PathPublisher publish_path_;
    GridPublisher publish_grid_;
    bool initialized_ = false;
};

}  // namespace shadow
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_SHADOW_SHADOW_COMPONENT_HPP_
