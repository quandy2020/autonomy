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
 * @file shadow_component.cpp
 * @brief Shadow autolink lifecycle and coherent frame orchestration.
 */

#include "autonomy/perception/shadow/shadow_component.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/perception/shadow/options.hpp"

#include "autolink/time/clock.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autonomy {
namespace perception {
namespace shadow {
namespace {

using Header = automsgs::msgs::std_msgs::Header;

constexpr int64_t kNanosecondsPerSecond = 1'000'000'000LL;

void SetError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = "Shadow component: " + message;
    }
}

bool StampNanoseconds(const Header& header, const char* label,
                      int64_t* stamp_ns, std::string* error) {
    if (stamp_ns == nullptr) {
        SetError(error, "timestamp output is null.");
        return false;
    }
    if (header.stamp().sec() < 0 ||
        header.stamp().nanosec() >=
            static_cast<uint32_t>(kNanosecondsPerSecond)) {
        SetError(error, std::string(label) + " timestamp is invalid.");
        return false;
    }
    *stamp_ns =
        static_cast<int64_t>(header.stamp().sec()) * kNanosecondsPerSecond +
        static_cast<int64_t>(header.stamp().nanosec());
    if (*stamp_ns <= 0) {
        SetError(error, std::string(label) + " timestamp must be positive.");
        return false;
    }
    return true;
}

bool ImageStorageIsValid(const ShadowComponent::Image& image,
                         size_t bytes_per_pixel, const char* label,
                         std::string* error) {
    if (image.width() == 0 || image.height() == 0) {
        SetError(error, std::string(label) + " dimensions must be positive.");
        return false;
    }
    if (static_cast<size_t>(image.width()) >
        std::numeric_limits<size_t>::max() / bytes_per_pixel) {
        SetError(error, std::string(label) + " row size overflows.");
        return false;
    }
    const size_t minimum_step =
        static_cast<size_t>(image.width()) * bytes_per_pixel;
    if (image.step() < minimum_step) {
        SetError(error, std::string(label) + " step is too small.");
        return false;
    }
    if (static_cast<size_t>(image.height()) >
        std::numeric_limits<size_t>::max() /
            static_cast<size_t>(image.step())) {
        SetError(error, std::string(label) + " byte size overflows.");
        return false;
    }
    const size_t minimum_size =
        static_cast<size_t>(image.height()) * image.step();
    if (image.data().size() < minimum_size) {
        SetError(error, std::string(label) + " data is too small.");
        return false;
    }
    return true;
}

bool QuaternionIsValid(
    const automsgs::msgs::geometry_msgs::Quaternion& quaternion) {
    const double x = quaternion.x();
    const double y = quaternion.y();
    const double z = quaternion.z();
    const double w = quaternion.w();
    const double norm_squared = x * x + y * y + z * z + w * w;
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
           std::isfinite(w) && std::isfinite(norm_squared) &&
           norm_squared > std::numeric_limits<double>::epsilon();
}

bool PointIsFinite(const automsgs::msgs::geometry_msgs::Point& point) {
    return std::isfinite(point.x()) && std::isfinite(point.y()) &&
           std::isfinite(point.z());
}

bool VectorIsFinite(const automsgs::msgs::geometry_msgs::Vector3& vector) {
    return std::isfinite(vector.x()) && std::isfinite(vector.y()) &&
           std::isfinite(vector.z());
}

uint16_t DecodeUint16(const char* data, bool big_endian) {
    const auto* bytes = reinterpret_cast<const uint8_t*>(data);
    if (big_endian) {
        return static_cast<uint16_t>((static_cast<uint16_t>(bytes[0]) << 8) |
                                     static_cast<uint16_t>(bytes[1]));
    }
    return static_cast<uint16_t>(static_cast<uint16_t>(bytes[0]) |
                                 (static_cast<uint16_t>(bytes[1]) << 8));
}

void EncodeFloat32LittleEndian(float value, char* data) {
    uint32_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    data[0] = static_cast<char>(bits & 0xffU);
    data[1] = static_cast<char>((bits >> 8) & 0xffU);
    data[2] = static_cast<char>((bits >> 16) & 0xffU);
    data[3] = static_cast<char>((bits >> 24) & 0xffU);
}

}  // namespace

ShadowComponent::~ShadowComponent() {
    Clear();
}

bool ShadowComponent::Init() {
    Clear();

    proto::ShadowOptions options;
    if (!GetProtoConfig(&options)) {
        AERROR << "Shadow component failed to load config from '"
               << ConfigFilePath() << "'.";
        return false;
    }

    std::string error;
    if (!ValidateShadowOptions(options, &error)) {
        AERROR << error;
        return false;
    }
    if (node_ == nullptr) {
        AERROR << "Shadow component has no autolink node.";
        Clear();
        return false;
    }
    options_ = options;

    try {
        detector_ = YoloDetector::Create(options_, &error);
        if (detector_ == nullptr) {
            AERROR << "Shadow component failed to create detector: " << error;
            Clear();
            return false;
        }
        tracker_ = std::make_unique<PersonTracker>(options_);
        localizer_ = std::make_unique<TargetLocalizer>(options_);
        local_grid_ = std::make_unique<LocalGrid>(options_);
        policy_ = YopoPolicy::Create(options_, &error);
        if (policy_ == nullptr) {
            AERROR << "Shadow component failed to create policy: " << error;
            Clear();
            return false;
        }
        planner_ = std::make_unique<FollowPlanner>(options_);
    } catch (const std::exception& exception) {
        AERROR << "Shadow component resource creation failed: "
               << exception.what();
        Clear();
        return false;
    }

    select_ = [this](const std::string& target_id) {
        tracker_->Select(target_id);
        localizer_->Clear();
    };
    confirmed_tracks_ = [this](Detection2DArray* output) {
        tracker_->Confirmed(output);
    };
    selection_reader_ = node_->CreateReader<Selection>(
        options_.select_topic(),
        [this](const std::shared_ptr<Selection>& selection) {
            HandleSelection(selection);
        });
    detections_writer_ =
        node_->CreateWriter<Detection2DArray>(options_.detections_topic());
    target_writer_ = node_->CreateWriter<PoseStamped>(options_.target_topic());
    path_writer_ = node_->CreateWriter<Path>(options_.path_topic());
    grid_writer_ = node_->CreateWriter<GridMap>(options_.grid_topic());
    if (selection_reader_ == nullptr || detections_writer_ == nullptr ||
        target_writer_ == nullptr || path_writer_ == nullptr ||
        grid_writer_ == nullptr) {
        AERROR << "Shadow component failed to create selection or output "
                  "transport endpoints.";
        Clear();
        return false;
    }

    listener_start_ = [](transform::AutolinkTfListener* listener,
                         const std::shared_ptr<autolink::Node>& node,
                         const std::string& dynamic_topic,
                         const std::string& legacy_topic,
                         const std::string& static_topic) {
        return listener->Start(node, dynamic_topic, legacy_topic, static_topic);
    };
    if (!StartTransformListeners(&error)) {
        AERROR << error;
        Clear();
        return false;
    }

    now_ = [] { return autolink::Clock::Now().ToNanosecond(); };
    lookup_transform_ = [this](const Image& rgb, TransformStamped* transform,
                               std::string* lookup_error) {
        return LookupCameraToMap(rgb, transform, lookup_error);
    };
    process_ = [this](const Image& rgb, const Image& depth,
                      const CameraInfo& camera, const Odometry& odometry,
                      const TransformStamped& camera_to_map,
                      FrameOutputs* outputs, std::string* process_error) {
        return ProcessFrame(rgb, depth, camera, odometry, camera_to_map,
                            outputs, process_error);
    };
    publish_detections_ = [this](const Detection2DArray& message) {
        return detections_writer_->Write(message);
    };
    publish_target_ = [this](const PoseStamped& message) {
        return target_writer_->Write(message);
    };
    publish_path_ = [this](const Path& message) {
        return path_writer_->Write(message);
    };
    publish_grid_ = [this](const GridMap& message) {
        return grid_writer_->Write(message);
    };
    initialized_ = true;
    return true;
}

void ShadowComponent::HandleSelection(
    const std::shared_ptr<Selection>& selection) {
    if (selection == nullptr) {
        AWARN << "Shadow component ignored a null selection message.";
        return;
    }
    std::lock_guard<std::mutex> lock(selection_mutex_);
    pending_selection_ = selection->data();
    selection_pending_ = true;
}

void ShadowComponent::ApplyPendingSelection() {
    std::string target_id;
    {
        std::lock_guard<std::mutex> lock(selection_mutex_);
        if (!selection_pending_) {
            return;
        }
        target_id = pending_selection_;
        pending_selection_.clear();
        selection_pending_ = false;
        automatic_selection_pending_ = target_id.empty();
    }
    select_(target_id);
}

bool ShadowComponent::CanonicalizeDepth(const Image& input, Image* output,
                                        std::string* error) const {
    if (error != nullptr) {
        error->clear();
    }
    if (output == nullptr) {
        SetError(error, "canonical depth output is null.");
        return false;
    }
    output->Clear();

    size_t input_bytes_per_pixel = 0;
    if (input.encoding() == "16UC1") {
        input_bytes_per_pixel = sizeof(uint16_t);
        if (!std::isfinite(options_.depth_scale()) ||
            options_.depth_scale() <= 0.0F) {
            SetError(error, "depth_scale must be finite and positive.");
            return false;
        }
    } else if (input.encoding() == "32FC1") {
        input_bytes_per_pixel = sizeof(float);
    } else {
        SetError(error, "depth encoding must be '16UC1' or '32FC1'.");
        return false;
    }
    if (!ImageStorageIsValid(input, input_bytes_per_pixel, "depth image",
                             error)) {
        return false;
    }
    if (input.width() > std::numeric_limits<uint32_t>::max() / sizeof(float)) {
        SetError(error, "canonical depth row size exceeds uint32 step.");
        return false;
    }
    const uint32_t output_step =
        input.width() * static_cast<uint32_t>(sizeof(float));
    if (static_cast<size_t>(input.height()) >
        std::numeric_limits<size_t>::max() / static_cast<size_t>(output_step)) {
        SetError(error, "canonical depth byte size overflows.");
        return false;
    }

    Image canonical;
    *canonical.mutable_header() = input.header();
    canonical.set_height(input.height());
    canonical.set_width(input.width());
    canonical.set_encoding("32FC1");
    canonical.set_is_bigendian(false);
    canonical.set_step(output_step);
    canonical.mutable_data()->resize(static_cast<size_t>(input.height()) *
                                     output_step);
    for (uint32_t row = 0; row < input.height(); ++row) {
        const char* input_row =
            input.data().data() + static_cast<size_t>(row) * input.step();
        char* output_row = canonical.mutable_data()->data() +
                           static_cast<size_t>(row) * output_step;
        for (uint32_t column = 0; column < input.width(); ++column) {
            const char* source =
                input_row + static_cast<size_t>(column) * input_bytes_per_pixel;
            char* destination =
                output_row + static_cast<size_t>(column) * sizeof(float);
            if (input.encoding() == "16UC1") {
                const float value_m = static_cast<float>(DecodeUint16(
                                          source, input.is_bigendian())) *
                                      options_.depth_scale();
                if (!std::isfinite(value_m)) {
                    SetError(
                        error,
                        "scaled 16UC1 depth is not a finite metric value.");
                    return false;
                }
                EncodeFloat32LittleEndian(value_m, destination);
            } else if (input.is_bigendian()) {
                destination[0] = source[3];
                destination[1] = source[2];
                destination[2] = source[1];
                destination[3] = source[0];
            } else {
                std::memcpy(destination, source, sizeof(float));
            }
        }
    }
    *output = std::move(canonical);
    return true;
}

bool ShadowComponent::StartTransformListeners(std::string* error) {
    if (error != nullptr) {
        error->clear();
    }
    if (!listener_start_) {
        SetError(error, "TF listener start callback is unavailable.");
        return false;
    }

    tf_buffer_ = transform::Buffer::Instance();
    dynamic_tf_listener_ = std::make_unique<transform::AutolinkTfListener>();
    static_tf_listener_ = std::make_unique<transform::AutolinkTfListener>();
    if (!listener_start_(dynamic_tf_listener_.get(), node_, "/tf", "", "")) {
        SetError(error, "failed to subscribe to dynamic TF on /tf.");
        return false;
    }
    if (!listener_start_(static_tf_listener_.get(), node_, "", "",
                         "/tf_static")) {
        SetError(error, "failed to subscribe to static TF on /tf_static.");
        return false;
    }
    return true;
}

bool ShadowComponent::ResolveAutomaticSelection(const Image& depth,
                                                const CameraInfo& camera,
                                                std::string* error) {
    Detection2DArray confirmed;
    confirmed_tracks_(&confirmed);
    std::string nearest_id;
    float nearest_range = std::numeric_limits<float>::infinity();
    for (const auto& detection : confirmed.detections()) {
        float range_m = 0.0F;
        std::string range_error;
        if (detection.id().empty() ||
            !localizer_->EstimateRange(detection, depth, camera, &range_m,
                                       &range_error)) {
            continue;
        }
        if (range_m < nearest_range) {
            nearest_range = range_m;
            nearest_id = detection.id();
        }
    }
    if (nearest_id.empty()) {
        SetError(error,
                 "automatic selection found no confirmed track with a valid "
                 "range.");
        return false;
    }

    select_(nearest_id);
    {
        std::lock_guard<std::mutex> lock(selection_mutex_);
        if (!selection_pending_) {
            automatic_selection_pending_ = false;
        }
    }
    return true;
}

ShadowComponent::InputStatus ShadowComponent::ValidateInputs(
    const Image& rgb, const Image& depth, const CameraInfo& camera,
    const Odometry& odometry, uint64_t now_ns, int64_t* stamp_ns,
    std::string* error) const {
    if (error != nullptr) {
        error->clear();
    }
    if (stamp_ns == nullptr) {
        SetError(error, "RGB timestamp output is null.");
        return InputStatus::kInvalid;
    }
    if (rgb.encoding() != "rgb8" && rgb.encoding() != "bgr8") {
        SetError(error, "RGB encoding must be 'rgb8' or 'bgr8'.");
        return InputStatus::kInvalid;
    }
    if (rgb.is_bigendian()) {
        SetError(error, "big-endian RGB images are unsupported.");
        return InputStatus::kInvalid;
    }
    if (!ImageStorageIsValid(rgb, 3, "RGB image", error)) {
        return InputStatus::kInvalid;
    }

    if (camera.width() == 0 || camera.height() == 0 ||
        rgb.width() != depth.width() || rgb.height() != depth.height() ||
        depth.width() != camera.width() || depth.height() != camera.height()) {
        SetError(error,
                 "RGB, depth, and CameraInfo dimensions must be positive and "
                 "match.");
        return InputStatus::kInvalid;
    }
    if (camera.k_size() < 9 || !std::isfinite(camera.k(0)) ||
        camera.k(0) <= 0.0 || !std::isfinite(camera.k(4)) ||
        camera.k(4) <= 0.0 || !std::isfinite(camera.k(2)) ||
        !std::isfinite(camera.k(5))) {
        SetError(error, "CameraInfo intrinsics are invalid.");
        return InputStatus::kInvalid;
    }
    if (rgb.header().frame_id() != options_.camera_frame() ||
        depth.header().frame_id() != options_.camera_frame() ||
        camera.header().frame_id() != options_.camera_frame()) {
        SetError(error,
                 "RGB, depth, and CameraInfo frames must match camera_frame.");
        return InputStatus::kInvalid;
    }
    if (odometry.header().frame_id() != options_.map_frame() ||
        odometry.child_frame_id() != options_.base_frame()) {
        SetError(error, "Odometry frames must match map_frame and base_frame.");
        return InputStatus::kInvalid;
    }
    const auto& robot_pose = odometry.pose().pose().pose();
    if (!PointIsFinite(robot_pose.position()) ||
        !QuaternionIsValid(robot_pose.orientation()) ||
        !VectorIsFinite(odometry.twist().twist().linear()) ||
        !VectorIsFinite(odometry.twist().twist().angular())) {
        SetError(error, "Odometry pose or velocity is invalid.");
        return InputStatus::kInvalid;
    }

    std::array<int64_t, 4> stamps{};
    if (!StampNanoseconds(rgb.header(), "RGB", &stamps[0], error) ||
        !StampNanoseconds(depth.header(), "depth", &stamps[1], error) ||
        !StampNanoseconds(camera.header(), "CameraInfo", &stamps[2], error) ||
        !StampNanoseconds(odometry.header(), "Odometry", &stamps[3], error)) {
        return InputStatus::kInvalid;
    }
    *stamp_ns = stamps[0];

    const auto minimum_and_maximum =
        std::minmax_element(stamps.begin(), stamps.end());
    const long double skew_ns = static_cast<long double>(
        *minimum_and_maximum.second - *minimum_and_maximum.first);
    const long double maximum_skew_ns =
        static_cast<long double>(options_.max_input_skew_sec()) *
        kNanosecondsPerSecond;
    if (skew_ns > maximum_skew_ns) {
        SetError(error, "input timestamp skew exceeds max_input_skew_sec.");
        return InputStatus::kInvalid;
    }

    const long double maximum_age_ns =
        static_cast<long double>(options_.max_data_age_sec()) *
        kNanosecondsPerSecond;
    for (const int64_t input_stamp : stamps) {
        const long double age_ns =
            std::fabs(static_cast<long double>(now_ns) - input_stamp);
        if (age_ns > maximum_age_ns) {
            SetError(error, "input data exceeds max_data_age_sec.");
            return InputStatus::kStale;
        }
    }
    return InputStatus::kValid;
}

bool ShadowComponent::LookupCameraToMap(const Image& rgb,
                                        TransformStamped* transform,
                                        std::string* error) const {
    if (error != nullptr) {
        error->clear();
    }
    if (transform == nullptr || tf_buffer_ == nullptr) {
        SetError(error, "camera-to-map transform output or buffer is null.");
        return false;
    }
    try {
        *transform = tf_buffer_->lookupTransform(options_.map_frame(),
                                                 options_.camera_frame(),
                                                 rgb.header().stamp());
        return true;
    } catch (const std::exception& exception) {
        SetError(error, std::string("camera-to-map TF lookup failed: ") +
                            exception.what());
        return false;
    }
}

bool ShadowComponent::ValidateTransform(const TransformStamped& transform,
                                        int64_t rgb_stamp_ns,
                                        std::string* error) const {
    if (transform.header().frame_id() != options_.map_frame() ||
        transform.child_frame_id() != options_.camera_frame()) {
        SetError(error, "camera-to-map TF frames are invalid.");
        return false;
    }
    if (!VectorIsFinite(transform.transform().translation()) ||
        !QuaternionIsValid(transform.transform().rotation())) {
        SetError(error, "camera-to-map TF is non-finite or invalid.");
        return false;
    }

    const Header& header = transform.header();
    if (header.stamp().sec() == 0 && header.stamp().nanosec() == 0) {
        return true;
    }
    int64_t transform_stamp_ns = 0;
    if (!StampNanoseconds(header, "TF", &transform_stamp_ns, error)) {
        return false;
    }
    const long double age_ns =
        std::fabs(static_cast<long double>(rgb_stamp_ns) - transform_stamp_ns);
    const long double maximum_age_ns =
        static_cast<long double>(options_.max_data_age_sec()) *
        kNanosecondsPerSecond;
    if (age_ns > maximum_age_ns) {
        SetError(error, "camera-to-map TF exceeds max_data_age_sec.");
        return false;
    }
    return true;
}

bool ShadowComponent::ProcessFrame(const Image& rgb, const Image& depth,
                                   const CameraInfo& camera,
                                   const Odometry& odometry,
                                   const TransformStamped& camera_to_map,
                                   FrameOutputs* outputs, std::string* error) {
    if (error != nullptr) {
        error->clear();
    }
    if (outputs == nullptr) {
        SetError(error, "frame output is null.");
        return false;
    }
    outputs->Clear();
    if (detector_ == nullptr || tracker_ == nullptr || localizer_ == nullptr ||
        local_grid_ == nullptr || policy_ == nullptr || planner_ == nullptr) {
        SetError(error, "processing resources are incomplete.");
        return false;
    }

    int64_t stamp_ns = 0;
    if (!StampNanoseconds(rgb.header(), "RGB", &stamp_ns, error)) {
        return false;
    }

    Detection2DArray detections;
    if (!detector_->Detect(rgb, &detections, error) ||
        !tracker_->Update(stamp_ns, &detections, error)) {
        return false;
    }
    outputs->detections = detections;
    outputs->has_detections = true;

    if (!local_grid_->Update(stamp_ns, depth, camera, camera_to_map, odometry,
                             error)) {
        return false;
    }
    if (!local_grid_->ToMessage(&outputs->grid, error)) {
        return false;
    }
    outputs->has_grid = true;

    if (automatic_selection_pending_ &&
        !ResolveAutomaticSelection(depth, camera, error)) {
        return false;
    }

    automsgs::msgs::vision_msgs::Detection2D selected;
    if (!tracker_->Selected(&selected)) {
        SetError(error, "selected target is unavailable or lost.");
        return false;
    }

    automsgs::msgs::geometry_msgs::TwistStamped target_velocity;
    if (!localizer_->Locate(selected.id(), stamp_ns, selected, depth, camera,
                            camera_to_map, &outputs->target, &target_velocity,
                            error)) {
        return false;
    }
    outputs->has_target = true;

    std::vector<Path> candidates;
    std::vector<float> learned_scores;
    if (!policy_->Generate(depth, odometry, outputs->target, target_velocity,
                           &candidates, &learned_scores, error)) {
        return false;
    }
    if (!planner_->Select(stamp_ns, candidates, learned_scores,
                          local_grid_->map(), odometry, outputs->target,
                          &outputs->path, error)) {
        return false;
    }
    outputs->has_path = true;
    if (outputs->path.poses_size() == 0) {
        SetError(error, "no safe following candidate is available.");
        return false;
    }
    return true;
}

void ShadowComponent::StampEmptyPath(const Image& rgb, Path* path) const {
    path->Clear();
    *path->mutable_header()->mutable_stamp() = rgb.header().stamp();
    path->mutable_header()->set_frame_id(options_.map_frame());
}

bool ShadowComponent::PublishOutputs(const FrameOutputs& outputs) const {
    bool published = true;
    if (outputs.has_detections) {
        const bool ok = publish_detections_(outputs.detections);
        if (!ok) {
            AERROR << "Shadow component failed to publish Detection2DArray.";
        }
        published = ok && published;
    }
    if (outputs.has_target) {
        const bool ok = publish_target_(outputs.target);
        if (!ok) {
            AERROR << "Shadow component failed to publish target PoseStamped.";
        }
        published = ok && published;
    }
    if (outputs.has_path) {
        const bool ok = publish_path_(outputs.path);
        if (!ok) {
            AERROR << "Shadow component failed to publish following Path.";
        }
        published = ok && published;
    }
    if (outputs.has_grid) {
        const bool ok = publish_grid_(outputs.grid);
        if (!ok) {
            AERROR << "Shadow component failed to publish GridMap.";
        }
        published = ok && published;
    }
    return published;
}

bool ShadowComponent::Proc(const std::shared_ptr<Image>& rgb,
                           const std::shared_ptr<Image>& depth,
                           const std::shared_ptr<CameraInfo>& camera_info,
                           const std::shared_ptr<Odometry>& odometry) {
    if (rgb == nullptr || depth == nullptr || camera_info == nullptr ||
        odometry == nullptr) {
        AERROR << "Shadow component received a null RGB, depth, CameraInfo, or "
                  "Odometry input.";
        return false;
    }
    if (!initialized_ || !now_ || !select_ || !confirmed_tracks_ ||
        !lookup_transform_ || !process_ || !publish_detections_ ||
        !publish_target_ || !publish_path_ || !publish_grid_) {
        AERROR << "Shadow component is not initialized.";
        return false;
    }

    int64_t stamp_ns = 0;
    std::string error;
    const InputStatus input_status = ValidateInputs(
        *rgb, *depth, *camera_info, *odometry, now_(), &stamp_ns, &error);
    if (input_status == InputStatus::kInvalid) {
        AERROR << "Shadow component rejected inputs: " << error;
        return false;
    }
    if (input_status == InputStatus::kStale) {
        AWARN << "Shadow component rejected stale inputs: " << error;
        FrameOutputs outputs;
        StampEmptyPath(*rgb, &outputs.path);
        outputs.has_path = true;
        PublishOutputs(outputs);
        return false;
    }

    Image canonical_depth;
    if (!CanonicalizeDepth(*depth, &canonical_depth, &error)) {
        AWARN << "Shadow component failed to canonicalize depth: " << error;
        FrameOutputs outputs;
        StampEmptyPath(*rgb, &outputs.path);
        outputs.has_path = true;
        PublishOutputs(outputs);
        return false;
    }

    ApplyPendingSelection();

    TransformStamped camera_to_map;
    if (!lookup_transform_(*rgb, &camera_to_map, &error) ||
        !ValidateTransform(camera_to_map, stamp_ns, &error)) {
        AWARN << "Shadow component cannot use camera-to-map TF: " << error;
        FrameOutputs outputs;
        StampEmptyPath(*rgb, &outputs.path);
        outputs.has_path = true;
        PublishOutputs(outputs);
        return false;
    }

    FrameOutputs outputs;
    const bool processed = process_(*rgb, canonical_depth, *camera_info,
                                    *odometry, camera_to_map, &outputs, &error);
    if (!processed) {
        AWARN << "Shadow component frame processing failed closed: " << error;
        StampEmptyPath(*rgb, &outputs.path);
        outputs.has_path = true;
    } else if (!outputs.has_path) {
        AERROR << "Shadow component processing omitted its path output.";
        StampEmptyPath(*rgb, &outputs.path);
        outputs.has_path = true;
        PublishOutputs(outputs);
        return false;
    }

    const bool published = PublishOutputs(outputs);
    return processed && published;
}

void ShadowComponent::Clear() {
    initialized_ = false;

    if (node_ != nullptr && !options_.select_topic().empty()) {
        node_->DeleteReader(options_.select_topic());
    }
    selection_reader_.reset();
    if (dynamic_tf_listener_ != nullptr) {
        dynamic_tf_listener_->Stop();
    }
    if (static_tf_listener_ != nullptr) {
        static_tf_listener_->Stop();
    }
    if (node_ != nullptr) {
        node_->DeleteReader("/tf");
        node_->DeleteReader("/tf_static");
    }
    dynamic_tf_listener_.reset();
    static_tf_listener_.reset();
    if (tf_buffer_ != nullptr) {
        tf_buffer_->clear();
    }
    tf_buffer_ = nullptr;

    publish_grid_ = nullptr;
    publish_path_ = nullptr;
    publish_target_ = nullptr;
    publish_detections_ = nullptr;
    process_ = nullptr;
    lookup_transform_ = nullptr;
    listener_start_ = nullptr;
    confirmed_tracks_ = nullptr;
    select_ = nullptr;
    now_ = nullptr;
    grid_writer_.reset();
    path_writer_.reset();
    target_writer_.reset();
    detections_writer_.reset();

    if (local_grid_ != nullptr) {
        local_grid_->Clear();
    }
    if (localizer_ != nullptr) {
        localizer_->Clear();
    }
    if (tracker_ != nullptr) {
        tracker_->Clear();
    }
    planner_.reset();
    policy_.reset();
    local_grid_.reset();
    localizer_.reset();
    tracker_.reset();
    detector_.reset();

    {
        std::lock_guard<std::mutex> lock(selection_mutex_);
        selection_pending_ = false;
        automatic_selection_pending_ = false;
        pending_selection_.clear();
    }
    options_.Clear();
}

}  // namespace shadow
}  // namespace perception
}  // namespace autonomy

AUTOLINK_REGISTER_COMPONENT(autonomy::perception::shadow::ShadowComponent)
