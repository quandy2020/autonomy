/* Copyright 2026 The Openbot Authors. */
/** @file observation_ingress.cpp
 * @brief Correlated measurement/localization admission without map mutation.
 */
#include "autonomy/perception/observation/observation_ingress.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace autonomy::perception {
namespace {
/** @brief Require the view to use the same attested acquisition transform. */
bool SameTransform(const RigidTransform& a, const RigidTransform& b) {
    if ((a.translation - b.translation).Norm() > 1e-9) return false;
    for (std::size_t i = 0; i < a.rotation.size(); ++i)
        if (std::abs(a.rotation[i] - b.rotation[i]) > 1e-9) return false;
    return true;
}
/** @brief Check bounded metric values without treating masked pixels as free. */
bool ValidDepth(const DepthView& view) {
    if (!view.camera.Valid() || !view.world_from_camera.Valid() ||
        view.valid.size() != view.axial_depth.size() ||
        view.valid.size() != static_cast<std::size_t>(view.camera.width) * view.camera.height)
        return false;
    for (std::size_t i = 0; i < view.valid.size(); ++i) {
        if (view.valid[i] > 1) return false;
        if (view.valid[i] && (!std::isfinite(view.axial_depth[i]) ||
            view.axial_depth[i] < view.camera.min_depth || view.axial_depth[i] > view.camera.max_depth))
            return false;
    }
    return true;
}
}  // namespace

bool AcquisitionLocalization::Valid(SensorKind sensor, const std::string& frame,
    double stamp, double now) const {
    return healthy && !world_frame.empty() && !sensor_frame.empty() && sensor_frame == frame &&
        map_epoch && calibration_revision && (support & Mask(sensor)) && !(support & ~SensorMask{3}) &&
        world_from_sensor.Valid() && world_from_base.Valid() && std::isfinite(stamp) &&
        std::isfinite(now) && stamp <= now + 1e-6 && std::isfinite(valid_from) &&
        std::isfinite(valid_until) && valid_from <= stamp && stamp <= valid_until && now <= valid_until;
}

bool ObservationPacket::Valid(double now) const {
    if ((observation.sensor != SensorKind::kLidar && observation.sensor != SensorKind::kRgbd) ||
        !observation.sequence || !observation.Valid(now) ||
        !localization.Valid(observation.sensor, localization.sensor_frame, observation.stamp, now) ||
        observation.map_epoch != localization.map_epoch || observation.frame_id != localization.world_frame ||
        observation.valid_until > localization.valid_until) return false;
    if (depth_view && (observation.sensor != SensorKind::kRgbd ||
        depth_view->sensor != observation.sensor || depth_view->sequence != observation.sequence ||
        depth_view->map_epoch != observation.map_epoch || depth_view->stamp != observation.stamp ||
        !ValidDepth(*depth_view) ||
        !SameTransform(depth_view->world_from_camera, localization.world_from_sensor)))
        return false;
    if (observation.sensor == SensorKind::kRgbd && !acquisition_poses.empty()) return false;
    if (observation.sensor == SensorKind::kLidar && acquisition_poses.empty()) return false;
    const AcquisitionPose* previous = nullptr;
    for (const auto& sample : acquisition_poses) {
        const auto& pose = sample.localization;
        if (!pose.Valid(observation.sensor, localization.sensor_frame, sample.stamp, now) ||
            sample.stamp > observation.stamp || pose.map_epoch != localization.map_epoch ||
            pose.calibration_revision != localization.calibration_revision ||
            pose.world_frame != localization.world_frame || pose.support != localization.support ||
            observation.valid_until > pose.valid_until) return false;
        if (previous && (sample.stamp < previous->stamp ||
            (sample.stamp == previous->stamp &&
             (!SameTransform(pose.world_from_sensor, previous->localization.world_from_sensor) ||
              !SameTransform(pose.world_from_base, previous->localization.world_from_base)))))
            return false;
        if (sample.stamp == observation.stamp &&
            (!SameTransform(pose.world_from_sensor, localization.world_from_sensor) ||
             !SameTransform(pose.world_from_base, localization.world_from_base))) return false;
        previous = &sample;
    }
    for (const auto& ray : observation.rays) {
        if (ray.stamp > observation.stamp) return false;
        if (observation.sensor == SensorKind::kRgbd) {
            if (ray.stamp != observation.stamp ||
                (ray.origin - localization.world_from_sensor.translation).Norm() > 1e-9)
                return false;
        } else {
            const auto proof = std::lower_bound(acquisition_poses.begin(), acquisition_poses.end(),
                ray.stamp, [](const AcquisitionPose& sample, double stamp) { return sample.stamp < stamp; });
            if (proof == acquisition_poses.end() || proof->stamp != ray.stamp ||
                (ray.origin - proof->localization.world_from_sensor.translation).Norm() > 1e-9)
                return false;
        }
    }
    return true;
}

bool LocalizedDepthView::Valid(double now) const {
    return view.sensor == SensorKind::kRgbd && view.sequence && view.map_epoch == localization.map_epoch &&
        localization.Valid(view.sensor, localization.sensor_frame, view.stamp, now) &&
        std::isfinite(valid_until) && view.stamp <= valid_until && now <= valid_until &&
        valid_until <= localization.valid_until && ValidDepth(view) &&
        SameTransform(view.world_from_camera, localization.world_from_sensor);
}

ObservationIngress::ObservationIngress(SensorMode mode, SensorDecodingOptions options,
    LocalizationLookup localization, Sink sink, Clock clock)
    : mode_(mode), options_(std::move(options)), localization_(std::move(localization)),
      sink_(std::move(sink)), clock_(std::move(clock)) {
    ValidateSensorDecodingOptions(options_);
    if ((mode != SensorMode::kLidarOnly && mode != SensorMode::kRgbdOnly && mode != SensorMode::kLidarRgbd) ||
        !localization_ || !sink_ || !clock_)
        throw std::invalid_argument("native observation ingress requires a valid mode and real dependencies");
}

void ObservationIngress::Close() noexcept { closed_.store(true); }

ObservationIngressResult ObservationIngress::Emit(ObservationPacket packet) const {
    if (closed_.load()) return ObservationIngressResult::kClosed;
    if (!packet.Valid(clock_())) return ObservationIngressResult::kInvalidEvidence;
    // The destination additionally owns generation/epoch admission. Closing this
    // source cannot undo an input already committed to that destination.
    if (closed_.load()) return ObservationIngressResult::kClosed;
    const double now = clock_();
    if (!packet.Valid(now)) return ObservationIngressResult::kInvalidEvidence;
    return sink_(std::move(packet), now) ? ObservationIngressResult::kAccepted :
        ObservationIngressResult::kSinkRejected;
}

ObservationIngressResult ObservationIngress::PushDepth(const DepthImageBytes& image,
    const DepthCalibration& calibration, std::uint64_t sequence) const {
    if (closed_.load()) return ObservationIngressResult::kClosed;
    if (!(Mask(mode_) & Mask(SensorKind::kRgbd))) return ObservationIngressResult::kWrongMode;
    try {
        auto frame = DecodeDepthImage(image, calibration, options_);
        if (!frame || !sequence) return ObservationIngressResult::kInvalidMessage;
        auto pose = localization_(SensorKind::kRgbd, image.frame_id, image.stamp);
        if (!pose || !pose->Valid(SensorKind::kRgbd, image.frame_id, image.stamp, clock_()) ||
            pose->calibration_revision != calibration.revision)
            return ObservationIngressResult::kInvalidLocalization;
        frame->world_from_camera = pose->world_from_sensor;
        frame->sequence = sequence; frame->map_epoch = pose->map_epoch;
        frame->localization_valid = true; frame->frame_id = pose->world_frame;
        auto observation = RgbdObservationAdapter{}.Adapt(*frame,options_.rgbd_pixel_stride);
        if (!observation) return ObservationIngressResult::kInvalidEvidence;
        observation->valid_until = std::min(observation->valid_until, pose->valid_until);
        ObservationPacket packet{*pose, std::move(*observation), RgbdObservationAdapter{}.MakeDepthView(*frame), {}};
        return Emit(std::move(packet));
    } catch (...) { return ObservationIngressResult::kDependencyFailure; }
}

ObservationIngressResult ObservationIngress::PushCloud(const PointCloudBytes& cloud,
    std::uint64_t sequence) const {
    if (closed_.load()) return ObservationIngressResult::kClosed;
    if (!(Mask(mode_) & Mask(SensorKind::kLidar))) return ObservationIngressResult::kWrongMode;
    try {
        auto scan = DecodePointCloud(cloud, options_);
        if (!scan || !sequence) return ObservationIngressResult::kInvalidMessage;
        auto end = localization_(SensorKind::kLidar, cloud.frame_id, scan->stamp + scan->duration);
        if (!end || !end->Valid(SensorKind::kLidar, cloud.frame_id, scan->stamp + scan->duration, clock_()))
            return ObservationIngressResult::kInvalidLocalization;
        scan->sequence = sequence; scan->map_epoch = end->map_epoch;
        scan->frame_id = end->world_frame; scan->localization_valid = true;
        bool incompatible_alignment = false;
        double valid_until = end->valid_until;
        std::vector<AcquisitionPose> acquisition_poses;
        auto observation = LidarObservationAdapter{}.Adapt(*scan, [&](double stamp) -> std::optional<RigidTransform> {
            auto pose = localization_(SensorKind::kLidar, cloud.frame_id, stamp);
            if (!pose || !pose->Valid(SensorKind::kLidar, cloud.frame_id, stamp, clock_())) return {};
            if (pose->map_epoch != end->map_epoch || pose->world_frame != end->world_frame ||
                pose->calibration_revision != end->calibration_revision || pose->support != end->support) {
                incompatible_alignment = true; return {};
            }
            valid_until = std::min(valid_until, pose->valid_until);
            acquisition_poses.push_back({stamp, *pose});
            return pose->world_from_sensor;
        });
        if (incompatible_alignment) return ObservationIngressResult::kInvalidLocalization;
        if (!observation) return ObservationIngressResult::kInvalidEvidence;
        observation->valid_until = std::min(observation->valid_until, valid_until);
        // PointCloud2 storage order is independent of actual beam acquisition time.
        std::stable_sort(acquisition_poses.begin(), acquisition_poses.end(),
            [](const AcquisitionPose& a, const AcquisitionPose& b) { return a.stamp < b.stamp; });
        return Emit(ObservationPacket{*end, std::move(*observation), {}, std::move(acquisition_poses)});
    } catch (...) { return ObservationIngressResult::kDependencyFailure; }
}
}  // namespace autonomy::perception
